/**
 * @file Cutter.cpp
 * @brief Main Cutter controller implementation
 */

#include "Cutter.h"
#include <cstring>

namespace Cutter {

Controller::Controller(ISerial* serial)
    : serial_(serial)
    , input_pos_(0)
    , response_buffer_{}
    , response_(response_buffer_, sizeof(response_buffer_))
    , next_seq_(1)
{
    // Initialize input buffer
    input_buffer_[0] = '\0';

    // Initialize pins to unconfigured
    for (size_t i = 0; i < NUM_PINS; i++) {
        pins_[i].mode = PinMode::UNCONFIGURED;
        pins_[i].pin_index = static_cast<uint8_t>(i);
    }

    // Initialize motors to unconfigured
    for (size_t i = 0; i < NUM_MOTORS; i++) {
        memset(&motors_[i], 0, sizeof(MotorSlot));
        motors_[i].type = MotorType::UNCONFIGURED;
        motors_[i].motor_index = static_cast<uint8_t>(i);
    }
}

void Controller::Update() {
    uint32_t now = CutterHal::Milliseconds();

    // Process incoming serial data
    ProcessInput();

    // Check pin states and generate events
    CheckPins();

    // Check motor states and generate events
    CheckMotors();

    (void)now;  // Will be used in CheckPins/CheckMotors
}

void Controller::ProcessInput() {
    // Read characters until newline or buffer full
    while (serial_->AvailableForRead() > 0) {
        int16_t c = serial_->CharGet();
        if (c < 0) break;

        // On first character, mark as connected
        if (state_machine_.GetState() == State::UNCONNECTED) {
            state_machine_.MarkConnected();
        }

        if (c == '\n' || c == '\r') {
            // End of line - process command
            if (input_pos_ > 0) {
                input_buffer_[input_pos_] = '\0';

                ParsedCommand cmd;
                if (parser_.Parse(input_buffer_, &cmd)) {
                    DispatchCommand(cmd);
                }
                // Empty/comment lines are silently ignored

                input_pos_ = 0;
            }
        } else if (input_pos_ < sizeof(input_buffer_) - 1) {
            input_buffer_[input_pos_++] = static_cast<char>(c);
        }
        // Overflow: continue reading but don't store
    }
}

void Controller::DispatchCommand(const ParsedCommand& cmd) {
    // Check epoch if provided
    if (cmd.has_epoch && cmd.epoch != state_machine_.GetEpoch()) {
        response_.Error(static_cast<uint32_t>(ErrorCode::EPOCH_MISMATCH),
                       "Epoch mismatch");
        if (cmd.has_seq) {
            response_.Param("seq", cmd.seq);
        }
        response_.Param("expected", state_machine_.GetEpoch());
        response_.Param("got", cmd.epoch);
        SendResponse();
        return;
    }

    // Built-in commands (available in any state)
    if (strcmp(cmd.name, "ping") == 0) {
        CmdPing(cmd);
        return;
    }
    if (strcmp(cmd.name, "reset") == 0) {
        CmdReset(cmd);
        return;
    }
    if (strcmp(cmd.name, "status") == 0) {
        CmdStatus(cmd);
        return;
    }
    if (strcmp(cmd.name, "version") == 0) {
        CmdVersion(cmd);
        return;
    }
    if (strcmp(cmd.name, "emergency_stop") == 0) {
        CmdEmergencyStop(cmd);
        return;
    }
    if (strcmp(cmd.name, "get_next_seq") == 0) {
        CmdGetNextSeq(cmd);
        return;
    }

    // Pin configuration commands (require CONNECTED or higher)
    if (strcmp(cmd.name, "configure_digital_in") == 0 ||
        strcmp(cmd.name, "configure_digital_out") == 0 ||
        strcmp(cmd.name, "configure_analog_in") == 0 ||
        strcmp(cmd.name, "configure_pwm") == 0 ||
        strcmp(cmd.name, "configure_hbridge") == 0 ||
        strcmp(cmd.name, "configure_endstop") == 0) {
        // Dispatch to pin command handlers (implemented in PinCommands.cpp)
        extern void DispatchPinCommand(Controller* ctrl, const ParsedCommand& cmd);
        DispatchPinCommand(this, cmd);
        return;
    }

    // Pin operation commands
    if (strcmp(cmd.name, "read_pin") == 0 ||
        strcmp(cmd.name, "write_pin") == 0 ||
        strcmp(cmd.name, "set_pwm") == 0 ||
        strcmp(cmd.name, "set_hbridge") == 0 ||
        strcmp(cmd.name, "start_tone") == 0 ||
        strcmp(cmd.name, "stop_tone") == 0) {
        extern void DispatchPinCommand(Controller* ctrl, const ParsedCommand& cmd);
        DispatchPinCommand(this, cmd);
        return;
    }

    // Motor configuration commands
    if (strcmp(cmd.name, "configure_sdsk") == 0 ||
        strcmp(cmd.name, "configure_stepper") == 0) {
        extern void DispatchMotorCommand(Controller* ctrl, const ParsedCommand& cmd);
        DispatchMotorCommand(this, cmd);
        return;
    }

    // Motor operation commands
    if (strcmp(cmd.name, "enable") == 0 ||
        strcmp(cmd.name, "disable") == 0 ||
        strcmp(cmd.name, "move") == 0 ||
        strcmp(cmd.name, "move_velocity") == 0 ||
        strcmp(cmd.name, "stop") == 0 ||
        strcmp(cmd.name, "home") == 0 ||
        strcmp(cmd.name, "set_position") == 0) {
        extern void DispatchMotorCommand(Controller* ctrl, const ParsedCommand& cmd);
        DispatchMotorCommand(this, cmd);
        return;
    }

    // Unknown command
    response_.Error(static_cast<uint32_t>(ErrorCode::UNKNOWN_COMMAND),
                   "Unknown command");
    if (cmd.has_seq) {
        response_.Param("seq", cmd.seq);
    }
    response_.Param("command", cmd.name);
    SendResponse();
}

void Controller::CheckPins() {
    uint32_t now = CutterHal::Milliseconds();

    for (size_t i = 0; i < NUM_PINS; i++) {
        PinSlot& pin = pins_[i];
        if (pin.mode == PinMode::UNCONFIGURED) continue;

        switch (pin.mode) {
            case PinMode::DIGITAL_IN: {
                bool raw_val = CutterHal::ReadDigitalPin(pin.pin_index);
                bool val = pin.digital_in.invert ? !raw_val : raw_val;

                // Check error trigger
                if (pin.digital_in.error_trigger_enabled && val == pin.digital_in.error_trigger_value) {
                    state_machine_.EnterError(ErrorCode::PIN_ERROR_TRIGGER, "Digital input error trigger");
                    response_.Event("error")
                        .Param("code", static_cast<uint32_t>(ErrorCode::PIN_ERROR_TRIGGER))
                        .Param("pin", static_cast<int32_t>(pin.pin_index))
                        .Param("message", "Error trigger activated");
                    SendResponse();
                    // Stop all motors on error
                    for (size_t j = 0; j < NUM_MOTORS; j++) {
                        if (motors_[j].type != MotorType::UNCONFIGURED) {
                            CutterHal::StopMotor(motors_[j].motor_index, true);
                            motors_[j].moving = false;
                        }
                    }
                    // Apply on_error values to digital outputs
                    for (size_t j = 0; j < NUM_PINS; j++) {
                        if (pins_[j].mode == PinMode::DIGITAL_OUT && pins_[j].digital_out.on_error_enabled) {
                            CutterHal::WriteDigitalPin(pins_[j].pin_index, pins_[j].digital_out.on_error_value);
                        }
                    }
                    return;
                }

                // Report changes
                if (pin.digital_in.report_changes && val != pin.digital_in.last_value) {
                    pin.digital_in.last_value = val;
                    response_.Event("input")
                        .Param("pin", static_cast<int32_t>(pin.pin_index))
                        .Param("value", val);
                    SendResponse();
                }
                break;
            }

            case PinMode::DIGITAL_OUT: {
                // Check hardware fault (overcurrent)
                if (CutterHal::IsPinInFault(pin.pin_index)) {
                    state_machine_.EnterError(ErrorCode::PIN_OVERCURRENT, "Pin overcurrent");
                    response_.Event("error")
                        .Param("code", static_cast<uint32_t>(ErrorCode::PIN_OVERCURRENT))
                        .Param("pin", static_cast<int32_t>(pin.pin_index))
                        .Param("message", "Pin overcurrent fault");
                    SendResponse();
                    // Stop all motors on error
                    for (size_t j = 0; j < NUM_MOTORS; j++) {
                        if (motors_[j].type != MotorType::UNCONFIGURED) {
                            CutterHal::StopMotor(motors_[j].motor_index, true);
                            motors_[j].moving = false;
                        }
                    }
                    return;
                }

                // Check timeout
                if (pin.digital_out.max_raised_ms > 0 && pin.digital_out.current_value) {
                    if ((now - pin.digital_out.raise_start_time) > pin.digital_out.max_raised_ms) {
                        state_machine_.EnterError(ErrorCode::PIN_TIMEOUT, "Digital output timeout");
                        response_.Event("error")
                            .Param("code", static_cast<uint32_t>(ErrorCode::PIN_TIMEOUT))
                            .Param("pin", static_cast<int32_t>(pin.pin_index))
                            .Param("message", "Output timeout");
                        SendResponse();
                        // Stop all motors on error
                        for (size_t j = 0; j < NUM_MOTORS; j++) {
                            if (motors_[j].type != MotorType::UNCONFIGURED) {
                                CutterHal::StopMotor(motors_[j].motor_index, true);
                                motors_[j].moving = false;
                            }
                        }
                        return;
                    }
                }
                break;
            }

            case PinMode::ANALOG_IN: {
                int16_t val = CutterHal::ReadAnalogPin(pin.pin_index);

                // Check error thresholds
                if (pin.analog_in.error_threshold_enabled) {
                    if (val < pin.analog_in.error_threshold_low || val > pin.analog_in.error_threshold_high) {
                        state_machine_.EnterError(ErrorCode::ANALOG_THRESHOLD, "Analog error threshold");
                        response_.Event("error")
                            .Param("code", static_cast<uint32_t>(ErrorCode::ANALOG_THRESHOLD))
                            .Param("pin", static_cast<int32_t>(pin.pin_index))
                            .Param("value", static_cast<int32_t>(val))
                            .Param("message", "Analog threshold exceeded");
                        SendResponse();
                        // Stop all motors on error
                        for (size_t j = 0; j < NUM_MOTORS; j++) {
                            if (motors_[j].type != MotorType::UNCONFIGURED) {
                                CutterHal::StopMotor(motors_[j].motor_index, true);
                                motors_[j].moving = false;
                            }
                        }
                        return;
                    }
                }

                // Periodic reporting
                if (pin.analog_in.report_interval_ms > 0) {
                    if ((now - pin.analog_in.last_report_time) >= pin.analog_in.report_interval_ms) {
                        pin.analog_in.last_report_time = now;
                        response_.Event("analog")
                            .Param("pin", static_cast<int32_t>(pin.pin_index))
                            .Param("value", static_cast<int32_t>(val));
                        SendResponse();
                    }
                }

                pin.analog_in.last_value = val;
                break;
            }

            default:
                break;
        }
    }
}

void Controller::CheckMotors() {
    uint32_t now = CutterHal::Milliseconds();

    for (size_t i = 0; i < NUM_MOTORS; i++) {
        MotorSlot& motor = motors_[i];
        if (motor.type == MotorType::UNCONFIGURED) continue;

        // Check for hardware fault
        if (CutterHal::IsMotorInFault(motor.motor_index)) {
            state_machine_.EnterError(ErrorCode::MOTOR_FAULT, "Motor hardware fault");
            response_.Event("error")
                .Param("code", static_cast<uint32_t>(ErrorCode::MOTOR_FAULT))
                .Param("motor", static_cast<int32_t>(motor.motor_index))
                .Param("message", "Motor hardware fault");
            SendResponse();
            // Stop all motors
            for (size_t j = 0; j < NUM_MOTORS; j++) {
                if (motors_[j].type != MotorType::UNCONFIGURED) {
                    CutterHal::StopMotor(motors_[j].motor_index, true);
                    motors_[j].moving = false;
                }
            }
            return;
        }

        // Check for HLFB state changes (ClearPath only)
        if (motor.type == MotorType::CLEARPATH && motor.enabled) {
            uint8_t hlfb_state = CutterHal::GetHlfbState(motor.motor_index);
            if (hlfb_state != motor.last_hlfb_state) {
                motor.last_hlfb_state = hlfb_state;
                response_.Event("hlfb")
                    .Param("motor", static_cast<int32_t>(motor.motor_index))
                    .Param("state", static_cast<int32_t>(hlfb_state));
                SendResponse();
            }
        }

        // Check for HLFB timeout during enabling
        if (state_machine_.GetState() == State::ENABLING && motor.enabled) {
            if (motor.type == MotorType::CLEARPATH) {
                if (CutterHal::IsMotorReady(motor.motor_index)) {
                    // Motor is ready - transition to READY if all motors ready
                    bool all_ready = true;
                    for (size_t j = 0; j < NUM_MOTORS; j++) {
                        if (motors_[j].type != MotorType::UNCONFIGURED &&
                            motors_[j].enabled &&
                            !CutterHal::IsMotorReady(motors_[j].motor_index)) {
                            all_ready = false;
                            break;
                        }
                    }
                    if (all_ready) {
                        state_machine_.TransitionTo(State::READY);
                    }
                } else if ((now - motor.enable_start_time) > motor.hlfb_timeout_ms) {
                    // HLFB timeout
                    state_machine_.EnterError(ErrorCode::HLFB_TIMEOUT,
                                             "Motor HLFB timeout");
                    response_.Event("error")
                        .Param("code", static_cast<uint32_t>(ErrorCode::HLFB_TIMEOUT))
                        .Param("motor", static_cast<int32_t>(motor.motor_index));
                    SendResponse();
                }
            }
        }

        // Check for move completion
        if (motor.moving && CutterHal::StepsComplete(motor.motor_index)) {
            motor.moving = false;
            response_.Event("done")
                .Param("motor", static_cast<int32_t>(motor.motor_index))
                .Param("seq", motor.move_seq)
                .Param("position", CutterHal::GetMotorPosition(motor.motor_index));
            SendResponse();

            // If no motors are moving, transition back to READY
            bool any_moving = false;
            for (size_t j = 0; j < NUM_MOTORS; j++) {
                if (motors_[j].moving) {
                    any_moving = true;
                    break;
                }
            }
            if (!any_moving && state_machine_.GetState() == State::WORKING) {
                state_machine_.TransitionTo(State::READY);
            }
        }

        // Check homing state for generic steppers
        if (motor.type == MotorType::GENERIC_STEPPER &&
            motor.homing_state != HomingState::IDLE &&
            motor.homing_state != HomingState::COMPLETE) {
            // Homing logic handled in MotorCommands.cpp
            extern void CheckHomingState(Controller* ctrl, MotorSlot& motor);
            CheckHomingState(this, motor);
        }
    }
}

void Controller::SendEvent(const char* type) {
    response_.Event(type);
    SendResponse();
}

void Controller::SendResponse() {
    const char* resp = response_.Finish();
    serial_->Send(resp);
    response_.Reset();
}

PinSlot* Controller::GetPin(uint8_t index) {
    if (index >= NUM_PINS) return nullptr;
    return &pins_[index];
}

const PinSlot* Controller::GetPin(uint8_t index) const {
    if (index >= NUM_PINS) return nullptr;
    return &pins_[index];
}

MotorSlot* Controller::GetMotor(uint8_t index) {
    if (index >= NUM_MOTORS) return nullptr;
    return &motors_[index];
}

const MotorSlot* Controller::GetMotor(uint8_t index) const {
    if (index >= NUM_MOTORS) return nullptr;
    return &motors_[index];
}

// Built-in commands

void Controller::CmdPing(const ParsedCommand& cmd) {
    response_.Ok();
    if (cmd.has_seq) {
        response_.Param("seq", cmd.seq);
    }
    SendResponse();
}

void Controller::CmdReset(const ParsedCommand& cmd) {
    if (state_machine_.GetState() == State::ERROR) {
        state_machine_.Reset();

        // Reset all pins to unconfigured
        for (size_t i = 0; i < NUM_PINS; i++) {
            pins_[i].mode = PinMode::UNCONFIGURED;
        }

        // Reset all motors
        for (size_t i = 0; i < NUM_MOTORS; i++) {
            if (motors_[i].type != MotorType::UNCONFIGURED) {
                CutterHal::EnableMotor(motors_[i].motor_index, false);
            }
            memset(&motors_[i], 0, sizeof(MotorSlot));
            motors_[i].type = MotorType::UNCONFIGURED;
            motors_[i].motor_index = static_cast<uint8_t>(i);
        }

        response_.Ok();
        if (cmd.has_seq) {
            response_.Param("seq", cmd.seq);
        }
        response_.Param("epoch", state_machine_.GetEpoch());
    } else {
        response_.Error(static_cast<uint32_t>(ErrorCode::INVALID_STATE),
                       "Not in error state");
        if (cmd.has_seq) {
            response_.Param("seq", cmd.seq);
        }
    }
    SendResponse();
}

void Controller::CmdStatus(const ParsedCommand& cmd) {
    response_.Ok();
    if (cmd.has_seq) {
        response_.Param("seq", cmd.seq);
    }
    response_.Param("state", StateName(state_machine_.GetState()));
    response_.Param("epoch", state_machine_.GetEpoch());

    if (state_machine_.GetState() == State::ERROR) {
        response_.Param("error_code", static_cast<uint32_t>(state_machine_.GetErrorCode()));
        response_.Param("error_message", state_machine_.GetErrorMessage());
    }
    SendResponse();
}

void Controller::CmdVersion(const ParsedCommand& cmd) {
    response_.Ok();
    if (cmd.has_seq) {
        response_.Param("seq", cmd.seq);
    }
    response_.Param("version", CUTTER_VERSION);
    response_.Param("protocol", PROTOCOL_VERSION);
    SendResponse();
}

void Controller::CmdEmergencyStop(const ParsedCommand& cmd) {
    // Stop all motors immediately
    for (size_t i = 0; i < NUM_MOTORS; i++) {
        if (motors_[i].type != MotorType::UNCONFIGURED) {
            CutterHal::StopMotor(motors_[i].motor_index, true);  // immediate stop
            motors_[i].moving = false;
            motors_[i].homing_state = HomingState::IDLE;
        }
    }

    // Enter error state
    state_machine_.EnterError(ErrorCode::EMERGENCY_STOP, "Emergency stop");

    // Send response
    response_.Ok();
    if (cmd.has_seq) {
        response_.Param("seq", cmd.seq);
    }
    response_.Param("epoch", state_machine_.GetEpoch());
    SendResponse();

    // Send error event
    response_.Event("error")
        .Param("code", static_cast<uint32_t>(ErrorCode::EMERGENCY_STOP))
        .Param("message", "Emergency stop activated");
    SendResponse();
}

void Controller::CmdGetNextSeq(const ParsedCommand& cmd) {
    response_.Ok();
    if (cmd.has_seq) {
        response_.Param("seq", cmd.seq);
    }
    response_.Param("next_seq", next_seq_);
    SendResponse();
}

}  // namespace Cutter
