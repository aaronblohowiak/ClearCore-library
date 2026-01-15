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
    if (strcmp(cmd.name, "configure_motor") == 0) {
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
                if (pin.digital_in.report_changes) {
                    bool val = CutterHal::ReadDigitalPin(pin.pin_index);
                    if (val != pin.digital_in.last_value) {
                        pin.digital_in.last_value = val;
                        response_.Event("input")
                            .Param("pin", static_cast<int32_t>(pin.pin_index))
                            .Param("value", val);
                        SendResponse();
                    }
                }
                break;
            }

            case PinMode::ANALOG_IN: {
                if (pin.analog_in.report_threshold) {
                    if ((now - pin.analog_in.last_sample_time) >= pin.analog_in.sample_interval_ms) {
                        pin.analog_in.last_sample_time = now;
                        int16_t val = CutterHal::ReadAnalogPin(pin.pin_index);

                        bool was_in_range = (pin.analog_in.last_value >= pin.analog_in.threshold_low &&
                                            pin.analog_in.last_value <= pin.analog_in.threshold_high);
                        bool is_in_range = (val >= pin.analog_in.threshold_low &&
                                           val <= pin.analog_in.threshold_high);

                        if (was_in_range != is_in_range) {
                            response_.Event("threshold")
                                .Param("pin", static_cast<int32_t>(pin.pin_index))
                                .Param("value", static_cast<int32_t>(val))
                                .Param("in_range", is_in_range);
                            SendResponse();
                        }
                        pin.analog_in.last_value = val;
                    }
                }
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

}  // namespace Cutter
