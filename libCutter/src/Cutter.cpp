/**
 * @file Cutter.cpp
 * @brief Main Cutter controller implementation
 */

#include "Cutter.h"
#include <cstring>

namespace Cutter {

Controller::Controller(ISerial* serial)
    : m_serial(serial)
    , m_inputPos(0)
    , m_inputOverflow(false)
    , m_responseBuffer{}
    , m_response(m_responseBuffer, sizeof(m_responseBuffer))
    , m_nextSeq(1)
    , m_maxSeenSeq(0)
    , m_seenAnySeq(false)
    , m_currentCommandId{}
    , m_enableAllActive(false)
    , m_enableAllId{}
    , m_homingOrder{}
    , m_homingCount(0)
    , m_currentHomingIndex(0)
    , m_wasPortOpen(false)
    , m_bannerSent(false)
    , m_debugWaitActive(false)
    , m_debugWaitMotor(0xFF)
    , m_debugWaitStart(0)
    , m_debugWaitTimeoutMs(0)
    , m_debugWaitId{}
{
    // Initialize input buffer
    m_inputBuffer[0] = '\0';

    // Initialize pins to unconfigured
    for (size_t i = 0; i < NUM_PINS; i++) {
        m_pins[i].mode = PinMode::UNCONFIGURED;
        m_pins[i].pin_index = static_cast<uint8_t>(i);
    }

    // Initialize motors to unconfigured
    for (size_t i = 0; i < NUM_MOTORS; i++) {
        memset(&m_motors[i], 0, sizeof(MotorSlot));
        m_motors[i].type = MotorType::UNCONFIGURED;
        m_motors[i].motor_index = static_cast<uint8_t>(i);
    }
}

void Controller::Update() {
    uint32_t now = CutterHal::Milliseconds();

    // Detect a host (re)connection from the serial port-open edge. On USB this
    // tracks the virtual-port DTR flag, so the banner is sent the moment the
    // host opens the port - no need for the host to send a byte first.
    bool portOpen = m_serial->PortIsOpen();
    if (portOpen && !m_wasPortOpen) {
        OnHostConnected();
    } else if (!portOpen && m_wasPortOpen) {
        // Host closed the port; re-arm the banner for the next connection.
        m_bannerSent = false;
    }
    m_wasPortOpen = portOpen;

    // Resolve a pending debug_wait (DEBUG-ONLY). While active, new serial input
    // is NOT processed - the move in flight runs to completion first - so a
    // pasted script steps through one move at a time. CheckMotors (below) still
    // runs every tick, so the previous tick may already have cleared `moving`
    // (and emitted the move's done/alert) by the time we get here.
    if (m_debugWaitActive) {
        bool moving = (m_debugWaitMotor == 0xFF)
                          ? AnyMotorMoving()
                          : m_motors[m_debugWaitMotor].moving;
        if (!moving) {
            m_debugWaitActive = false;
            m_response.Ok(m_debugWaitId);
            if (m_debugWaitMotor != 0xFF) {
                m_response.Param("motor", static_cast<int32_t>(m_debugWaitMotor));
                m_response.Param("position",
                                 CutterHal::GetMotorPosition(m_debugWaitMotor));
            }
            SendResponse();
        } else if ((now - m_debugWaitStart) > m_debugWaitTimeoutMs) {
            m_debugWaitActive = false;
            m_response.Error(static_cast<uint32_t>(ErrorCode::DEBUG_WAIT_TIMEOUT),
                             "debug_wait timed out", m_debugWaitId);
            if (m_debugWaitMotor != 0xFF) {
                m_response.Param("motor", static_cast<int32_t>(m_debugWaitMotor));
            }
            SendResponse();
        }
    }

    // Process incoming serial data (gated while a debug_wait is pending)
    if (!m_debugWaitActive) {
        ProcessInput();
    }

    // Check pin states and generate events
    CheckPins();

    // Check motor states and generate events
    CheckMotors();
}

void Controller::ProcessInput() {
    // Read characters until newline or buffer full
    while (m_serial->AvailableForRead() > 0) {
        int16_t c = m_serial->CharGet();
        if (c < 0) break;

        // Fallback for transports that do not report a port-open edge (e.g. a
        // terminal that never asserts DTR): treat the first received byte as
        // the connection. OnHostConnected() is idempotent, so this is a no-op
        // when the port-open edge already greeted the host.
        if (m_stateMachine.GetState() == State::UNCONNECTED) {
            OnHostConnected();
        }

        if (c == '\n' || c == '\r') {
            // End of line - process command or report overflow
            if (m_inputOverflow) {
                // Line exceeded buffer - report error to host
                m_response.Error(static_cast<uint32_t>(ErrorCode::INPUT_OVERFLOW),
                               "Command too long");
                m_response.Param("max_length", static_cast<int32_t>(MAX_COMMAND_LENGTH - 1));
                SendResponse();
                m_inputOverflow = false;
                m_inputPos = 0;
            } else if (m_inputPos > 0) {
                m_inputBuffer[m_inputPos] = '\0';

                ParsedCommand cmd;
                if (m_parser.Parse(m_inputBuffer, &cmd)) {
                    DispatchCommand(cmd);
                }
                // Empty/comment lines are silently ignored

                m_inputPos = 0;

                // If the just-dispatched command armed a debug_wait, stop
                // draining: any remaining pasted lines must wait in the RX
                // buffer until the move completes (see Update()).
                if (m_debugWaitActive) {
                    break;
                }
            }
        } else if (m_inputPos < sizeof(m_inputBuffer) - 1) {
            m_inputBuffer[m_inputPos++] = static_cast<char>(c);
        } else {
            // Buffer overflow - mark and continue discarding until newline
            m_inputOverflow = true;
        }
    }
}

void Controller::DispatchCommand(const ParsedCommand& cmd) {
    // Check epoch if provided
    if (cmd.has_epoch && cmd.epoch != m_stateMachine.GetEpoch()) {
        m_response.Error(static_cast<uint32_t>(ErrorCode::EPOCH_MISMATCH),
                       "Epoch mismatch");
        if (cmd.has_seq) {
            m_response.Param("seq", cmd.seq);
        }
        m_response.Param("expected", m_stateMachine.GetEpoch());
        m_response.Param("got", cmd.epoch);
        SendResponse();
        return;
    }

    // Check user-supplied seq for staleness (using serial number arithmetic for wrap-around)
    // A seq is stale if it's <= the max we've seen (accounting for wrap)
    // User-supplied seq/epoch are for verification/sync only, not command identification
    if (cmd.has_seq) {
        if (m_seenAnySeq) {
            int32_t diff = static_cast<int32_t>(cmd.seq - m_maxSeenSeq);
            if (diff <= 0) {
                m_response.Error(static_cast<uint32_t>(ErrorCode::STALE_SEQ),
                               "Sequence number already used");
                m_response.Param("seq", cmd.seq);
                m_response.Param("max_seen", m_maxSeenSeq);
                SendResponse();
                return;
            }
        }
        m_maxSeenSeq = cmd.seq;
        m_seenAnySeq = true;
    }

    // Assign internal command ID before processing
    // Every command gets a unique internal seq and the current epoch
    m_currentCommandId.epoch = m_stateMachine.GetEpoch();
    m_currentCommandId.seq = m_nextSeq;
    m_nextSeq++;

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
    if (strcmp(cmd.name, "debug_wait") == 0) {
        CmdDebugWait(cmd);
        return;
    }
    if (strcmp(cmd.name, "get_next_seq") == 0) {
        CmdGetNextSeq(cmd);
        return;
    }
    if (strcmp(cmd.name, "get_status") == 0) {
        CmdGetStatus(cmd);
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
        strcmp(cmd.name, "enable_all") == 0 ||
        strcmp(cmd.name, "disable") == 0 ||
        strcmp(cmd.name, "move") == 0 ||
        strcmp(cmd.name, "move_velocity") == 0 ||
        strcmp(cmd.name, "move_until") == 0 ||
        strcmp(cmd.name, "stop") == 0 ||
        strcmp(cmd.name, "home") == 0 ||
        strcmp(cmd.name, "set_position") == 0 ||
        strcmp(cmd.name, "set_motor_clock") == 0 ||
        strcmp(cmd.name, "configure_estop") == 0 ||
        strcmp(cmd.name, "clear_alerts") == 0) {
        extern void DispatchMotorCommand(Controller* ctrl, const ParsedCommand& cmd);
        DispatchMotorCommand(this, cmd);
        return;
    }

    // Unknown command
    m_response.Error(static_cast<uint32_t>(ErrorCode::UNKNOWN_COMMAND),
                   "Unknown command");
    if (cmd.has_seq) {
        m_response.Param("seq", cmd.seq);
    }
    m_response.Param("command", cmd.name);
    SendResponse();
}

void Controller::CheckPins() {
    uint32_t now = CutterHal::Milliseconds();

    for (size_t i = 0; i < NUM_PINS; i++) {
        PinSlot& pin = m_pins[i];
        if (pin.mode == PinMode::UNCONFIGURED) continue;

        switch (pin.mode) {
            case PinMode::DIGITAL_IN: {
                bool raw_val = CutterHal::ReadDigitalPin(pin.pin_index);
                bool val = pin.digital_in.invert ? !raw_val : raw_val;

                // Check error trigger
                if (pin.digital_in.error_trigger_enabled && val == pin.digital_in.error_trigger_value) {
                    m_stateMachine.EnterError(ErrorCode::PIN_ERROR_TRIGGER, "Digital input error trigger");
                    m_response.Event("error", pin.digital_in.config_id)
                        .Param("code", static_cast<uint32_t>(ErrorCode::PIN_ERROR_TRIGGER))
                        .Param("pin", static_cast<int32_t>(pin.pin_index))
                        .Param("message", "Error trigger activated");
                    SendResponse();
                    // Stop all motors on error
                    for (size_t j = 0; j < NUM_MOTORS; j++) {
                        if (m_motors[j].type != MotorType::UNCONFIGURED) {
                            CutterHal::StopMotor(m_motors[j].motor_index, true);
                            m_motors[j].moving = false;
                        }
                    }
                    // Apply on_error values to digital outputs
                    for (size_t j = 0; j < NUM_PINS; j++) {
                        if (m_pins[j].mode == PinMode::DIGITAL_OUT && m_pins[j].digital_out.on_error_enabled) {
                            CutterHal::WriteDigitalPin(m_pins[j].pin_index, m_pins[j].digital_out.on_error_value);
                        }
                    }
                    return;
                }

                // Report changes
                if (pin.digital_in.report_changes && val != pin.digital_in.last_value) {
                    pin.digital_in.last_value = val;
                    m_response.Event("input", pin.digital_in.config_id)
                        .Param("pin", static_cast<int32_t>(pin.pin_index))
                        .Param("value", val);
                    SendResponse();
                }

                // Check for edge events (using hardware edge detection)
                if (pin.digital_in.report_edges != EdgeMode::NONE) {
                    bool risen = CutterHal::InputRisen(pin.pin_index);
                    bool fallen = CutterHal::InputFallen(pin.pin_index);

                    // Apply invert to edge direction
                    if (pin.digital_in.invert) {
                        bool tmp = risen;
                        risen = fallen;
                        fallen = tmp;
                    }

                    if (risen && (pin.digital_in.report_edges == EdgeMode::RISING ||
                                  pin.digital_in.report_edges == EdgeMode::BOTH)) {
                        m_response.Event("edge", pin.digital_in.config_id)
                            .Param("pin", static_cast<int32_t>(pin.pin_index))
                            .Param("direction", "rising")
                            .Param("value", true);
                        SendResponse();
                    }
                    if (fallen && (pin.digital_in.report_edges == EdgeMode::FALLING ||
                                   pin.digital_in.report_edges == EdgeMode::BOTH)) {
                        m_response.Event("edge", pin.digital_in.config_id)
                            .Param("pin", static_cast<int32_t>(pin.pin_index))
                            .Param("direction", "falling")
                            .Param("value", false);
                        SendResponse();
                    }
                }

                // Update last_value for next iteration (even if not reporting changes)
                pin.digital_in.last_value = val;
                break;
            }

            case PinMode::DIGITAL_OUT: {
                // Check hardware fault (overcurrent)
                if (CutterHal::IsPinInFault(pin.pin_index)) {
                    m_stateMachine.EnterError(ErrorCode::PIN_OVERCURRENT, "Pin overcurrent");
                    m_response.Event("error", pin.digital_out.config_id)
                        .Param("code", static_cast<uint32_t>(ErrorCode::PIN_OVERCURRENT))
                        .Param("pin", static_cast<int32_t>(pin.pin_index))
                        .Param("message", "Pin overcurrent fault");
                    SendResponse();
                    // Stop all motors on error
                    for (size_t j = 0; j < NUM_MOTORS; j++) {
                        if (m_motors[j].type != MotorType::UNCONFIGURED) {
                            CutterHal::StopMotor(m_motors[j].motor_index, true);
                            m_motors[j].moving = false;
                        }
                    }
                    return;
                }

                // Check timeout
                if (pin.digital_out.max_raised_ms > 0 && pin.digital_out.current_value) {
                    if ((now - pin.digital_out.raise_start_time) > pin.digital_out.max_raised_ms) {
                        // Auto-lower the pin and send notification (not an error)
                        CutterHal::WriteDigitalPin(pin.pin_index, false);
                        pin.digital_out.current_value = false;
                        pin.digital_out.max_raised_ms = 0;  // Clear timeout

                        m_response.Event("pin_timeout", pin.digital_out.set_id)
                            .Param("pin", static_cast<int32_t>(pin.pin_index));
                        SendResponse();
                    }
                }
                break;
            }

            case PinMode::ANALOG_IN: {
                int16_t val = CutterHal::ReadAnalogPin(pin.pin_index);

                // Check error thresholds
                if (pin.analog_in.error_threshold_enabled) {
                    if (val < pin.analog_in.error_threshold_low || val > pin.analog_in.error_threshold_high) {
                        m_stateMachine.EnterError(ErrorCode::ANALOG_THRESHOLD, "Analog error threshold");
                        m_response.Event("error", pin.analog_in.config_id)
                            .Param("code", static_cast<uint32_t>(ErrorCode::ANALOG_THRESHOLD))
                            .Param("pin", static_cast<int32_t>(pin.pin_index))
                            .Param("value", static_cast<int32_t>(val))
                            .Param("message", "Analog threshold exceeded");
                        SendResponse();
                        // Stop all motors on error
                        for (size_t j = 0; j < NUM_MOTORS; j++) {
                            if (m_motors[j].type != MotorType::UNCONFIGURED) {
                                CutterHal::StopMotor(m_motors[j].motor_index, true);
                                m_motors[j].moving = false;
                            }
                        }
                        return;
                    }
                }

                // Periodic reporting
                if (pin.analog_in.report_interval_ms > 0) {
                    if ((now - pin.analog_in.last_report_time) >= pin.analog_in.report_interval_ms) {
                        pin.analog_in.last_report_time = now;
                        m_response.Event("analog", pin.analog_in.config_id)
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
        MotorSlot& motor = m_motors[i];
        if (motor.type == MotorType::UNCONFIGURED) continue;

        // Check for hardware fault
        if (CutterHal::IsMotorInFault(motor.motor_index)) {
            m_stateMachine.EnterError(ErrorCode::MOTOR_FAULT, "Motor hardware fault");
            m_response.Event("error", motor.enable_id)
                .Param("code", static_cast<uint32_t>(ErrorCode::MOTOR_FAULT))
                .Param("motor", static_cast<int32_t>(motor.motor_index))
                .Param("message", "Motor hardware fault");
            SendResponse();
            // Stop all motors
            for (size_t j = 0; j < NUM_MOTORS; j++) {
                if (m_motors[j].type != MotorType::UNCONFIGURED) {
                    CutterHal::StopMotor(m_motors[j].motor_index, true);
                    m_motors[j].moving = false;
                    m_motors[j].stop_sensor_pin = CutterHal::PIN_INVALID;
                }
            }
            return;
        }

        // Report limit switch state changes. Whenever a configured limit
        // input transitions (pressed or released), emit a "limit" event with
        // the new value so the host always knows the endstop state - even
        // while idle or homing. The move-cancellation handling below relies on
        // the hardware alert, not on this report.
        {
            bool pos_now = CutterHal::InPosLimit(motor.motor_index);
            if (pos_now != motor.last_pos_limit) {
                motor.last_pos_limit = pos_now;
                m_response.Event("limit", motor.move_id)
                    .Param("motor", static_cast<int32_t>(motor.motor_index))
                    .Param("direction", "pos")
                    .Param("value", pos_now)
                    .Param("position", CutterHal::GetMotorPosition(motor.motor_index));
                SendResponse();
            }
            bool neg_now = CutterHal::InNegLimit(motor.motor_index);
            if (neg_now != motor.last_neg_limit) {
                motor.last_neg_limit = neg_now;
                m_response.Event("limit", motor.move_id)
                    .Param("motor", static_cast<int32_t>(motor.motor_index))
                    .Param("direction", "neg")
                    .Param("value", neg_now)
                    .Param("position", CutterHal::GetMotorPosition(motor.motor_index));
                SendResponse();
            }
        }

        // Check for HLFB state changes (ClearPath only)
        if (motor.type == MotorType::CLEARPATH && motor.enabled) {
            uint8_t hlfb_state = CutterHal::GetHlfbState(motor.motor_index);
            if (hlfb_state != motor.last_hlfb_state) {
                motor.last_hlfb_state = hlfb_state;
                m_response.Event("hlfb", motor.enable_id)
                    .Param("motor", static_cast<int32_t>(motor.motor_index))
                    .Param("state", static_cast<int32_t>(hlfb_state));
                SendResponse();
            }
        }

        // Check for HLFB timeout during enabling
        if (m_stateMachine.GetState() == State::ENABLING && motor.enabled) {
            if (motor.type == MotorType::CLEARPATH) {
                if (CutterHal::IsMotorReady(motor.motor_index)) {
                    // Motor is ready - check if all motors ready
                    bool all_ready = true;
                    for (size_t j = 0; j < NUM_MOTORS; j++) {
                        if (m_motors[j].type == MotorType::CLEARPATH &&
                            m_motors[j].enabled &&
                            !CutterHal::IsMotorReady(m_motors[j].motor_index)) {
                            all_ready = false;
                            break;
                        }
                    }
                    if (all_ready) {
                        // All SDSK motors ready - transition to READY
                        m_stateMachine.TransitionTo(State::READY);

                        // If enable_all is active, start sequential homing
                        if (m_enableAllActive) {
                            StartNextHoming();
                        }
                    }
                } else if ((now - motor.enable_start_time) > motor.hlfb_timeout_ms) {
                    // HLFB timeout
                    m_stateMachine.EnterError(ErrorCode::HLFB_TIMEOUT,
                                             "Motor HLFB timeout");
                    m_response.Event("error", motor.enable_id)
                        .Param("code", static_cast<uint32_t>(ErrorCode::HLFB_TIMEOUT))
                        .Param("motor", static_cast<int32_t>(motor.motor_index));
                    SendResponse();
                    m_enableAllActive = false;  // Cancel enable_all on error
                }
            } else if (motor.type == MotorType::GENERIC_STEPPER) {
                // Generic steppers don't have HLFB - check if all SDSK motors ready
                bool all_sdsk_ready = true;
                for (size_t j = 0; j < NUM_MOTORS; j++) {
                    if (m_motors[j].type == MotorType::CLEARPATH &&
                        m_motors[j].enabled &&
                        !CutterHal::IsMotorReady(m_motors[j].motor_index)) {
                        all_sdsk_ready = false;
                        break;
                    }
                }
                if (all_sdsk_ready) {
                    m_stateMachine.TransitionTo(State::READY);
                    if (m_enableAllActive) {
                        StartNextHoming();
                    }
                }
            }
        }

        // Check for E-Stop trigger - puts the motor into the alert state.
        // Reported via the unified "alert" event (cause=estop). This is
        // distinct from any sensor-level event: it says the motor's motion was
        // canceled and it is now latched in alert until clear_alerts.
        if (motor.moving && CutterHal::HasMotionCanceledEStop(motor.motor_index)) {
            motor.moving = false;
            motor.stop_sensor_pin = CutterHal::PIN_INVALID;
            m_response.Event("alert", motor.move_id)
                .Param("motor", static_cast<int32_t>(motor.motor_index))
                .Param("cause", "estop")
                .Param("position", CutterHal::GetMotorPosition(motor.motor_index));
            SendResponse();

            // Check if any motors still moving
            bool any_moving = false;
            for (size_t j = 0; j < NUM_MOTORS; j++) {
                if (m_motors[j].moving) {
                    any_moving = true;
                    break;
                }
            }
            if (!any_moving && m_stateMachine.GetState() == State::WORKING) {
                m_stateMachine.TransitionTo(State::READY);
            }
            continue;  // Skip other checks for this motor
        }

        // Handle a limit switch canceling a normal (non-homing) move.
        // ClearCore auto-decelerates the motor and latches a
        // MotionCanceled*Limit alert; report it via the unified "alert" event
        // (cause=pos_limit/neg_limit), stop tracking the move, and return to
        // READY. This is distinct from the "limit" sensor event emitted by the
        // monitor above: a mid-move trip yields both a "limit" (the switch
        // changed state) and an "alert" (the motor was put into alert).
        // Without this, open-loop steppers would report a false "done" (success
        // on a crash) and SDSK/ClearPath moves would never complete because the
        // latched alert blocks move completion (hang).
        // Active homing legitimately drives into the limit and consumes these
        // alerts itself (see CheckHomingState), so skip while homing.
        if (motor.moving &&
            (motor.homing_state == HomingState::IDLE ||
             motor.homing_state == HomingState::COMPLETE)) {
            bool neg_limit = CutterHal::HasMotionCanceledNegLimit(motor.motor_index);
            bool pos_limit = CutterHal::HasMotionCanceledPosLimit(motor.motor_index);
            if (neg_limit || pos_limit) {
                motor.moving = false;
                motor.stop_sensor_pin = CutterHal::PIN_INVALID;
                m_response.Event("alert", motor.move_id)
                    .Param("motor", static_cast<int32_t>(motor.motor_index))
                    .Param("cause", neg_limit ? "neg_limit" : "pos_limit")
                    .Param("position", CutterHal::GetMotorPosition(motor.motor_index));
                SendResponse();

                // Transition back to READY if no other motors moving
                bool any_moving = false;
                for (size_t j = 0; j < NUM_MOTORS; j++) {
                    if (m_motors[j].moving) {
                        any_moving = true;
                        break;
                    }
                }
                if (!any_moving && m_stateMachine.GetState() == State::WORKING) {
                    m_stateMachine.TransitionTo(State::READY);
                }
                continue;  // Skip other checks for this motor
            }
        }

        // Check the move_until sensor stop condition. When a non-limit sensor
        // (e.g. a vacuum switch) reaches its target logical level, stop the
        // velocity move and report the position. This is a successful
        // completion, NOT a fault: no alert is latched, so no clear_alerts is
        // needed afterward. The soft-limit check below remains active as the
        // travel bound for the "sensor never tripped" outcome. Checked before
        // the soft limit so a sensor trip at the boundary reports sensor_stop.
        if (motor.moving && motor.stop_sensor_pin != CutterHal::PIN_INVALID) {
            // CheckPins() ran earlier this Update(), but read fresh and apply
            // the configured inversion to match its logical-level semantics.
            bool raw = CutterHal::ReadDigitalPin(motor.stop_sensor_pin);
            PinSlot* sensor = GetPin(motor.stop_sensor_pin);
            bool logical = sensor->digital_in.invert ? !raw : raw;
            if (logical == motor.stop_sensor_value) {
                CutterHal::StopMotor(motor.motor_index, true);
                motor.moving = false;
                uint8_t sensor_pin = motor.stop_sensor_pin;
                motor.stop_sensor_pin = CutterHal::PIN_INVALID;
                m_response.Event("sensor_stop", motor.move_id)
                    .Param("motor", static_cast<int32_t>(motor.motor_index))
                    .Param("pin", static_cast<int32_t>(sensor_pin))
                    .Param("position", CutterHal::GetMotorPosition(motor.motor_index));
                SendResponse();

                // Transition back to READY if no other motors moving
                bool any_moving = false;
                for (size_t j = 0; j < NUM_MOTORS; j++) {
                    if (m_motors[j].moving) {
                        any_moving = true;
                        break;
                    }
                }
                if (!any_moving && m_stateMachine.GetState() == State::WORKING) {
                    m_stateMachine.TransitionTo(State::READY);
                }
                continue;  // Skip the remaining checks for this motor
            }
        }

        // Check soft limits for velocity moves. Skip during an active homing
        // sequence: the position counter is not yet referenced (zero is only set
        // when homing completes) and homing deliberately drives into the limit,
        // which sits outside the soft-limit range. Matches the homing exclusion
        // on the alert check above.
        if (motor.moving && motor.velocity_move && motor.soft_limits_enabled &&
            (motor.homing_state == HomingState::IDLE ||
             motor.homing_state == HomingState::COMPLETE)) {
            int32_t pos = CutterHal::GetMotorPosition(motor.motor_index);
            bool at_limit = false;
            // Use strict inequality so starting at boundary is allowed
            if (pos < motor.soft_limit_min || pos > motor.soft_limit_max) {
                at_limit = true;
            }
            if (at_limit) {
                CutterHal::StopMotor(motor.motor_index, true);
                motor.moving = false;
                motor.stop_sensor_pin = CutterHal::PIN_INVALID;
                m_response.Event("soft_limit", motor.move_id)
                    .Param("motor", static_cast<int32_t>(motor.motor_index))
                    .Param("position", pos);
                SendResponse();

                // Transition back to READY if no other motors moving
                bool any_moving = false;
                for (size_t j = 0; j < NUM_MOTORS; j++) {
                    if (m_motors[j].moving) {
                        any_moving = true;
                        break;
                    }
                }
                if (!any_moving && m_stateMachine.GetState() == State::WORKING) {
                    m_stateMachine.TransitionTo(State::READY);
                }
                continue;  // Skip move completion check since we stopped it
            }
        }

        // Check for move completion
        // For SDSK/ClearPath: steps complete AND HLFB asserted AND no alerts
        // For generic steppers: just steps complete (open-loop, no position feedback)
        if (motor.moving && CutterHal::StepsComplete(motor.motor_index)) {
            bool move_complete = true;
            if (motor.type == MotorType::CLEARPATH) {
                // SDSK needs HLFB asserted AND no alerts to confirm move truly complete
                move_complete = (CutterHal::GetHlfbState(motor.motor_index) == CutterHal::HLFB_ASSERTED) &&
                               !CutterHal::HasMotorAlerts(motor.motor_index);
            }

            if (move_complete) {
                motor.moving = false;
                motor.stop_sensor_pin = CutterHal::PIN_INVALID;
                m_response.Event("done", motor.move_id)
                    .Param("motor", static_cast<int32_t>(motor.motor_index))
                    .Param("position", CutterHal::GetMotorPosition(motor.motor_index));
                SendResponse();

                // If no motors are moving, transition back to READY
                bool any_moving = false;
                for (size_t j = 0; j < NUM_MOTORS; j++) {
                    if (m_motors[j].moving) {
                        any_moving = true;
                        break;
                    }
                }
                if (!any_moving && m_stateMachine.GetState() == State::WORKING) {
                    m_stateMachine.TransitionTo(State::READY);
                }
            }
        }

        // Check homing state for both motor types
        if (motor.homing_state != HomingState::IDLE &&
            motor.homing_state != HomingState::COMPLETE) {
            // Homing logic handled in MotorCommands.cpp
            extern void CheckHomingState(Controller* ctrl, MotorSlot& motor);
            HomingState prev_state = motor.homing_state;
            CheckHomingState(this, motor);

            // If homing just completed and enable_all is active, start next motor
            if (prev_state != HomingState::COMPLETE &&
                motor.homing_state == HomingState::COMPLETE &&
                m_enableAllActive) {
                m_currentHomingIndex++;
                StartNextHoming();
            }
        }
    }
}

void Controller::SendEvent(const char* type) {
    m_response.Event(type);
    SendResponse();
}

void Controller::SendResponse() {
    const char* resp = m_response.Finish();
    m_serial->Send(resp);
    m_response.Reset();
}

void Controller::OnHostConnected() {
    if (m_stateMachine.GetState() == State::UNCONNECTED) {
        m_stateMachine.MarkConnected();
    }
    // Greet exactly once per connection (re-armed on port close).
    if (!m_bannerSent) {
        SendBanner();
        m_bannerSent = true;
    }
}

void Controller::SendBanner() {
    // Emitted once when a host first connects, before the first command's
    // response. Carries the protocol/firmware versions and the board's
    // unique device id so a client can identify what it is talking to.
    m_response.Debug("cutter ready")
        .Param("protocol", PROTOCOL_VERSION)
        .Param("version", CUTTER_VERSION)
        .Param("device_id", CutterHal::DeviceId());
    SendResponse();
}

PinSlot* Controller::GetPin(uint8_t index) {
    if (index >= NUM_PINS) return nullptr;
    return &m_pins[index];
}

const PinSlot* Controller::GetPin(uint8_t index) const {
    if (index >= NUM_PINS) return nullptr;
    return &m_pins[index];
}

MotorSlot* Controller::GetMotor(uint8_t index) {
    if (index >= NUM_MOTORS) return nullptr;
    return &m_motors[index];
}

const MotorSlot* Controller::GetMotor(uint8_t index) const {
    if (index >= NUM_MOTORS) return nullptr;
    return &m_motors[index];
}

void Controller::SetEnableAllId(const CommandId& id) {
    m_enableAllId = id;
    m_enableAllActive = true;
    m_currentHomingIndex = 0;
    m_homingCount = 0;

    // Build sorted list of motors to home by priority
    struct MotorPriority {
        uint8_t index;
        uint8_t priority;
    };
    MotorPriority motors[NUM_MOTORS];
    size_t count = 0;

    for (size_t i = 0; i < NUM_MOTORS; i++) {
        MotorSlot& slot = m_motors[i];
        if (slot.type != MotorType::UNCONFIGURED && slot.homing_mode != HomingMode::NONE) {
            motors[count].index = static_cast<uint8_t>(i);
            motors[count].priority = slot.enable_priority;
            count++;
        }
    }

    // Simple bubble sort by priority
    for (size_t i = 0; i + 1 < count; i++) {
        for (size_t j = 0; j < count - i - 1; j++) {
            if (motors[j].priority > motors[j + 1].priority) {
                MotorPriority tmp = motors[j];
                motors[j] = motors[j + 1];
                motors[j + 1] = tmp;
            }
        }
    }

    // Store sorted order
    for (size_t i = 0; i < count; i++) {
        m_homingOrder[i] = motors[i].index;
    }
    m_homingCount = static_cast<uint8_t>(count);
}

void Controller::StartNextHoming() {
    // Forward declaration of StartHoming from MotorCommands.cpp
    extern void StartHoming(MotorSlot* slot, const CommandId& id);

    while (m_currentHomingIndex < m_homingCount) {
        uint8_t motor_idx = m_homingOrder[m_currentHomingIndex];
        MotorSlot* slot = &m_motors[motor_idx];

        // Skip if already homed or not enabled
        if (slot->homed || !slot->enabled) {
            m_currentHomingIndex++;
            continue;
        }

        // Start homing this motor
        StartHoming(slot, m_enableAllId);

        // Transition to WORKING if needed
        if (m_stateMachine.GetState() == State::READY) {
            m_stateMachine.TransitionTo(State::WORKING);
        }

        m_response.Event("homing_started", m_enableAllId)
            .Param("motor", static_cast<int32_t>(motor_idx));
        SendResponse();

        return;  // Wait for this motor to finish homing
    }

    // All motors homed - emit completion event
    m_enableAllActive = false;
    m_response.Event("all_homed", m_enableAllId)
        .Param("count", static_cast<int32_t>(m_homingCount));
    SendResponse();
}

// Built-in commands

void Controller::CmdPing(const ParsedCommand& cmd) {
    (void)cmd;  // User-supplied params not needed for response
    m_response.Ok(m_currentCommandId);
    SendResponse();
}

void Controller::CmdReset(const ParsedCommand& cmd) {
    (void)cmd;  // User-supplied params not needed for response
    if (m_stateMachine.GetState() == State::ERROR) {
        m_stateMachine.Reset();

        // Reset all pins to unconfigured
        for (size_t i = 0; i < NUM_PINS; i++) {
            m_pins[i].mode = PinMode::UNCONFIGURED;
        }

        // Reset all motors
        for (size_t i = 0; i < NUM_MOTORS; i++) {
            if (m_motors[i].type != MotorType::UNCONFIGURED) {
                CutterHal::EnableMotor(m_motors[i].motor_index, false);
            }
            memset(&m_motors[i], 0, sizeof(MotorSlot));
            m_motors[i].type = MotorType::UNCONFIGURED;
            m_motors[i].motor_index = static_cast<uint8_t>(i);
        }

        m_response.Ok(m_currentCommandId);
        m_response.Param("new_epoch", m_stateMachine.GetEpoch());
    } else {
        m_response.Error(static_cast<uint32_t>(ErrorCode::INVALID_STATE),
                       "Not in error state", m_currentCommandId);
    }
    SendResponse();
}

void Controller::CmdStatus(const ParsedCommand& cmd) {
    (void)cmd;  // User-supplied params not needed for response
    m_response.Ok(m_currentCommandId);
    m_response.Param("state", StateName(m_stateMachine.GetState()));

    if (m_stateMachine.GetState() == State::ERROR) {
        m_response.Param("error_code", static_cast<uint32_t>(m_stateMachine.GetErrorCode()));
        m_response.Param("error_message", m_stateMachine.GetErrorMessage());
    }
    SendResponse();
}

void Controller::CmdVersion(const ParsedCommand& cmd) {
    (void)cmd;  // User-supplied params not needed for response
    m_response.Ok(m_currentCommandId);
    m_response.Param("version", CUTTER_VERSION);
    m_response.Param("protocol", PROTOCOL_VERSION);
    SendResponse();
}

void Controller::CmdEmergencyStop(const ParsedCommand& cmd) {
    (void)cmd;  // User-supplied params not needed for response
    // Stop all motors immediately
    for (size_t i = 0; i < NUM_MOTORS; i++) {
        if (m_motors[i].type != MotorType::UNCONFIGURED) {
            CutterHal::StopMotor(m_motors[i].motor_index, true);  // immediate stop
            m_motors[i].moving = false;
            m_motors[i].homing_state = HomingState::IDLE;
            m_motors[i].stop_sensor_pin = CutterHal::PIN_INVALID;
        }
    }

    // Enter error state
    m_stateMachine.EnterError(ErrorCode::EMERGENCY_STOP, "Emergency stop");

    // Send response with command's assigned ID and the new epoch after error
    m_response.Ok(m_currentCommandId);
    m_response.Param("new_epoch", m_stateMachine.GetEpoch());
    SendResponse();

    // Send error event
    m_response.Event("error", m_currentCommandId)
        .Param("code", static_cast<uint32_t>(ErrorCode::EMERGENCY_STOP))
        .Param("message", "Emergency stop activated");
    SendResponse();
}

bool Controller::AnyMotorMoving() const {
    for (size_t i = 0; i < NUM_MOTORS; i++) {
        if (m_motors[i].moving) return true;
    }
    return false;
}

// DEBUG-ONLY. Block processing of further serial commands until the target
// motor's move completes (or a timeout elapses), so a script pasted into a dumb
// terminal steps through one move at a time. This deliberately suspends the
// serial stop/emergency_stop path while waiting - the hardware E-stop input and
// limit switches still abort (CheckMotors keeps running), and the timeout bounds
// the wait. Not for production host code; the host should sequence on the
// move's "done"/"sensor_stop" events instead. See Update() for the resolution.
void Controller::CmdDebugWait(const ParsedCommand& cmd) {
    State s = m_stateMachine.GetState();
    if (s != State::READY && s != State::WORKING) {
        m_response.Error(static_cast<uint32_t>(ErrorCode::INVALID_STATE),
                         "Not in ready state", m_currentCommandId);
        SendResponse();
        return;
    }

    // Optional motor: absent means "wait until no motor is moving".
    int32_t motor;
    bool has_motor = cmd.GetInt("motor", &motor);
    if (has_motor && (motor < 0 || motor >= static_cast<int32_t>(NUM_MOTORS))) {
        m_response.Error(static_cast<uint32_t>(ErrorCode::INVALID_MOTOR),
                         "Invalid motor", m_currentCommandId);
        SendResponse();
        return;
    }

    uint8_t target = has_motor ? static_cast<uint8_t>(motor) : 0xFF;
    bool moving = (target == 0xFF) ? AnyMotorMoving() : m_motors[target].moving;

    // Already idle: nothing to wait for, ack immediately (no wait mode entered).
    if (!moving) {
        m_response.Ok(m_currentCommandId);
        if (target != 0xFF) {
            m_response.Param("motor", static_cast<int32_t>(target));
            m_response.Param("position", CutterHal::GetMotorPosition(target));
        }
        SendResponse();
        return;
    }

    // Enter wait mode. The ok/error is deferred to Update() when the wait ends.
    m_debugWaitMotor = target;
    m_debugWaitTimeoutMs =
        static_cast<uint32_t>(cmd.GetIntOr("timeout_ms", 30000));
    m_debugWaitStart = CutterHal::Milliseconds();
    m_debugWaitId = m_currentCommandId;
    m_debugWaitActive = true;
}

void Controller::CmdGetNextSeq(const ParsedCommand& cmd) {
    (void)cmd;  // User-supplied params not needed for response
    m_response.Ok(m_currentCommandId);
    m_response.Param("next_seq", m_nextSeq);
    SendResponse();
}

// Helper to convert PinMode to string
static const char* PinModeName(PinMode mode) {
    switch (mode) {
        case PinMode::UNCONFIGURED: return "unconfigured";
        case PinMode::DIGITAL_IN:   return "digital_in";
        case PinMode::DIGITAL_OUT:  return "digital_out";
        case PinMode::ANALOG_IN:    return "analog_in";
        case PinMode::PWM:          return "pwm";
        case PinMode::H_BRIDGE:     return "hbridge";
        case PinMode::END_STOP:     return "endstop";
        case PinMode::MOTOR_LIMIT:  return "motor_limit";
        default:                    return "unknown";
    }
}

// Helper to convert MotorType to string
static const char* MotorTypeName(MotorType type) {
    switch (type) {
        case MotorType::UNCONFIGURED:    return "unconfigured";
        case MotorType::CLEARPATH:       return "clearpath";
        case MotorType::GENERIC_STEPPER: return "stepper";
        default:                         return "unknown";
    }
}

// Helper to convert HomingMode to string
static const char* HomingModeName(HomingMode mode) {
    switch (mode) {
        case HomingMode::NONE:         return "none";
        case HomingMode::MSP:          return "msp";
        case HomingMode::LIMIT_SWITCH: return "limit_switch";
        default:                       return "unknown";
    }
}

void Controller::CmdGetStatus(const ParsedCommand& cmd) {
    (void)cmd;  // User-supplied params not needed for response
    // Count configured pins
    int pin_count = 0;
    for (size_t i = 0; i < NUM_PINS; i++) {
        if (m_pins[i].mode != PinMode::UNCONFIGURED) {
            pin_count++;
        }
    }

    // Count configured motors
    int motor_count = 0;
    for (size_t i = 0; i < NUM_MOTORS; i++) {
        if (m_motors[i].type != MotorType::UNCONFIGURED) {
            motor_count++;
        }
    }

    // Send one status row per configured pin. When nothing is configured,
    // both loops emit nothing and only the terminating summary row is sent.
    for (size_t i = 0; i < NUM_PINS; i++) {
        const PinSlot& pin = m_pins[i];
        if (pin.mode == PinMode::UNCONFIGURED) continue;

        m_response.Status("pin", m_currentCommandId);
        m_response.Param("pin", static_cast<int32_t>(i));
        m_response.Param("mode", PinModeName(pin.mode));

        // Add mode-specific info
        switch (pin.mode) {
            case PinMode::DIGITAL_IN: {
                bool raw_val = CutterHal::ReadDigitalPin(pin.pin_index);
                bool val = pin.digital_in.invert ? !raw_val : raw_val;
                m_response.Param("value", val);
                m_response.Param("invert", pin.digital_in.invert);
                if (pin.digital_in.report_edges != EdgeMode::NONE) {
                    const char* edge_str = "none";
                    switch (pin.digital_in.report_edges) {
                        case EdgeMode::RISING:  edge_str = "rising"; break;
                        case EdgeMode::FALLING: edge_str = "falling"; break;
                        case EdgeMode::BOTH:    edge_str = "both"; break;
                        default: break;
                    }
                    m_response.Param("report_edges", edge_str);
                }
                if (pin.digital_in.error_trigger_enabled) {
                    m_response.Param("error_trigger", pin.digital_in.error_trigger_value);
                }
                break;
            }
            case PinMode::DIGITAL_OUT:
                m_response.Param("value", pin.digital_out.current_value);
                if (pin.digital_out.on_error_enabled) {
                    m_response.Param("on_error", pin.digital_out.on_error_value);
                }
                if (pin.digital_out.default_max_ms > 0) {
                    m_response.Param("max_ms", pin.digital_out.default_max_ms);
                }
                break;
            case PinMode::ANALOG_IN: {
                int16_t val = CutterHal::ReadAnalogPin(pin.pin_index);
                m_response.Param("value", static_cast<int32_t>(val));
                if (pin.analog_in.error_threshold_enabled) {
                    m_response.Param("error_low", static_cast<int32_t>(pin.analog_in.error_threshold_low));
                    m_response.Param("error_high", static_cast<int32_t>(pin.analog_in.error_threshold_high));
                }
                if (pin.analog_in.report_interval_ms > 0) {
                    m_response.Param("report_interval", pin.analog_in.report_interval_ms);
                }
                break;
            }
            case PinMode::PWM:
                m_response.Param("duty", static_cast<int32_t>(pin.pwm.duty));
                break;
            case PinMode::H_BRIDGE:
                m_response.Param("value", static_cast<int32_t>(pin.hbridge.value));
                if (pin.hbridge.tone_active) {
                    m_response.Param("tone_freq", static_cast<int32_t>(pin.hbridge.tone_freq));
                    m_response.Param("tone_amplitude", static_cast<int32_t>(pin.hbridge.tone_amplitude));
                }
                break;
            case PinMode::END_STOP: {
                bool raw_val = CutterHal::ReadDigitalPin(pin.pin_index);
                bool triggered = (raw_val == (pin.end_stop.triggered_value != 0));
                m_response.Param("triggered", triggered);
                m_response.Param("triggered_value", static_cast<int32_t>(pin.end_stop.triggered_value));
                break;
            }
            default:
                break;
        }
        SendResponse();
    }

    // Send one response per configured motor
    for (size_t i = 0; i < NUM_MOTORS; i++) {
        const MotorSlot& motor = m_motors[i];
        if (motor.type == MotorType::UNCONFIGURED) continue;

        m_response.Status("motor", m_currentCommandId);
        m_response.Param("motor", static_cast<int32_t>(i));
        m_response.Param("motor_type", MotorTypeName(motor.type));
        m_response.Param("enabled", motor.enabled);
        m_response.Param("moving", motor.moving);
        m_response.Param("position", CutterHal::GetMotorPosition(motor.motor_index));
        m_response.Param("homed", motor.homed);
        m_response.Param("homing_mode", HomingModeName(motor.homing_mode));
        m_response.Param("vel_max", motor.vel_max);
        m_response.Param("accel_max", motor.accel_max);

        // Add soft limit info if enabled
        if (motor.soft_limits_enabled) {
            m_response.Param("soft_limit_min", motor.soft_limit_min);
            m_response.Param("soft_limit_max", motor.soft_limit_max);
        }

        // Add velocity move indicator
        if (motor.moving && motor.velocity_move) {
            m_response.Param("velocity_move", true);
        }

        // Add homing state if homing is in progress
        if (motor.homing_state != HomingState::IDLE &&
            motor.homing_state != HomingState::COMPLETE) {
            const char* homing_state_str = "unknown";
            switch (motor.homing_state) {
                case HomingState::SEEKING:     homing_state_str = "seeking"; break;
                case HomingState::RELEASING:   homing_state_str = "releasing"; break;
                case HomingState::LATCHING:    homing_state_str = "latching"; break;
                case HomingState::BACKING_OFF: homing_state_str = "backing_off"; break;
                default: break;
            }
            m_response.Param("homing_state", homing_state_str);
        }

        SendResponse();
    }

    // Terminating summary row with counts. This (not an "ok") marks the end
    // of the get_status stream; clients read status rows until type=summary.
    m_response.Status("summary", m_currentCommandId);
    m_response.Param("pin_count", static_cast<int32_t>(pin_count));
    m_response.Param("motor_count", static_cast<int32_t>(motor_count));
    SendResponse();
}

}  // namespace Cutter
