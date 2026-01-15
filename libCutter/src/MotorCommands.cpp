/**
 * @file MotorCommands.cpp
 * @brief Motor command handlers
 */

#include "Cutter.h"
#include <cstring>

namespace Cutter {

// Helper to send error response
static void SendError(Controller* ctrl, const ParsedCommand& cmd,
                      ErrorCode code, const char* message) {
    ctrl->Response().Error(static_cast<uint32_t>(code), message);
    if (cmd.has_seq) {
        ctrl->Response().Param("seq", cmd.seq);
    }
    ctrl->SendResponse();
}

static bool RequireReady(Controller* ctrl, const ParsedCommand& cmd) {
    State s = ctrl->GetState();
    if (s != State::READY && s != State::WORKING) {
        SendError(ctrl, cmd, ErrorCode::INVALID_STATE, "Not in ready state");
        return false;
    }
    return true;
}

// === Configuration Commands ===

static void CmdConfigureMotor(Controller* ctrl, const ParsedCommand& cmd) {
    State s = ctrl->GetState();
    if (s == State::UNCONNECTED) {
        SendError(ctrl, cmd, ErrorCode::INVALID_STATE, "Not connected");
        return;
    }

    int32_t motor;
    if (!cmd.GetInt("motor", &motor) || motor < 0 || motor >= static_cast<int32_t>(NUM_MOTORS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_MOTOR, "Invalid motor");
        return;
    }

    const char* type_str = cmd.GetString("type");
    if (!type_str) {
        SendError(ctrl, cmd, ErrorCode::MISSING_PARAM, "Missing type parameter");
        return;
    }

    MotorType type;
    if (strcmp(type_str, "clearpath") == 0) {
        type = MotorType::CLEARPATH;
    } else if (strcmp(type_str, "stepper") == 0) {
        type = MotorType::GENERIC_STEPPER;
    } else {
        SendError(ctrl, cmd, ErrorCode::INVALID_PARAM, "Invalid motor type");
        return;
    }

    MotorSlot* slot = ctrl->GetMotor(static_cast<uint8_t>(motor));
    memset(slot, 0, sizeof(MotorSlot));
    slot->motor_index = static_cast<uint8_t>(motor);
    slot->type = type;
    slot->vel_max = cmd.GetIntOr("vel_max", 10000);
    slot->accel_max = cmd.GetIntOr("accel_max", 100000);
    slot->hlfb_timeout_ms = static_cast<uint32_t>(cmd.GetIntOr("hlfb_timeout", 5000));

    // Soft limits
    slot->soft_limits_enabled = cmd.GetBoolOr("soft_limits", false);
    slot->soft_limit_min = cmd.GetIntOr("soft_min", INT32_MIN);
    slot->soft_limit_max = cmd.GetIntOr("soft_max", INT32_MAX);

    // Homing config for generic steppers
    if (type == MotorType::GENERIC_STEPPER) {
        slot->end_stop_pin = static_cast<uint8_t>(cmd.GetIntOr("end_stop_pin", 6));
        slot->end_stop_active_high = cmd.GetBoolOr("end_stop_active_high", true);
        slot->homing_seek_velocity = cmd.GetIntOr("homing_seek_velocity", 5000);
        slot->homing_latch_velocity = cmd.GetIntOr("homing_latch_velocity", 500);
        slot->homing_backoff_distance = cmd.GetIntOr("homing_backoff", 200);
    }

    // Set motor parameters in HAL
    CutterHal::SetMotorParams(slot->motor_index, slot->vel_max, slot->accel_max);

    // Transition to CONFIGURED if we were in CONNECTED
    if (s == State::CONNECTED) {
        ctrl->GetStateMachine().TransitionTo(State::CONFIGURED);
    }

    ctrl->Response().Ok();
    if (cmd.has_seq) ctrl->Response().Param("seq", cmd.seq);
    ctrl->Response().Param("motor", motor);
    ctrl->SendResponse();
}

// === Enable/Disable Commands ===

static void CmdEnable(Controller* ctrl, const ParsedCommand& cmd) {
    State s = ctrl->GetState();
    if (s != State::CONFIGURED && s != State::READY) {
        SendError(ctrl, cmd, ErrorCode::INVALID_STATE, "Not in configured or ready state");
        return;
    }

    int32_t motor;
    if (!cmd.GetInt("motor", &motor) || motor < 0 || motor >= static_cast<int32_t>(NUM_MOTORS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_MOTOR, "Invalid motor");
        return;
    }

    MotorSlot* slot = ctrl->GetMotor(static_cast<uint8_t>(motor));
    if (slot->type == MotorType::UNCONFIGURED) {
        SendError(ctrl, cmd, ErrorCode::MOTOR_NOT_CONFIGURED, "Motor not configured");
        return;
    }

    slot->enabled = true;
    slot->enable_start_time = CutterHal::Milliseconds();
    CutterHal::EnableMotor(slot->motor_index, true);

    // Transition to ENABLING to wait for HLFB
    if (s == State::CONFIGURED) {
        ctrl->GetStateMachine().TransitionTo(State::ENABLING);
    }

    // For generic steppers, go straight to READY since no HLFB
    if (slot->type == MotorType::GENERIC_STEPPER) {
        // Check if all enabled motors are generic steppers (no HLFB wait needed)
        bool all_stepper = true;
        for (size_t i = 0; i < NUM_MOTORS; i++) {
            const MotorSlot* m = ctrl->GetMotor(static_cast<uint8_t>(i));
            if (m->type == MotorType::CLEARPATH && m->enabled) {
                all_stepper = false;
                break;
            }
        }
        if (all_stepper && ctrl->GetState() == State::ENABLING) {
            ctrl->GetStateMachine().TransitionTo(State::READY);
        }
    }

    ctrl->Response().Ok();
    if (cmd.has_seq) ctrl->Response().Param("seq", cmd.seq);
    ctrl->Response().Param("motor", motor);
    ctrl->SendResponse();
}

static void CmdDisable(Controller* ctrl, const ParsedCommand& cmd) {
    int32_t motor;
    if (!cmd.GetInt("motor", &motor) || motor < 0 || motor >= static_cast<int32_t>(NUM_MOTORS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_MOTOR, "Invalid motor");
        return;
    }

    MotorSlot* slot = ctrl->GetMotor(static_cast<uint8_t>(motor));
    if (slot->type == MotorType::UNCONFIGURED) {
        SendError(ctrl, cmd, ErrorCode::MOTOR_NOT_CONFIGURED, "Motor not configured");
        return;
    }

    slot->enabled = false;
    slot->moving = false;
    CutterHal::EnableMotor(slot->motor_index, false);

    ctrl->Response().Ok();
    if (cmd.has_seq) ctrl->Response().Param("seq", cmd.seq);
    ctrl->Response().Param("motor", motor);
    ctrl->SendResponse();
}

// === Motion Commands ===

static bool CheckSoftLimits(Controller* ctrl, const ParsedCommand& cmd,
                            MotorSlot* slot, int32_t target) {
    if (!slot->soft_limits_enabled) return true;

    if (target < slot->soft_limit_min || target > slot->soft_limit_max) {
        SendError(ctrl, cmd, ErrorCode::SOFT_LIMIT, "Move exceeds soft limits");
        return false;
    }
    return true;
}

static void CmdMove(Controller* ctrl, const ParsedCommand& cmd) {
    if (!RequireReady(ctrl, cmd)) return;

    int32_t motor;
    if (!cmd.GetInt("motor", &motor) || motor < 0 || motor >= static_cast<int32_t>(NUM_MOTORS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_MOTOR, "Invalid motor");
        return;
    }

    MotorSlot* slot = ctrl->GetMotor(static_cast<uint8_t>(motor));
    if (slot->type == MotorType::UNCONFIGURED || !slot->enabled) {
        SendError(ctrl, cmd, ErrorCode::MOTOR_NOT_READY, "Motor not enabled");
        return;
    }

    // Relative or absolute move
    int32_t steps;
    int32_t position;
    bool is_relative = cmd.GetInt("steps", &steps);
    bool is_absolute = cmd.GetInt("position", &position);

    if (!is_relative && !is_absolute) {
        SendError(ctrl, cmd, ErrorCode::MISSING_PARAM, "Missing steps or position parameter");
        return;
    }

    int32_t current_pos = CutterHal::GetMotorPosition(slot->motor_index);
    int32_t target;

    if (is_relative) {
        target = current_pos + steps;
    } else {
        target = position;
    }

    if (!CheckSoftLimits(ctrl, cmd, slot, target)) return;

    slot->moving = true;
    slot->move_seq = cmd.seq;

    if (is_relative) {
        CutterHal::MoveRelative(slot->motor_index, steps);
    } else {
        CutterHal::MoveAbsolute(slot->motor_index, position);
    }

    // Transition to WORKING
    if (ctrl->GetState() == State::READY) {
        ctrl->GetStateMachine().TransitionTo(State::WORKING);
    }

    ctrl->Response().Ok();
    if (cmd.has_seq) ctrl->Response().Param("seq", cmd.seq);
    ctrl->Response().Param("motor", motor);
    ctrl->SendResponse();
}

static void CmdMoveVelocity(Controller* ctrl, const ParsedCommand& cmd) {
    if (!RequireReady(ctrl, cmd)) return;

    int32_t motor;
    if (!cmd.GetInt("motor", &motor) || motor < 0 || motor >= static_cast<int32_t>(NUM_MOTORS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_MOTOR, "Invalid motor");
        return;
    }

    MotorSlot* slot = ctrl->GetMotor(static_cast<uint8_t>(motor));
    if (slot->type == MotorType::UNCONFIGURED || !slot->enabled) {
        SendError(ctrl, cmd, ErrorCode::MOTOR_NOT_READY, "Motor not enabled");
        return;
    }

    int32_t velocity;
    if (!cmd.GetInt("velocity", &velocity)) {
        SendError(ctrl, cmd, ErrorCode::MISSING_PARAM, "Missing velocity parameter");
        return;
    }

    slot->moving = true;
    slot->move_seq = cmd.seq;
    CutterHal::MoveVelocity(slot->motor_index, velocity);

    // Transition to WORKING
    if (ctrl->GetState() == State::READY) {
        ctrl->GetStateMachine().TransitionTo(State::WORKING);
    }

    ctrl->Response().Ok();
    if (cmd.has_seq) ctrl->Response().Param("seq", cmd.seq);
    ctrl->Response().Param("motor", motor);
    ctrl->SendResponse();
}

static void CmdStop(Controller* ctrl, const ParsedCommand& cmd) {
    int32_t motor;
    if (!cmd.GetInt("motor", &motor) || motor < 0 || motor >= static_cast<int32_t>(NUM_MOTORS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_MOTOR, "Invalid motor");
        return;
    }

    MotorSlot* slot = ctrl->GetMotor(static_cast<uint8_t>(motor));
    if (slot->type == MotorType::UNCONFIGURED) {
        SendError(ctrl, cmd, ErrorCode::MOTOR_NOT_CONFIGURED, "Motor not configured");
        return;
    }

    bool immediate = cmd.GetBoolOr("immediate", false);
    CutterHal::StopMotor(slot->motor_index, immediate);
    slot->moving = false;

    ctrl->Response().Ok();
    if (cmd.has_seq) ctrl->Response().Param("seq", cmd.seq);
    ctrl->Response().Param("motor", motor);
    ctrl->Response().Param("position", CutterHal::GetMotorPosition(slot->motor_index));
    ctrl->SendResponse();
}

static void CmdSetPosition(Controller* ctrl, const ParsedCommand& cmd) {
    int32_t motor;
    if (!cmd.GetInt("motor", &motor) || motor < 0 || motor >= static_cast<int32_t>(NUM_MOTORS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_MOTOR, "Invalid motor");
        return;
    }

    MotorSlot* slot = ctrl->GetMotor(static_cast<uint8_t>(motor));
    if (slot->type == MotorType::UNCONFIGURED) {
        SendError(ctrl, cmd, ErrorCode::MOTOR_NOT_CONFIGURED, "Motor not configured");
        return;
    }

    int32_t position;
    if (!cmd.GetInt("position", &position)) {
        SendError(ctrl, cmd, ErrorCode::MISSING_PARAM, "Missing position parameter");
        return;
    }

    CutterHal::SetMotorPosition(slot->motor_index, position);

    ctrl->Response().Ok();
    if (cmd.has_seq) ctrl->Response().Param("seq", cmd.seq);
    ctrl->Response().Param("motor", motor);
    ctrl->Response().Param("position", position);
    ctrl->SendResponse();
}

// === Homing Command ===

static void CmdHome(Controller* ctrl, const ParsedCommand& cmd) {
    if (!RequireReady(ctrl, cmd)) return;

    int32_t motor;
    if (!cmd.GetInt("motor", &motor) || motor < 0 || motor >= static_cast<int32_t>(NUM_MOTORS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_MOTOR, "Invalid motor");
        return;
    }

    MotorSlot* slot = ctrl->GetMotor(static_cast<uint8_t>(motor));
    if (slot->type != MotorType::GENERIC_STEPPER) {
        SendError(ctrl, cmd, ErrorCode::INVALID_PARAM, "Only generic steppers support homing");
        return;
    }
    if (!slot->enabled) {
        SendError(ctrl, cmd, ErrorCode::MOTOR_NOT_READY, "Motor not enabled");
        return;
    }

    // Start homing sequence
    slot->homing_state = HomingState::SEEKING;
    slot->moving = true;
    slot->move_seq = cmd.seq;

    // Move toward endstop at seek velocity
    CutterHal::MoveVelocity(slot->motor_index, -slot->homing_seek_velocity);

    // Transition to WORKING
    if (ctrl->GetState() == State::READY) {
        ctrl->GetStateMachine().TransitionTo(State::WORKING);
    }

    ctrl->Response().Ok();
    if (cmd.has_seq) ctrl->Response().Param("seq", cmd.seq);
    ctrl->Response().Param("motor", motor);
    ctrl->SendResponse();
}

// === Homing State Machine (called from CheckMotors) ===

void CheckHomingState(Controller* ctrl, MotorSlot& motor) {
    PinSlot* endstop = ctrl->GetPin(motor.end_stop_pin);
    if (!endstop || endstop->mode != PinMode::END_STOP) return;

    bool triggered = CutterHal::ReadDigitalPin(motor.end_stop_pin);
    if (!motor.end_stop_active_high) triggered = !triggered;

    switch (motor.homing_state) {
        case HomingState::SEEKING:
            // Looking for endstop
            if (triggered) {
                // Hit endstop - stop and back off
                CutterHal::StopMotor(motor.motor_index, true);
                motor.homing_state = HomingState::BACKING_OFF;
                CutterHal::MoveRelative(motor.motor_index, motor.homing_backoff_distance);
            }
            break;

        case HomingState::BACKING_OFF:
            // Waiting for backoff move to complete
            if (CutterHal::StepsComplete(motor.motor_index)) {
                // Now approach slowly for latch
                motor.homing_state = HomingState::LATCHING;
                CutterHal::MoveVelocity(motor.motor_index, -motor.homing_latch_velocity);
            }
            break;

        case HomingState::LATCHING:
            // Slow approach for precise position
            if (triggered) {
                // Final contact - set zero and complete
                CutterHal::StopMotor(motor.motor_index, true);
                CutterHal::SetMotorPosition(motor.motor_index, 0);
                motor.homing_state = HomingState::COMPLETE;
                motor.moving = false;

                ctrl->Response().Event("homed")
                    .Param("motor", static_cast<int32_t>(motor.motor_index))
                    .Param("seq", motor.move_seq);
                ctrl->SendResponse();

                // Check if all motors done
                bool any_moving = false;
                for (size_t i = 0; i < NUM_MOTORS; i++) {
                    if (ctrl->GetMotor(static_cast<uint8_t>(i))->moving) {
                        any_moving = true;
                        break;
                    }
                }
                if (!any_moving && ctrl->GetState() == State::WORKING) {
                    ctrl->GetStateMachine().TransitionTo(State::READY);
                }
            }
            break;

        default:
            break;
    }
}

// === Dispatch Function ===

void DispatchMotorCommand(Controller* ctrl, const ParsedCommand& cmd) {
    if (strcmp(cmd.name, "configure_motor") == 0) {
        CmdConfigureMotor(ctrl, cmd);
    } else if (strcmp(cmd.name, "enable") == 0) {
        CmdEnable(ctrl, cmd);
    } else if (strcmp(cmd.name, "disable") == 0) {
        CmdDisable(ctrl, cmd);
    } else if (strcmp(cmd.name, "move") == 0) {
        CmdMove(ctrl, cmd);
    } else if (strcmp(cmd.name, "move_velocity") == 0) {
        CmdMoveVelocity(ctrl, cmd);
    } else if (strcmp(cmd.name, "stop") == 0) {
        CmdStop(ctrl, cmd);
    } else if (strcmp(cmd.name, "home") == 0) {
        CmdHome(ctrl, cmd);
    } else if (strcmp(cmd.name, "set_position") == 0) {
        CmdSetPosition(ctrl, cmd);
    }
}

}  // namespace Cutter
