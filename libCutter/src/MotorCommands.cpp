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

// Configure ClearPath-SD/SK motor (uses HLFB for done/error notification)
// SDSK motors use hard-stop homing: move into mechanical stop, detect via HLFB torque
static void CmdConfigureSdsk(Controller* ctrl, const ParsedCommand& cmd) {
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

    MotorSlot* slot = ctrl->GetMotor(static_cast<uint8_t>(motor));
    memset(slot, 0, sizeof(MotorSlot));
    slot->motor_index = static_cast<uint8_t>(motor);
    slot->type = MotorType::CLEARPATH;
    slot->vel_max = cmd.GetIntOr("vel_max", 10000);
    slot->accel_max = cmd.GetIntOr("accel_max", 100000);
    slot->hlfb_timeout_ms = static_cast<uint32_t>(cmd.GetIntOr("hlfb_timeout", 5000));

    // Enable/homing configuration
    slot->enable_priority = static_cast<uint8_t>(cmd.GetIntOr("enable_priority", motor));
    slot->home_on_enable = cmd.GetBoolOr("home_on_enable", true);
    slot->homing_direction = cmd.GetIntOr("homing_direction", -1);
    slot->homing_seek_velocity = cmd.GetIntOr("homing_velocity", 2000);
    slot->homing_torque_limit = cmd.GetIntOr("homing_torque_limit", 0);  // 0 = use HLFB default

    // Soft limits
    slot->soft_limits_enabled = cmd.GetBoolOr("soft_limits", false);
    slot->soft_limit_min = cmd.GetIntOr("soft_min", INT32_MIN);
    slot->soft_limit_max = cmd.GetIntOr("soft_max", INT32_MAX);

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

// Configure generic stepper motor (uses endstop for homing)
// Uses NC (normally-closed) endstop by default: triggered when pin reads LOW (fail-safe)
static void CmdConfigureStepper(Controller* ctrl, const ParsedCommand& cmd) {
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

    MotorSlot* slot = ctrl->GetMotor(static_cast<uint8_t>(motor));
    memset(slot, 0, sizeof(MotorSlot));
    slot->motor_index = static_cast<uint8_t>(motor);
    slot->type = MotorType::GENERIC_STEPPER;
    slot->vel_max = cmd.GetIntOr("vel_max", 10000);
    slot->accel_max = cmd.GetIntOr("accel_max", 100000);

    // Enable/homing configuration
    slot->enable_priority = static_cast<uint8_t>(cmd.GetIntOr("enable_priority", motor));
    slot->home_on_enable = cmd.GetBoolOr("home_on_enable", true);

    // Soft limits
    slot->soft_limits_enabled = cmd.GetBoolOr("soft_limits", false);
    slot->soft_limit_min = cmd.GetIntOr("soft_min", INT32_MIN);
    slot->soft_limit_max = cmd.GetIntOr("soft_max", INT32_MAX);

    // Endstop homing configuration
    // end_stop_triggered=0 means triggered when LOW (NC switch, fail-safe default)
    // end_stop_triggered=1 means triggered when HIGH (NO switch)
    slot->end_stop_pin = static_cast<uint8_t>(cmd.GetIntOr("end_stop_pin", 6));
    slot->end_stop_triggered = static_cast<uint8_t>(cmd.GetIntOr("end_stop_triggered", 0));
    slot->homing_seek_velocity = cmd.GetIntOr("homing_seek_velocity", 5000);
    slot->homing_latch_velocity = cmd.GetIntOr("homing_latch_velocity", 500);
    slot->homing_backoff_distance = cmd.GetIntOr("homing_backoff", 200);

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

// === Homing Commands ===

// Start homing sequence for a motor (called from enable_all and home command)
void StartHoming(MotorSlot* slot, uint32_t seq) {
    slot->moving = true;
    slot->move_seq = seq;
    slot->homed = false;

    if (slot->type == MotorType::GENERIC_STEPPER) {
        // Stepper: endstop-based homing
        slot->homing_state = HomingState::SEEKING;
        CutterHal::MoveVelocity(slot->motor_index, -slot->homing_seek_velocity);
    } else if (slot->type == MotorType::CLEARPATH) {
        // SDSK: hard-stop homing via HLFB
        slot->homing_state = HomingState::SDSK_SEEKING;
        int32_t velocity = slot->homing_seek_velocity * slot->homing_direction;
        CutterHal::MoveVelocity(slot->motor_index, velocity);
    }
}

static void CmdHome(Controller* ctrl, const ParsedCommand& cmd) {
    if (!RequireReady(ctrl, cmd)) return;

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
    if (!slot->enabled) {
        SendError(ctrl, cmd, ErrorCode::MOTOR_NOT_READY, "Motor not enabled");
        return;
    }

    // Start homing sequence
    StartHoming(slot, cmd.seq);

    // Transition to WORKING
    if (ctrl->GetState() == State::READY) {
        ctrl->GetStateMachine().TransitionTo(State::WORKING);
    }

    ctrl->Response().Ok();
    if (cmd.has_seq) ctrl->Response().Param("seq", cmd.seq);
    ctrl->Response().Param("motor", motor);
    ctrl->SendResponse();
}

// === Enable All Command ===
// Enables and homes all configured motors sequentially by priority

static void CmdEnableAll(Controller* ctrl, const ParsedCommand& cmd) {
    State s = ctrl->GetState();
    if (s != State::CONFIGURED && s != State::READY) {
        SendError(ctrl, cmd, ErrorCode::INVALID_STATE, "Not in configured or ready state");
        return;
    }

    // Collect configured motors and sort by enable_priority
    struct MotorPriority {
        uint8_t index;
        uint8_t priority;
    };
    MotorPriority motors[NUM_MOTORS];
    size_t motor_count = 0;

    for (size_t i = 0; i < NUM_MOTORS; i++) {
        MotorSlot* slot = ctrl->GetMotor(static_cast<uint8_t>(i));
        if (slot->type != MotorType::UNCONFIGURED) {
            motors[motor_count].index = static_cast<uint8_t>(i);
            motors[motor_count].priority = slot->enable_priority;
            motor_count++;
        }
    }

    if (motor_count == 0) {
        SendError(ctrl, cmd, ErrorCode::MOTOR_NOT_CONFIGURED, "No motors configured");
        return;
    }

    // Simple bubble sort by priority (small array)
    for (size_t i = 0; i < motor_count - 1; i++) {
        for (size_t j = 0; j < motor_count - i - 1; j++) {
            if (motors[j].priority > motors[j + 1].priority) {
                MotorPriority tmp = motors[j];
                motors[j] = motors[j + 1];
                motors[j + 1] = tmp;
            }
        }
    }

    // Enable all motors in priority order
    // Note: This is synchronous enable - the actual homing happens in CheckMotors
    for (size_t i = 0; i < motor_count; i++) {
        MotorSlot* slot = ctrl->GetMotor(motors[i].index);
        slot->enabled = true;
        slot->enable_start_time = CutterHal::Milliseconds();
        slot->homed = false;
        CutterHal::EnableMotor(slot->motor_index, true);
    }

    // Transition to ENABLING to wait for HLFB on SDSK motors
    ctrl->GetStateMachine().TransitionTo(State::ENABLING);

    // Store the sequence for enable_all completion event
    ctrl->SetEnableAllSeq(cmd.seq);

    ctrl->Response().Ok();
    if (cmd.has_seq) ctrl->Response().Param("seq", cmd.seq);
    ctrl->Response().Param("count", static_cast<int32_t>(motor_count));
    ctrl->SendResponse();
}

// === Homing State Machine (called from CheckMotors) ===

// Complete homing and emit event
static void CompleteHoming(Controller* ctrl, MotorSlot& motor) {
    CutterHal::StopMotor(motor.motor_index, true);
    CutterHal::SetMotorPosition(motor.motor_index, 0);
    motor.homing_state = HomingState::COMPLETE;
    motor.moving = false;
    motor.homed = true;

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

// Check endstop for generic stepper homing
static void CheckStepperHomingState(Controller* ctrl, MotorSlot& motor) {
    PinSlot* endstop = ctrl->GetPin(motor.end_stop_pin);
    if (!endstop || endstop->mode != PinMode::END_STOP) return;

    // Read pin and compare to triggered_value
    // end_stop_triggered=0: triggered when LOW (NC, fail-safe)
    // end_stop_triggered=1: triggered when HIGH (NO)
    bool pin_value = CutterHal::ReadDigitalPin(motor.end_stop_pin);
    bool triggered = (pin_value == (motor.end_stop_triggered == 1));

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
                CompleteHoming(ctrl, motor);
            }
            break;

        default:
            break;
    }
}

// Check HLFB for SDSK hard-stop homing
static void CheckSdskHomingState(Controller* ctrl, MotorSlot& motor) {
    switch (motor.homing_state) {
        case HomingState::SDSK_SEEKING: {
            // Moving toward hard stop, monitoring HLFB for torque limit
            uint8_t hlfb = CutterHal::GetHlfbState(motor.motor_index);

            // HLFB_DEASSERTED (0) indicates motor hit torque limit (hard stop)
            // This happens when ClearPath detects it can't complete the commanded move
            if (hlfb == 0) {  // HLFB_DEASSERTED
                motor.homing_state = HomingState::SDSK_CONFIRMED;
                CutterHal::StopMotor(motor.motor_index, true);
            }
            break;
        }

        case HomingState::SDSK_CONFIRMED:
            // Hard stop detected, complete homing
            CompleteHoming(ctrl, motor);
            break;

        default:
            break;
    }
}

void CheckHomingState(Controller* ctrl, MotorSlot& motor) {
    if (motor.type == MotorType::GENERIC_STEPPER) {
        CheckStepperHomingState(ctrl, motor);
    } else if (motor.type == MotorType::CLEARPATH) {
        CheckSdskHomingState(ctrl, motor);
    }
}

// === Dispatch Function ===

void DispatchMotorCommand(Controller* ctrl, const ParsedCommand& cmd) {
    if (strcmp(cmd.name, "configure_sdsk") == 0) {
        CmdConfigureSdsk(ctrl, cmd);
    } else if (strcmp(cmd.name, "configure_stepper") == 0) {
        CmdConfigureStepper(ctrl, cmd);
    } else if (strcmp(cmd.name, "enable") == 0) {
        CmdEnable(ctrl, cmd);
    } else if (strcmp(cmd.name, "enable_all") == 0) {
        CmdEnableAll(ctrl, cmd);
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
