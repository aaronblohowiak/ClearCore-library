/**
 * @file CutterHal_Fake.cpp
 * @brief Fake HAL implementation for testing
 *
 * This file is only compiled when CUTTER_PLATFORM_TEST is defined.
 * All functions read/write the global FakeHalState so tests can
 * control hardware behavior.
 */

#ifdef CUTTER_PLATFORM_TEST

#include "CutterHal.h"
#include "FakeHal.h"

// Global fake state instance
FakeHalState g_fake;

namespace CutterHal {

bool ReadDigitalPin(uint8_t pin) {
    if (pin < 13) {
        return g_fake.digital_pins[pin];
    }
    return false;
}

void WriteDigitalPin(uint8_t pin, bool value) {
    if (pin < 13) {
        g_fake.digital_pins[pin] = value;
        g_fake.digital_pin_writes[pin]++;
    }
}

int16_t ReadAnalogPin(uint8_t pin) {
    if (pin >= 9 && pin <= 12) {
        return g_fake.analog_pins[pin - 9];
    }
    return 0;
}

void SetPwmDuty(uint8_t pin, uint16_t duty) {
    if (pin < 6) {
        g_fake.pwm_duty[pin] = duty;
    }
}

void SetPwmFrequency(uint8_t pin, uint32_t freq) {
    // PWM frequency not tracked in fake state currently
    (void)pin;
    (void)freq;
}

void SetHBridgeValue(uint8_t pin, int16_t value) {
    if (pin == 4 || pin == 5) {
        g_fake.hbridge_value[pin - 4] = value;
    }
}

void StartTone(uint8_t pin, uint16_t freq, int16_t amplitude) {
    if (pin == 4 || pin == 5) {
        uint8_t idx = pin - 4;
        g_fake.tone_active[idx] = true;
        g_fake.tone_freq[idx] = freq;
        g_fake.tone_amplitude[idx] = amplitude;
    }
}

void StopTone(uint8_t pin) {
    if (pin == 4 || pin == 5) {
        g_fake.tone_active[pin - 4] = false;
    }
}

void ConfigurePinMode(uint8_t pin, uint8_t mode) {
    // Pin mode configuration not tracked in fake state
    (void)pin;
    (void)mode;
}

void EnableMotor(uint8_t motor, bool enable) {
    if (motor < 4) {
        g_fake.motor_enabled[motor] = enable;
    }
}

void MoveRelative(uint8_t motor, int32_t steps) {
    if (motor < 4) {
        g_fake.motor_target[motor] = g_fake.motor_position[motor] + steps;
        g_fake.motor_moving[motor] = true;
        g_fake.motor_steps_complete[motor] = false;
    }
}

void MoveAbsolute(uint8_t motor, int32_t position) {
    if (motor < 4) {
        g_fake.motor_target[motor] = position;
        g_fake.motor_moving[motor] = true;
        g_fake.motor_steps_complete[motor] = false;
    }
}

void MoveVelocity(uint8_t motor, int32_t velocity) {
    if (motor < 4) {
        g_fake.motor_velocity[motor] = velocity;
        g_fake.motor_moving[motor] = (velocity != 0);
        // Velocity moves don't have a target, so steps_complete stays false
        // until explicitly stopped
        if (velocity != 0) {
            g_fake.motor_steps_complete[motor] = false;
        }
    }
}

void StopMotor(uint8_t motor, bool immediate) {
    if (motor < 4) {
        g_fake.motor_velocity[motor] = 0;
        g_fake.motor_moving[motor] = false;
        g_fake.motor_steps_complete[motor] = true;
        if (immediate) {
            // Hard stop - position stays where it is
        } else {
            // Decel stop - in fake, just complete immediately
            g_fake.motor_position[motor] = g_fake.motor_target[motor];
        }
    }
    (void)immediate;
}

void SetMotorParams(uint8_t motor, int32_t velMax, int32_t accelMax) {
    if (motor < 4) {
        g_fake.motor_vel_max[motor] = velMax;
        g_fake.motor_accel_max[motor] = accelMax;
    }
}

int32_t GetMotorPosition(uint8_t motor) {
    if (motor < 4) {
        return g_fake.motor_position[motor];
    }
    return 0;
}

void SetMotorPosition(uint8_t motor, int32_t position) {
    if (motor < 4) {
        g_fake.motor_position[motor] = position;
    }
}

bool StepsComplete(uint8_t motor) {
    if (motor < 4) {
        return g_fake.motor_steps_complete[motor];
    }
    return true;
}

uint8_t GetHlfbState(uint8_t motor) {
    if (motor < 4) {
        return g_fake.hlfb_state[motor];
    }
    return 0;
}

bool IsMotorReady(uint8_t motor) {
    if (motor < 4) {
        return g_fake.motor_ready[motor];
    }
    return false;
}

bool IsMotorInFault(uint8_t motor) {
    if (motor < 4) {
        return g_fake.motor_fault[motor];
    }
    return false;
}

bool HasMotorAlerts(uint8_t motor) {
    if (motor < 4) {
        // Alerts present if motion was canceled by limit switch
        return g_fake.motion_canceled_neg_limit[motor] ||
               g_fake.motion_canceled_pos_limit[motor];
    }
    return false;
}

bool IsPinInFault(uint8_t pin) {
    if (pin < 6) {
        return g_fake.pin_fault[pin];
    }
    return false;
}

bool SetLimitSwitchNeg(uint8_t motor, uint8_t pin) {
    if (motor < 4) {
        g_fake.limit_switch_neg_pin[motor] = pin;
        return true;
    }
    return false;
}

bool SetLimitSwitchPos(uint8_t motor, uint8_t pin) {
    if (motor < 4) {
        g_fake.limit_switch_pos_pin[motor] = pin;
        return true;
    }
    return false;
}

bool HasMotionCanceledNegLimit(uint8_t motor) {
    if (motor < 4) {
        return g_fake.motion_canceled_neg_limit[motor];
    }
    return false;
}

bool HasMotionCanceledPosLimit(uint8_t motor) {
    if (motor < 4) {
        return g_fake.motion_canceled_pos_limit[motor];
    }
    return false;
}

void ClearMotorAlerts(uint8_t motor) {
    if (motor < 4) {
        g_fake.motion_canceled_neg_limit[motor] = false;
        g_fake.motion_canceled_pos_limit[motor] = false;
    }
}

uint32_t Milliseconds() {
    return g_fake.time_ms;
}

}  // namespace CutterHal

#endif  // CUTTER_PLATFORM_TEST
