/**
 * @file FakeHal.h
 * @brief Fake HAL state for testing
 *
 * Provides a global state object that tests can manipulate to simulate
 * hardware behavior, plus convenience macros for common operations.
 */

#pragma once

#include <stdint.h>
#include <cstring>

/**
 * @brief Global fake hardware state
 *
 * Tests manipulate this directly to set up scenarios and verify behavior.
 */
struct FakeHalState {
    // === Pin State ===
    bool digital_pins[13] = {};           // Current digital pin values
    bool digital_pins_last[13] = {};      // Previous values for edge detection
    bool edge_risen[13] = {};             // Rising edge detected (clear-on-read)
    bool edge_fallen[13] = {};            // Falling edge detected (clear-on-read)
    int digital_pin_writes[13] = {};      // Write count per pin (for verification)
    int16_t analog_pins[4] = {};          // Analog values for pins 9-12
    uint16_t pwm_duty[6] = {};            // PWM duty for pins 0-5
    int16_t hbridge_value[2] = {};        // H-Bridge values for pins 4-5
    bool tone_active[2] = {};             // Tone playing on pins 4-5
    uint16_t tone_freq[2] = {};           // Tone frequency
    int16_t tone_amplitude[2] = {};       // Tone amplitude

    // === Fault State ===
    bool pin_fault[6] = {};               // Hardware fault on pins 0-5
    bool motor_fault[4] = {};             // Hardware fault on motors 0-3

    // === Motor State ===
    uint8_t motor_clock_rate = 1;             // Default: CLOCK_RATE_NORMAL
    bool motor_enabled[4] = {};
    bool motor_moving[4] = {};
    bool motor_steps_complete[4] = {true, true, true, true};
    bool move_rejected[4] = {};                // If set, Move*() returns false (sim alert/at limit)
    int32_t motor_position[4] = {};
    int32_t motor_target[4] = {};
    int32_t motor_velocity[4] = {};
    int32_t motor_vel_max[4] = {};
    int32_t motor_accel_max[4] = {};
    uint8_t hlfb_state[4] = {};
    uint8_t hlfb_mode[4] = {};                // HLFB interpretation mode
    bool motor_ready[4] = {};

    // === Limit Switch State ===
    uint8_t limit_switch_neg_pin[4] = {255, 255, 255, 255};  // PIN_INVALID = 255
    uint8_t limit_switch_pos_pin[4] = {255, 255, 255, 255};
    bool motion_canceled_neg_limit[4] = {};  // Alert flag
    bool motion_canceled_pos_limit[4] = {};  // Alert flag
    bool in_pos_limit[4] = {};               // Live positive limit input state
    bool in_neg_limit[4] = {};               // Live negative limit input state

    // === E-Stop State ===
    uint8_t estop_pin[4] = {255, 255, 255, 255};  // PIN_INVALID = 255
    uint32_t estop_decel[4] = {};
    bool motion_canceled_estop[4] = {};  // Alert flag

    // === Timing ===
    uint32_t time_ms = 0;

    /**
     * @brief Reset all state to defaults
     */
    void Reset() {
        memset(this, 0, sizeof(*this));
        // Motors start with steps_complete = true
        for (int i = 0; i < 4; i++) {
            motor_steps_complete[i] = true;
            limit_switch_neg_pin[i] = 255;  // PIN_INVALID
            limit_switch_pos_pin[i] = 255;
            estop_pin[i] = 255;  // PIN_INVALID
        }
    }

    /**
     * @brief Advance simulated time
     * @param ms Milliseconds to advance
     */
    void AdvanceTime(uint32_t ms) {
        time_ms += ms;
    }

    /**
     * @brief Simulate motor move completion
     * @param motor Motor index (0-3)
     *
     * Sets steps_complete=true and HLFB=ASSERTED. For SDSK motors,
     * both conditions are required for move completion.
     */
    void CompleteMotorMove(uint8_t motor) {
        if (motor < 4) {
            motor_position[motor] = motor_target[motor];
            motor_moving[motor] = false;
            motor_steps_complete[motor] = true;
            hlfb_state[motor] = 1;  // HLFB_ASSERTED - motor in position
        }
    }

    /**
     * @brief Set digital pin value and detect edges
     *
     * Sets the pin value and updates edge detection flags.
     * @param pin Pin index
     * @param value New value
     */
    void SetDigitalPin(uint8_t pin, bool value) {
        if (pin < 13) {
            bool old_val = digital_pins[pin];
            digital_pins[pin] = value;
            if (!old_val && value) {
                edge_risen[pin] = true;
            }
            if (old_val && !value) {
                edge_fallen[pin] = true;
            }
        }
    }

    /**
     * @brief Set end stop / digital input state
     * @param pin Pin index
     * @param active Active state
     */
    void TriggerEndStop(uint8_t pin, bool active) {
        SetDigitalPin(pin, active);
    }

    /**
     * @brief Set analog input value
     * @param pin Pin index (9-12)
     * @param value Analog value (0-4095)
     */
    void SetAnalogValue(uint8_t pin, int16_t value) {
        if (pin >= 9 && pin <= 12) {
            analog_pins[pin - 9] = value;
        }
    }

    /**
     * @brief Set motor HLFB state
     * @param motor Motor index (0-3)
     * @param state HLFB state code
     */
    void SetHlfbState(uint8_t motor, uint8_t state) {
        if (motor < 4) {
            hlfb_state[motor] = state;
        }
    }

    /**
     * @brief Set motor ready state
     * @param motor Motor index (0-3)
     * @param ready Ready state
     */
    void SetMotorReady(uint8_t motor, bool ready) {
        if (motor < 4) {
            motor_ready[motor] = ready;
        }
    }

    /**
     * @brief Trigger negative limit switch for motor
     *
     * Simulates hardware detecting limit - stops motor and sets alert.
     * @param motor Motor index (0-3)
     */
    void TriggerNegativeLimit(uint8_t motor) {
        if (motor < 4) {
            motion_canceled_neg_limit[motor] = true;
            in_neg_limit[motor] = true;       // Live input goes active
            motor_moving[motor] = false;
            motor_velocity[motor] = 0;
            motor_steps_complete[motor] = true;
        }
    }

    /**
     * @brief Set the live positive limit input state (press/release)
     * @param motor Motor index (0-3)
     * @param active true = switch active/pressed, false = released
     */
    void SetPosLimit(uint8_t motor, bool active) {
        if (motor < 4) {
            in_pos_limit[motor] = active;
        }
    }

    /**
     * @brief Set the live negative limit input state (press/release)
     * @param motor Motor index (0-3)
     * @param active true = switch active/pressed, false = released
     */
    void SetNegLimit(uint8_t motor, bool active) {
        if (motor < 4) {
            in_neg_limit[motor] = active;
        }
    }

    /**
     * @brief Trigger positive limit switch for motor
     *
     * Simulates hardware detecting limit - stops motor and sets alert.
     * @param motor Motor index (0-3)
     */
    void TriggerPositiveLimit(uint8_t motor) {
        if (motor < 4) {
            motion_canceled_pos_limit[motor] = true;
            in_pos_limit[motor] = true;       // Live input goes active
            motor_moving[motor] = false;
            motor_velocity[motor] = 0;
            motor_steps_complete[motor] = true;
        }
    }

    /**
     * @brief Clear motor alerts
     * @param motor Motor index (0-3)
     */
    void ClearAlerts(uint8_t motor) {
        if (motor < 4) {
            motion_canceled_neg_limit[motor] = false;
            motion_canceled_pos_limit[motor] = false;
            motion_canceled_estop[motor] = false;
        }
    }

    /**
     * @brief Trigger E-Stop for motor
     *
     * Simulates E-Stop activation - stops motor and sets alert.
     * @param motor Motor index (0-3)
     */
    void TriggerEStop(uint8_t motor) {
        if (motor < 4) {
            motion_canceled_estop[motor] = true;
            motor_moving[motor] = false;
            motor_velocity[motor] = 0;
            motor_steps_complete[motor] = true;
        }
    }
};

// Global fake state instance (defined in CutterHal_Fake.cpp)
extern FakeHalState g_fake;

// === Convenience Macros for Tests ===

#define RESET_HAL()             g_fake.Reset()
#define ADVANCE_TIME(ms)        g_fake.AdvanceTime(ms)
#define SET_PIN(pin, val)       g_fake.digital_pins[pin] = (val)
#define SET_PIN_EDGE(pin, val)  g_fake.SetDigitalPin(pin, val)  // Sets pin with edge detection
#define GET_PIN(pin)            g_fake.digital_pins[pin]
#define SET_ANALOG(pin, val)    g_fake.SetAnalogValue(pin, val)
#define COMPLETE_MOVE(motor)    g_fake.CompleteMotorMove(motor)
#define SET_HLFB(motor, state)  g_fake.SetHlfbState(motor, state)
#define SET_MOTOR_READY(m, r)   g_fake.SetMotorReady(m, r)
#define SET_PIN_FAULT(pin, f)   g_fake.pin_fault[pin] = (f)
#define SET_MOTOR_FAULT(m, f)   g_fake.motor_fault[m] = (f)
#define SET_MOVE_REJECTED(m, r) g_fake.move_rejected[m] = (r)
#define TRIGGER_NEG_LIMIT(m)    g_fake.TriggerNegativeLimit(m)
#define TRIGGER_POS_LIMIT(m)    g_fake.TriggerPositiveLimit(m)
#define SET_POS_LIMIT(m, a)     g_fake.SetPosLimit(m, a)
#define SET_NEG_LIMIT(m, a)     g_fake.SetNegLimit(m, a)
#define TRIGGER_ESTOP(m)        g_fake.TriggerEStop(m)
#define CLEAR_MOTOR_ALERTS(m)   g_fake.ClearAlerts(m)
