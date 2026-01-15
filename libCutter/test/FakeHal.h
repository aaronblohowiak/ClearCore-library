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
    int digital_pin_writes[13] = {};      // Write count per pin (for verification)
    int16_t analog_pins[4] = {};          // Analog values for pins 9-12
    uint16_t pwm_duty[6] = {};            // PWM duty for pins 0-5
    int16_t hbridge_value[2] = {};        // H-Bridge values for pins 4-5
    bool tone_active[2] = {};             // Tone playing on pins 4-5
    uint16_t tone_freq[2] = {};           // Tone frequency
    int16_t tone_amplitude[2] = {};       // Tone amplitude

    // === Motor State ===
    bool motor_enabled[4] = {};
    bool motor_moving[4] = {};
    bool motor_steps_complete[4] = {true, true, true, true};
    int32_t motor_position[4] = {};
    int32_t motor_target[4] = {};
    int32_t motor_velocity[4] = {};
    int32_t motor_vel_max[4] = {};
    int32_t motor_accel_max[4] = {};
    uint8_t hlfb_state[4] = {};
    bool motor_ready[4] = {};

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
     */
    void CompleteMotorMove(uint8_t motor) {
        if (motor < 4) {
            motor_position[motor] = motor_target[motor];
            motor_moving[motor] = false;
            motor_steps_complete[motor] = true;
        }
    }

    /**
     * @brief Set end stop / digital input state
     * @param pin Pin index
     * @param active Active state
     */
    void TriggerEndStop(uint8_t pin, bool active) {
        if (pin < 13) {
            digital_pins[pin] = active;
        }
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
};

// Global fake state instance (defined in CutterHal_Fake.cpp)
extern FakeHalState g_fake;

// === Convenience Macros for Tests ===

#define RESET_HAL()             g_fake.Reset()
#define ADVANCE_TIME(ms)        g_fake.AdvanceTime(ms)
#define SET_PIN(pin, val)       g_fake.digital_pins[pin] = (val)
#define GET_PIN(pin)            g_fake.digital_pins[pin]
#define SET_ANALOG(pin, val)    g_fake.SetAnalogValue(pin, val)
#define COMPLETE_MOVE(motor)    g_fake.CompleteMotorMove(motor)
#define SET_HLFB(motor, state)  g_fake.SetHlfbState(motor, state)
#define SET_MOTOR_READY(m, r)   g_fake.SetMotorReady(m, r)
