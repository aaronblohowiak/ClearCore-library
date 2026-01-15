/**
 * @file CutterHal.h
 * @brief Hardware Abstraction Layer for Cutter
 *
 * All ClearCore hardware access goes through this interface.
 * Two implementations exist:
 *   - CutterHal_ClearCore.cpp: Real hardware (CUTTER_PLATFORM_CLEARCORE)
 *   - CutterHal_Fake.cpp: Test fakes (CUTTER_PLATFORM_TEST)
 */

#pragma once

#include <stdint.h>

namespace CutterHal {

// === Pin Operations ===

/**
 * @brief Read digital pin state
 * @param pin Pin index (0-12)
 * @return true if pin is high, false if low
 */
bool ReadDigitalPin(uint8_t pin);

/**
 * @brief Write digital pin state
 * @param pin Pin index (0-5 for outputs)
 * @param value true for high, false for low
 */
void WriteDigitalPin(uint8_t pin, bool value);

/**
 * @brief Read analog pin value
 * @param pin Pin index (9-12 for analog capable pins)
 * @return 10-bit ADC value (0-4095)
 */
int16_t ReadAnalogPin(uint8_t pin);

/**
 * @brief Set PWM duty cycle
 * @param pin Pin index (0-5 for PWM capable pins)
 * @param duty Duty cycle (0-65535, maps to 0-100%)
 */
void SetPwmDuty(uint8_t pin, uint16_t duty);

/**
 * @brief Set PWM frequency
 * @param pin Pin index
 * @param freq Frequency in Hz
 */
void SetPwmFrequency(uint8_t pin, uint32_t freq);

/**
 * @brief Set H-Bridge output value
 * @param pin Pin index (4-5 for H-Bridge capable pins)
 * @param value Bidirectional value (-32767 to +32767)
 */
void SetHBridgeValue(uint8_t pin, int16_t value);

/**
 * @brief Start tone generation on H-Bridge pin
 * @param pin Pin index (4-5)
 * @param freq Frequency in Hz
 * @param amplitude Tone amplitude (0 to INT16_MAX)
 */
void StartTone(uint8_t pin, uint16_t freq, int16_t amplitude);

/**
 * @brief Stop tone generation
 * @param pin Pin index (4-5)
 */
void StopTone(uint8_t pin);

/**
 * @brief Configure pin mode
 * @param pin Pin index
 * @param mode Platform-specific mode value
 */
void ConfigurePinMode(uint8_t pin, uint8_t mode);

// === Motor Operations ===

/**
 * @brief Enable or disable motor
 * @param motor Motor index (0-3)
 * @param enable true to enable, false to disable
 */
void EnableMotor(uint8_t motor, bool enable);

/**
 * @brief Start relative move
 * @param motor Motor index (0-3)
 * @param steps Steps to move (signed)
 */
void MoveRelative(uint8_t motor, int32_t steps);

/**
 * @brief Start absolute move
 * @param motor Motor index (0-3)
 * @param position Target position in steps
 */
void MoveAbsolute(uint8_t motor, int32_t position);

/**
 * @brief Start velocity move
 * @param motor Motor index (0-3)
 * @param velocity Velocity in steps/sec (signed)
 */
void MoveVelocity(uint8_t motor, int32_t velocity);

/**
 * @brief Stop motor
 * @param motor Motor index (0-3)
 * @param immediate If true, hard stop; if false, decelerate
 */
void StopMotor(uint8_t motor, bool immediate);

/**
 * @brief Set motor motion parameters
 * @param motor Motor index (0-3)
 * @param velMax Maximum velocity in steps/sec
 * @param accelMax Maximum acceleration in steps/sec^2
 */
void SetMotorParams(uint8_t motor, int32_t velMax, int32_t accelMax);

/**
 * @brief Get current motor position
 * @param motor Motor index (0-3)
 * @return Position in steps
 */
int32_t GetMotorPosition(uint8_t motor);

/**
 * @brief Set motor position (without moving)
 * @param motor Motor index (0-3)
 * @param position New position value
 */
void SetMotorPosition(uint8_t motor, int32_t position);

/**
 * @brief Check if motor move is complete
 * @param motor Motor index (0-3)
 * @return true if no move in progress
 */
bool StepsComplete(uint8_t motor);

/**
 * @brief Get HLFB state for ClearPath motors
 * @param motor Motor index (0-3)
 * @return HLFB state code
 */
uint8_t GetHlfbState(uint8_t motor);

/**
 * @brief Check if motor is ready (enabled and not faulted)
 * @param motor Motor index (0-3)
 * @return true if motor is ready for commands
 */
bool IsMotorReady(uint8_t motor);

/**
 * @brief Check if motor is in hardware fault state
 * @param motor Motor index (0-3)
 * @return true if motor has a fault
 */
bool IsMotorInFault(uint8_t motor);

/**
 * @brief Check if digital output pin is in hardware fault (overcurrent)
 * @param pin Pin index (0-5 for outputs)
 * @return true if pin is in fault state
 */
bool IsPinInFault(uint8_t pin);

// === Timing ===

/**
 * @brief Get milliseconds since startup
 * @return Millisecond count (rolls over at ~49.7 days)
 */
uint32_t Milliseconds();

}  // namespace CutterHal
