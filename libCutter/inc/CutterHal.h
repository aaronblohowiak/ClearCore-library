/**
    \file CutterHal.h
    \brief Hardware Abstraction Layer for Cutter

    All ClearCore hardware access goes through this interface.
    Two implementations exist:
      - CutterHal_ClearCore.cpp: Real hardware (CUTTER_PLATFORM_CLEARCORE)
      - CutterHal_Fake.cpp: Test fakes (CUTTER_PLATFORM_TEST)
**/

#pragma once

#include <stdint.h>

namespace CutterHal {

// === Pin Operations ===

/**
    \brief Read digital pin state

    \param[in] pin Pin index (0-12)
    \return true if pin is high, false if low
**/
bool ReadDigitalPin(uint8_t pin);

/**
    \brief Write digital pin state

    \param[in] pin Pin index (0-5 for outputs)
    \param[in] value true for high, false for low
**/
void WriteDigitalPin(uint8_t pin, bool value);

/**
    \brief Read analog pin value

    \param[in] pin Pin index (9-12 for analog capable pins)
    \return 12-bit ADC value (0-4095)
**/
int16_t ReadAnalogPin(uint8_t pin);

/**
    \brief Set PWM duty cycle

    \param[in] pin Pin index (0-5 for PWM capable pins)
    \param[in] duty Duty cycle (0-65535, maps to 0-100%)
**/
void SetPwmDuty(uint8_t pin, uint16_t duty);

/**
    \brief Set PWM frequency

    \param[in] pin Pin index (0-5)
    \param[in] freq Frequency in Hz
**/
void SetPwmFrequency(uint8_t pin, uint32_t freq);

/**
    \brief Set H-Bridge output value

    \param[in] pin Pin index (4-5 for H-Bridge capable pins)
    \param[in] value Bidirectional value (-32767 to +32767)
**/
void SetHBridgeValue(uint8_t pin, int16_t value);

/**
    \brief Start tone generation on H-Bridge pin

    \param[in] pin Pin index (4-5)
    \param[in] freq Frequency in Hz
    \param[in] amplitude Tone amplitude (0 to INT16_MAX)
**/
void StartTone(uint8_t pin, uint16_t freq, int16_t amplitude);

/**
    \brief Stop tone generation

    \param[in] pin Pin index (4-5)
**/
void StopTone(uint8_t pin);

/**
    \brief Configure pin mode

    \param[in] pin Pin index
    \param[in] mode Platform-specific mode value
**/
void ConfigurePinMode(uint8_t pin, uint8_t mode);

// === Motor Operations ===

/**
    \brief Enable or disable motor

    \param[in] motor Motor index (0-3)
    \param[in] enable true to enable, false to disable
**/
void EnableMotor(uint8_t motor, bool enable);

/**
    \brief Start relative move

    \param[in] motor Motor index (0-3)
    \param[in] steps Steps to move (signed)
**/
void MoveRelative(uint8_t motor, int32_t steps);

/**
    \brief Start absolute move

    \param[in] motor Motor index (0-3)
    \param[in] position Target position in steps
**/
void MoveAbsolute(uint8_t motor, int32_t position);

/**
    \brief Start velocity move

    \param[in] motor Motor index (0-3)
    \param[in] velocity Velocity in steps/sec (signed)
**/
void MoveVelocity(uint8_t motor, int32_t velocity);

/**
    \brief Stop motor

    \param[in] motor Motor index (0-3)
    \param[in] immediate If true, hard stop; if false, decelerate
**/
void StopMotor(uint8_t motor, bool immediate);

/**
    \brief Set motor motion parameters

    \param[in] motor Motor index (0-3)
    \param[in] velMax Maximum velocity in steps/sec
    \param[in] accelMax Maximum acceleration in steps/sec^2
**/
void SetMotorParams(uint8_t motor, int32_t velMax, int32_t accelMax);

/**
    \brief Get current motor position

    \param[in] motor Motor index (0-3)
    \return Position in steps
**/
int32_t GetMotorPosition(uint8_t motor);

/**
    \brief Set motor position (without moving)

    \param[in] motor Motor index (0-3)
    \param[in] position New position value
**/
void SetMotorPosition(uint8_t motor, int32_t position);

/**
    \brief Check if motor move is complete

    \param[in] motor Motor index (0-3)
    \return true if no move in progress
**/
bool StepsComplete(uint8_t motor);

/**
    \brief HLFB state constants (ClearPath motors)
**/
constexpr uint8_t HLFB_DEASSERTED = 0;      ///< Not in position / torque limit hit
constexpr uint8_t HLFB_ASSERTED = 1;        ///< In position (move complete)
constexpr uint8_t HLFB_HAS_MEASUREMENT = 2; ///< PWM mode with speed/torque measurement
constexpr uint8_t HLFB_UNKNOWN = 3;         ///< State unknown / transitioning

/**
    \brief Get HLFB state for ClearPath motors

    \param[in] motor Motor index (0-3)
    \return HLFB state code
**/
uint8_t GetHlfbState(uint8_t motor);

/**
    \brief Check if motor is ready (enabled and not faulted)

    \param[in] motor Motor index (0-3)
    \return true if motor is ready for commands
**/
bool IsMotorReady(uint8_t motor);

/**
    \brief Check if motor is in hardware fault state

    \param[in] motor Motor index (0-3)
    \return true if motor has a fault
**/
bool IsMotorInFault(uint8_t motor);

/**
    \brief Check if motor has alerts present (ClearPath)

    Motion will be prevented if any Alert Register bits are set.
    Should be false for move to be considered complete.

    \param[in] motor Motor index (0-3)
    \return true if motor has alerts (not ready for moves)
**/
bool HasMotorAlerts(uint8_t motor);

/**
    \brief Check if digital output pin is in hardware fault (overcurrent)

    \param[in] pin Pin index (0-5 for outputs)
    \return true if pin is in fault state
**/
bool IsPinInFault(uint8_t pin);

// === Limit Switches ===

/// Invalid pin constant (matches ClearCore's CLEARCORE_PIN_INVALID)
constexpr uint8_t PIN_INVALID = 255;

/**
    \brief Configure negative direction limit switch for motor

    The pin must be configured as digital input before calling this.
    Uses NC (normally closed) switch logic - motion stops when pin goes low.

    \param[in] motor Motor index (0-3)
    \param[in] pin Pin to use as limit switch (or PIN_INVALID to disable)
    \return true if successfully configured
**/
bool SetLimitSwitchNeg(uint8_t motor, uint8_t pin);

/**
    \brief Configure positive direction limit switch for motor

    The pin must be configured as digital input before calling this.
    Uses NC (normally closed) switch logic - motion stops when pin goes low.

    \param[in] motor Motor index (0-3)
    \param[in] pin Pin to use as limit switch (or PIN_INVALID to disable)
    \return true if successfully configured
**/
bool SetLimitSwitchPos(uint8_t motor, uint8_t pin);

/**
    \brief Check if motion was canceled due to negative limit switch

    \param[in] motor Motor index (0-3)
    \return true if MotionCanceledNegativeLimit alert is set
**/
bool HasMotionCanceledNegLimit(uint8_t motor);

/**
    \brief Check if motion was canceled due to positive limit switch

    \param[in] motor Motor index (0-3)
    \return true if MotionCanceledPositiveLimit alert is set
**/
bool HasMotionCanceledPosLimit(uint8_t motor);

/**
    \brief Clear motor alerts

    Must be called after limit switch triggers to allow further motion.

    \param[in] motor Motor index (0-3)
**/
void ClearMotorAlerts(uint8_t motor);

// === Timing ===

/**
    \brief Get milliseconds since startup

    \return Millisecond count (rolls over at ~49.7 days)
**/
uint32_t Milliseconds();

}  // namespace CutterHal
