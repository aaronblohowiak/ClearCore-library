/**
 * \file CutterConfig.h
 * \brief Compile-time configuration constants for Cutter
 *
 * All buffer sizes, limits, and pin capability tables are defined here.
 * No dynamic allocation - everything uses fixed-size arrays.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace Cutter {

// === Buffer Sizes ===

/// Maximum length of a command line (including null terminator)
/// Sized to handle worst-case valid commands (e.g., configure_stepper with all
/// parameters at max int32 values is ~345 bytes)
constexpr size_t MAX_COMMAND_LENGTH = 512;

/// Maximum length of a parameter key
constexpr size_t MAX_KEY_LENGTH = 32;

/// Maximum length of a parameter value
constexpr size_t MAX_VALUE_LENGTH = 64;

/// Maximum number of key=value pairs per command
constexpr size_t MAX_PARAMS = 16;

/// Maximum length of a response line
constexpr size_t MAX_RESPONSE_LENGTH = 512;

// === Hardware Limits ===

/// Number of I/O pins (IO-0 through A-12)
constexpr size_t NUM_PINS = 13;

/// Number of motor connectors (M-0 through M-3)
constexpr size_t NUM_MOTORS = 4;

// === Pin Capabilities ===
// Each pin has specific capabilities based on ClearCore hardware

/**
 * \brief Pin capability flags
 */
enum class PinCap : uint8_t {
    NONE        = 0,
    DIGITAL_IN  = (1 << 0),  // All pins (0-12)
    DIGITAL_OUT = (1 << 1),  // Pins 0-5 only
    PWM         = (1 << 2),  // Pins 0-5 only
    ANALOG_IN   = (1 << 3),  // Pins 9-12 only
    H_BRIDGE    = (1 << 4),  // Pins 4-5 only
};

// Bitwise OR operator for PinCap (constexpr for use in constant expressions)
constexpr PinCap operator|(PinCap a, PinCap b) {
    return static_cast<PinCap>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

constexpr bool operator&(PinCap caps, PinCap flag) {
    return (static_cast<uint8_t>(caps) & static_cast<uint8_t>(flag)) != 0;
}

/**
 * \brief Pin capability table
 *
 * Index by pin number to get capabilities.
 * Pin 0-3: Digital I/O + PWM
 * Pin 4-5: Digital I/O + PWM + H-Bridge
 * Pin 6-8: Digital input only
 * Pin 9-12: Digital input + Analog input
 */
constexpr PinCap PIN_CAPABILITIES[NUM_PINS] = {
    // IO-0 to IO-3: Digital I/O + PWM
    PinCap::DIGITAL_IN | PinCap::DIGITAL_OUT | PinCap::PWM,
    PinCap::DIGITAL_IN | PinCap::DIGITAL_OUT | PinCap::PWM,
    PinCap::DIGITAL_IN | PinCap::DIGITAL_OUT | PinCap::PWM,
    PinCap::DIGITAL_IN | PinCap::DIGITAL_OUT | PinCap::PWM,

    // IO-4 to IO-5: Digital I/O + PWM + H-Bridge
    PinCap::DIGITAL_IN | PinCap::DIGITAL_OUT | PinCap::PWM | PinCap::H_BRIDGE,
    PinCap::DIGITAL_IN | PinCap::DIGITAL_OUT | PinCap::PWM | PinCap::H_BRIDGE,

    // DI-6 to DI-8: Digital input only
    PinCap::DIGITAL_IN,
    PinCap::DIGITAL_IN,
    PinCap::DIGITAL_IN,

    // A-9 to A-12: Digital input + Analog input
    PinCap::DIGITAL_IN | PinCap::ANALOG_IN,
    PinCap::DIGITAL_IN | PinCap::ANALOG_IN,
    PinCap::DIGITAL_IN | PinCap::ANALOG_IN,
    PinCap::DIGITAL_IN | PinCap::ANALOG_IN,
};

/**
 * \brief Check if a pin supports a capability
 * \param pin Pin index (0-12)
 * \param cap Capability to check
 * \return true if pin supports the capability
 */
inline bool PinSupports(uint8_t pin, PinCap cap) {
    if (pin >= NUM_PINS) return false;
    return PIN_CAPABILITIES[pin] & cap;
}

// === Protocol Version ===

// Both are SemVer strings, NOT numbers - hosts must compare them as strings,
// never parse as floats (e.g. "1.10" != "1.1"). They are emitted unquoted only
// because they contain no spaces. Protocol bumps minor for backward-compatible
// additions, major for breaking changes.
constexpr const char* CUTTER_VERSION = "1.1.0";
constexpr const char* PROTOCOL_VERSION = "1.1";

}  // namespace Cutter
