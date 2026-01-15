/**
 * @file Cutter.h
 * @brief Main Cutter controller
 *
 * The main entry point for Cutter. Manages state, processes commands,
 * monitors pins/motors, and generates events.
 */

#pragma once

#include "CutterConfig.h"
#include "CutterState.h"
#include "CommandParser.h"
#include "CutterResponse.h"
#include "CutterHal.h"
#include <stdint.h>

namespace Cutter {

// Forward declarations
struct PinSlot;
struct MotorSlot;

/**
 * @brief Serial interface abstraction
 *
 * Implement this interface to connect Cutter to USB, Ethernet, etc.
 */
class ISerial {
public:
    virtual ~ISerial() = default;
    virtual int16_t CharGet() = 0;
    virtual int16_t CharPeek() = 0;
    virtual int AvailableForRead() = 0;
    virtual bool SendChar(char c) = 0;
    virtual bool Send(const char* str) = 0;
};

/**
 * @brief Pin mode
 */
enum class PinMode : uint8_t {
    UNCONFIGURED = 0,
    DIGITAL_IN,
    DIGITAL_OUT,
    ANALOG_IN,
    PWM,
    H_BRIDGE,
    END_STOP,
};

/**
 * @brief Motor type
 */
enum class MotorType : uint8_t {
    UNCONFIGURED = 0,
    CLEARPATH,
    GENERIC_STEPPER,
};

/**
 * @brief Homing state for generic steppers
 */
enum class HomingState : uint8_t {
    IDLE = 0,
    SEEKING,
    BACKING_OFF,
    LATCHING,
    COMPLETE,
};

/**
 * @brief Digital input state
 */
struct DigitalInState {
    bool last_value;
    bool report_changes;
};

/**
 * @brief Digital output state
 */
struct DigitalOutState {
    bool current_value;
};

/**
 * @brief Analog input state
 */
struct AnalogInState {
    int16_t last_value;
    int16_t threshold_low;
    int16_t threshold_high;
    bool report_threshold;
    uint32_t sample_interval_ms;
    uint32_t last_sample_time;
};

/**
 * @brief PWM output state
 */
struct PwmState {
    uint16_t duty;
    uint32_t frequency;
};

/**
 * @brief H-Bridge output state
 */
struct HBridgeState {
    int16_t value;
    bool tone_active;
    uint16_t tone_freq;
    int16_t tone_amplitude;
};

/**
 * @brief End stop state (digital input used for homing)
 */
struct EndStopState {
    bool active_high;
    bool last_value;
};

/**
 * @brief Pin slot with tagged union
 */
struct PinSlot {
    PinMode mode;
    uint8_t pin_index;
    union {
        DigitalInState digital_in;
        DigitalOutState digital_out;
        AnalogInState analog_in;
        PwmState pwm;
        HBridgeState hbridge;
        EndStopState end_stop;
    };
};

/**
 * @brief Motor slot
 */
struct MotorSlot {
    MotorType type;
    uint8_t motor_index;
    bool enabled;
    bool moving;
    uint32_t move_seq;  // Sequence number of current move

    // Motion parameters
    int32_t vel_max;
    int32_t accel_max;

    // Soft limits
    bool soft_limits_enabled;
    int32_t soft_limit_min;
    int32_t soft_limit_max;

    // Homing (generic stepper only)
    HomingState homing_state;
    uint8_t end_stop_pin;
    bool end_stop_active_high;
    int32_t homing_seek_velocity;
    int32_t homing_latch_velocity;
    int32_t homing_backoff_distance;

    // Completion tracking
    uint32_t enable_start_time;
    uint32_t hlfb_timeout_ms;
};

/**
 * @brief Main Cutter controller
 */
class Controller {
public:
    /**
     * @brief Construct controller with serial interface
     */
    explicit Controller(ISerial* serial);

    /**
     * @brief Main update loop - call frequently
     *
     * Processes incoming commands, updates pin/motor state,
     * and generates events.
     */
    void Update();

    /**
     * @brief Get current state
     */
    State GetState() const { return state_machine_.GetState(); }

    /**
     * @brief Get current epoch
     */
    uint32_t GetEpoch() const { return state_machine_.GetEpoch(); }

    /**
     * @brief Send an event to the host
     */
    void SendEvent(const char* type);

    /**
     * @brief Get response writer for building responses
     */
    ResponseWriter& Response() { return response_; }

    /**
     * @brief Send the current response buffer
     */
    void SendResponse();

    // Pin access for command handlers
    PinSlot* GetPin(uint8_t index);
    const PinSlot* GetPin(uint8_t index) const;

    // Motor access for command handlers
    MotorSlot* GetMotor(uint8_t index);
    const MotorSlot* GetMotor(uint8_t index) const;

    // State machine access
    StateMachine& GetStateMachine() { return state_machine_; }

private:
    // Serial communication
    ISerial* serial_;
    char input_buffer_[MAX_COMMAND_LENGTH];
    size_t input_pos_;

    // State
    StateMachine state_machine_;

    // Response buffer
    char response_buffer_[MAX_RESPONSE_LENGTH];
    ResponseWriter response_;

    // Parser
    CommandParser parser_;

    // Pin slots
    PinSlot pins_[NUM_PINS];

    // Motor slots
    MotorSlot motors_[NUM_MOTORS];

    // Internal methods
    void ProcessInput();
    bool ReadLine();
    void DispatchCommand(const ParsedCommand& cmd);
    void CheckPins();
    void CheckMotors();

    // Built-in commands
    void CmdPing(const ParsedCommand& cmd);
    void CmdReset(const ParsedCommand& cmd);
    void CmdStatus(const ParsedCommand& cmd);
    void CmdVersion(const ParsedCommand& cmd);
    void CmdEmergencyStop(const ParsedCommand& cmd);
};

}  // namespace Cutter
