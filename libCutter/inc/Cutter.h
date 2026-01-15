/**
    \file Cutter.h
    \brief Main Cutter controller

    The main entry point for Cutter. Manages state, processes commands,
    monitors pins/motors, and generates events.

    \code{.cpp}
    // Basic usage with USB serial
    Cutter::Controller ctrl(&ConnectorUsb);

    while (true) {
        ctrl.Update();  // Call frequently in main loop
    }
    \endcode
**/

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
    \brief Serial interface abstraction

    Implement this interface to connect Cutter to USB, Ethernet, etc.
    ClearCore's SerialUsb and SerialDriver classes are compatible.

    \code{.cpp}
    // Using ClearCore's USB serial
    Cutter::Controller ctrl(&ConnectorUsb);

    // Or wrap a custom transport
    class MySerial : public Cutter::ISerial {
        int16_t CharGet() override { return myTransport.read(); }
        // ... implement other methods
    };
    \endcode
**/
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
    \brief Pin operating mode
**/
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
    \brief Motor type
**/
enum class MotorType : uint8_t {
    UNCONFIGURED = 0,
    CLEARPATH,
    GENERIC_STEPPER,
};

/**
    \brief Homing state machine states

    Used for both generic steppers (endstop homing) and SDSK motors (hard-stop homing).

    Stepper homing sequence: IDLE -> SEEKING -> BACKING_OFF -> LATCHING -> COMPLETE
    SDSK homing sequence: IDLE -> SDSK_SEEKING -> SDSK_CONFIRMED -> COMPLETE
**/
enum class HomingState : uint8_t {
    IDLE = 0,           ///< Not homing
    SEEKING,            ///< Stepper: moving toward endstop at seek velocity
    BACKING_OFF,        ///< Stepper: backing away from endstop
    LATCHING,           ///< Stepper: slow approach for precise contact
    SDSK_SEEKING,       ///< SDSK: moving toward hard stop, monitoring HLFB
    SDSK_CONFIRMED,     ///< SDSK: hard stop detected, setting position
    COMPLETE,           ///< Homing complete, position set to zero
};

/**
    \brief Digital input runtime state
**/
struct DigitalInState {
    bool last_value;
    bool report_changes;
    bool invert;                ///< Invert the pin value before processing
    bool error_trigger_enabled; ///< Enter error state when trigger value seen
    bool error_trigger_value;   ///< Value that triggers error (after invert)
};

/**
    \brief Digital output runtime state
**/
struct DigitalOutState {
    bool current_value;
    bool on_error_value;        ///< Value to set when entering error state
    bool on_error_enabled;      ///< Apply on_error_value on error
    uint32_t max_raised_ms;     ///< Max time pin can be high (0=disabled)
    uint32_t raise_start_time;  ///< When pin was last set high
};

/**
    \brief Analog input runtime state
**/
struct AnalogInState {
    int16_t last_value;
    int16_t error_threshold_low;    ///< Enter error if value below this
    int16_t error_threshold_high;   ///< Enter error if value above this
    bool error_threshold_enabled;
    uint32_t report_interval_ms;    ///< 0=disabled, else ms between reports
    uint32_t last_report_time;
};

/**
    \brief PWM output runtime state
**/
struct PwmState {
    uint16_t duty;
    uint32_t frequency;
};

/**
    \brief H-Bridge output runtime state
**/
struct HBridgeState {
    int16_t value;
    bool tone_active;
    uint16_t tone_freq;
    int16_t tone_amplitude;
};

/**
    \brief End stop runtime state (digital input used for homing)
**/
struct EndStopState {
    uint8_t triggered_value;  ///< 0 = triggered when LOW (NC, fail-safe), 1 = triggered when HIGH (NO)
    bool last_value;
};

/**
    \brief Pin slot with tagged union for mode-specific state
**/
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
    \brief Motor slot containing configuration and runtime state

    \par Homing Strategy
    Motors are enabled and homed sequentially by enable_priority (lower = first).
    - Generic steppers: Use endstop homing (seek -> backoff -> latch sequence)
    - SDSK/ClearPath: Use HLFB hard-stop homing (move into mechanical stop, detect via torque)

    \par Fail-Safe Endstops
    For generic steppers, end_stop_triggered defaults to 0, meaning the endstop
    is triggered when the pin reads LOW. This assumes normally-closed (NC) switches
    which fail safe: a broken wire reads the same as a triggered switch.
**/
struct MotorSlot {
    MotorType type;
    uint8_t motor_index;
    bool enabled;
    bool moving;
    uint32_t move_seq;          ///< Sequence number of current move

    // Common enable/homing configuration
    uint8_t enable_priority;    ///< Enable/home order (lower = earlier, default = motor index)
    bool home_on_enable;        ///< Automatically home when enabled via enable_all
    bool homed;                 ///< Motor has been homed since last enable

    // Motion parameters
    int32_t vel_max;
    int32_t accel_max;

    // Soft limits
    bool soft_limits_enabled;
    int32_t soft_limit_min;
    int32_t soft_limit_max;

    // Homing state (both motor types)
    HomingState homing_state;

    // Generic stepper homing (endstop-based)
    uint8_t end_stop_pin;           ///< Pin configured as END_STOP for this motor
    uint8_t end_stop_triggered;     ///< 0 = triggered when LOW (NC default), 1 = triggered when HIGH (NO)
    int32_t homing_seek_velocity;   ///< Fast approach velocity (steps/sec, negative = toward endstop)
    int32_t homing_latch_velocity;  ///< Slow precision velocity (steps/sec)
    int32_t homing_backoff_distance;///< Distance to back off after first contact (steps)

    // SDSK/ClearPath homing (hard-stop based)
    int32_t homing_direction;       ///< Direction to home: -1 = negative, 1 = positive
    int32_t homing_torque_limit;    ///< HLFB torque % that indicates hard stop (0 = use default)

    // SDSK/ClearPath runtime state
    uint8_t last_hlfb_state;        ///< For detecting HLFB changes
    uint32_t enable_start_time;     ///< When motor was enabled (for HLFB timeout)
    uint32_t hlfb_timeout_ms;       ///< Max time to wait for HLFB after enable
};

/**
    \brief Main Cutter controller

    Processes commands received over serial, monitors hardware state,
    and generates events. Call Update() frequently in your main loop.

    \code{.cpp}
    // Initialize with serial interface
    Cutter::Controller ctrl(&ConnectorUsb);

    // Main loop
    while (true) {
        ctrl.Update();

        // Check state if needed
        if (ctrl.GetState() == Cutter::State::ERROR) {
            // Handle error condition
        }
    }
    \endcode

    Commands are sent as text lines. Example session:
    \code
    -> ping
    <- ok
    -> configure_digital_out pin=0
    <- ok pin=0
    -> write_pin pin=0 value=1
    <- ok
    -> configure_stepper motor=0 vel_max=10000 accel_max=50000
    <- ok motor=0
    -> enable motor=0
    <- ok
    -> move motor=0 steps=1000 seq=1
    <- ok seq=1
    <- event type=done motor=0 seq=1 position=1000
    \endcode
**/
class Controller {
public:
    /**
        \brief Construct controller with serial interface

        \code{.cpp}
        Cutter::Controller ctrl(&ConnectorUsb);
        \endcode

        \param[in] serial Pointer to serial interface (USB, UART, etc.)
    **/
    explicit Controller(ISerial* serial);

    /**
        \brief Main update loop - call frequently

        Processes incoming commands, checks pin/motor state for faults
        and events, and sends responses. Should be called as frequently
        as possible in your main loop.

        \code{.cpp}
        while (true) {
            ctrl.Update();
        }
        \endcode
    **/
    void Update();

    /**
        \brief Get current state machine state

        \code{.cpp}
        if (ctrl.GetState() == Cutter::State::ERROR) {
            // Handle error - may need reset command
        }
        \endcode

        \return Current state (UNCONNECTED, CONNECTED, CONFIGURED, etc.)
    **/
    State GetState() const { return m_stateMachine.GetState(); }

    /**
        \brief Get current epoch

        Epoch increments on each error. Commands with mismatched epoch
        are rejected, preventing stale commands from executing.

        \return Current epoch value
    **/
    uint32_t GetEpoch() const { return m_stateMachine.GetEpoch(); }

    /**
        \brief Send an event to the host

        \param[in] type Event type string
    **/
    void SendEvent(const char* type);

    /**
        \brief Get response writer for building responses

        \return Reference to response writer
    **/
    ResponseWriter& Response() { return m_response; }

    /**
        \brief Send the current response buffer
    **/
    void SendResponse();

    /**
        \brief Get pin slot by index

        \param[in] index Pin index (0-12)
        \return Pointer to pin slot, or nullptr if invalid
    **/
    PinSlot* GetPin(uint8_t index);

    /**
        \brief Get pin slot by index (const)

        \param[in] index Pin index (0-12)
        \return Const pointer to pin slot, or nullptr if invalid
    **/
    const PinSlot* GetPin(uint8_t index) const;

    /**
        \brief Get motor slot by index

        \param[in] index Motor index (0-3)
        \return Pointer to motor slot, or nullptr if invalid
    **/
    MotorSlot* GetMotor(uint8_t index);

    /**
        \brief Get motor slot by index (const)

        \param[in] index Motor index (0-3)
        \return Const pointer to motor slot, or nullptr if invalid
    **/
    const MotorSlot* GetMotor(uint8_t index) const;

    /**
        \brief Get state machine for direct access

        \return Reference to state machine
    **/
    StateMachine& GetStateMachine() { return m_stateMachine; }

    /**
        \brief Set enable_all sequence number for completion event

        \param[in] seq Sequence number from enable_all command
    **/
    void SetEnableAllSeq(uint32_t seq);

    /**
        \brief Check if enable_all sequence is in progress

        \return true if enable_all is active and homing motors
    **/
    bool IsEnableAllActive() const { return m_enableAllActive; }

    /**
        \brief Start homing the next motor in priority order
    **/
    void StartNextHoming();

private:
    // Serial communication
    ISerial* m_serial;
    char m_inputBuffer[MAX_COMMAND_LENGTH];
    size_t m_inputPos;

    // State
    StateMachine m_stateMachine;

    // Response buffer
    char m_responseBuffer[MAX_RESPONSE_LENGTH];
    ResponseWriter m_response;

    // Parser
    CommandParser m_parser;

    // Pin slots
    PinSlot m_pins[NUM_PINS];

    // Motor slots
    MotorSlot m_motors[NUM_MOTORS];

    // Internal methods
    void ProcessInput();
    bool ReadLine();
    void DispatchCommand(const ParsedCommand& cmd);
    void CheckPins();
    void CheckMotors();

    // Sequence tracking
    uint32_t m_nextSeq;

    // Enable-all state (sequential enable and homing)
    bool m_enableAllActive;         ///< enable_all command in progress
    uint32_t m_enableAllSeq;        ///< Sequence number for enable_all completion event
    uint8_t m_homingOrder[NUM_MOTORS];  ///< Motor indices sorted by enable_priority
    uint8_t m_homingCount;          ///< Number of motors to home
    uint8_t m_currentHomingIndex;   ///< Index into m_homingOrder for current motor

    // Built-in commands
    void CmdPing(const ParsedCommand& cmd);
    void CmdReset(const ParsedCommand& cmd);
    void CmdStatus(const ParsedCommand& cmd);
    void CmdVersion(const ParsedCommand& cmd);
    void CmdEmergencyStop(const ParsedCommand& cmd);
    void CmdGetNextSeq(const ParsedCommand& cmd);
};

}  // namespace Cutter
