/**
    \file CutterState.h
    \brief State machine for Cutter controller

    States:
      - UNCONNECTED: No host connection
      - CONNECTED: Host connected, awaiting configuration
      - CONFIGURED: Pins/motors configured, not enabled
      - ENABLING: Motors enabling (waiting for HLFB)
      - READY: System ready for motion commands
      - WORKING: Motion in progress
      - ERROR: Error state, requires reset
**/

#pragma once

#include <stdint.h>

namespace Cutter {

/**
    \brief Controller states
**/
enum class State : uint8_t {
    UNCONNECTED = 0,
    CONNECTED,
    CONFIGURED,
    ENABLING,
    READY,
    WORKING,
    ERROR
};

/**
 * \brief Get state name as string
 */
const char* StateName(State s);

/**
 * \brief Error codes
 */
enum class ErrorCode : uint32_t {
    NONE = 0,

    // Protocol errors (100-199)
    UNKNOWN_COMMAND = 100,
    INVALID_PARAM = 101,
    MISSING_PARAM = 102,
    INVALID_STATE = 103,
    EPOCH_MISMATCH = 104,
    STALE_SEQ = 105,        ///< Sequence number is stale (lower than previously seen)

    // Pin errors (200-299)
    INVALID_PIN = 200,
    PIN_NOT_CONFIGURED = 201,
    PIN_CAPABILITY = 202,
    PIN_OVERCURRENT = 203,
    PIN_TIMEOUT = 204,
    PIN_ERROR_TRIGGER = 205,
    ANALOG_THRESHOLD = 206,

    // Motor errors (300-399)
    INVALID_MOTOR = 300,
    MOTOR_NOT_CONFIGURED = 301,
    MOTOR_FAULT = 302,
    MOTOR_NOT_READY = 303,
    HLFB_TIMEOUT = 304,
    SOFT_LIMIT = 305,
    EXCEEDS_LIMIT = 306,    ///< Per-move vel/accel exceeds motor max

    // System errors (400-499)
    INTERNAL_ERROR = 400,
    EMERGENCY_STOP = 401,
};

/**
 * \brief State machine manager
 *
 * Tracks current state, epoch, and handles transitions.
 */
class StateMachine {
public:
    StateMachine();

    /**
     * \brief Get current state
     */
    State GetState() const { return m_state; }

    /**
     * \brief Get current epoch
     */
    uint32_t GetEpoch() const { return m_epoch; }

    /**
     * \brief Get current error code (if in ERROR state)
     */
    ErrorCode GetErrorCode() const { return m_errorCode; }

    /**
     * \brief Get error message (if in ERROR state)
     */
    const char* GetErrorMessage() const { return m_errorMessage; }

    /**
     * \brief Check if a state transition is valid
     */
    bool CanTransitionTo(State target) const;

    /**
     * \brief Transition to a new state
     * \return true if transition succeeded
     */
    bool TransitionTo(State target);

    /**
     * \brief Enter error state
     * \param code Error code
     * \param message Error message (will be copied)
     */
    void EnterError(ErrorCode code, const char* message);

    /**
     * \brief Reset from error state
     * \return true if reset succeeded
     */
    bool Reset();

    /**
     * \brief Mark as connected (host connected)
     */
    void MarkConnected();

    /**
     * \brief Mark as disconnected (host disconnected)
     */
    void MarkDisconnected();

private:
    State m_state;
    uint32_t m_epoch;
    ErrorCode m_errorCode;
    char m_errorMessage[64];
};

}  // namespace Cutter
