/**
 * @file CutterState.cpp
 * @brief State machine implementation
 */

#include "CutterState.h"
#include <cstring>

namespace Cutter {

const char* StateName(State s) {
    switch (s) {
        case State::UNCONNECTED: return "unconnected";
        case State::CONNECTED:   return "connected";
        case State::CONFIGURED:  return "configured";
        case State::ENABLING:    return "enabling";
        case State::READY:       return "ready";
        case State::WORKING:     return "working";
        case State::ERROR:       return "error";
        default:                 return "unknown";
    }
}

StateMachine::StateMachine()
    : m_state(State::UNCONNECTED)
    , m_epoch(0)
    , m_errorCode(ErrorCode::NONE)
{
    m_errorMessage[0] = '\0';
}

bool StateMachine::CanTransitionTo(State target) const {
    // From ERROR, only Reset() can change state
    if (m_state == State::ERROR) {
        return false;
    }

    // Valid transitions based on state diagram
    switch (m_state) {
        case State::UNCONNECTED:
            return target == State::CONNECTED;

        case State::CONNECTED:
            return target == State::CONFIGURED ||
                   target == State::UNCONNECTED;

        case State::CONFIGURED:
            return target == State::ENABLING ||
                   target == State::CONNECTED ||
                   target == State::UNCONNECTED;

        case State::ENABLING:
            return target == State::READY ||
                   target == State::CONFIGURED ||
                   target == State::UNCONNECTED;

        case State::READY:
            return target == State::WORKING ||
                   target == State::CONFIGURED ||
                   target == State::UNCONNECTED;

        case State::WORKING:
            return target == State::READY ||
                   target == State::CONFIGURED ||
                   target == State::UNCONNECTED;

        default:
            return false;
    }
}

bool StateMachine::TransitionTo(State target) {
    if (!CanTransitionTo(target)) {
        return false;
    }
    m_state = target;
    return true;
}

void StateMachine::EnterError(ErrorCode code, const char* message) {
    m_state = State::ERROR;
    m_errorCode = code;
    m_epoch++;

    if (message) {
        strncpy(m_errorMessage, message, sizeof(m_errorMessage) - 1);
        m_errorMessage[sizeof(m_errorMessage) - 1] = '\0';
    } else {
        m_errorMessage[0] = '\0';
    }
}

bool StateMachine::Reset() {
    if (m_state != State::ERROR) {
        return false;
    }

    m_state = State::CONNECTED;
    m_errorCode = ErrorCode::NONE;
    m_errorMessage[0] = '\0';
    return true;
}

void StateMachine::MarkConnected() {
    if (m_state == State::UNCONNECTED) {
        m_state = State::CONNECTED;
    }
}

void StateMachine::MarkDisconnected() {
    // Can disconnect from any state except ERROR
    if (m_state != State::ERROR) {
        m_state = State::UNCONNECTED;
    }
}

}  // namespace Cutter
