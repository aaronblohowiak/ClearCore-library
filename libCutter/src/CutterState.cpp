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
    : state_(State::UNCONNECTED)
    , epoch_(0)
    , error_code_(ErrorCode::NONE)
{
    error_message_[0] = '\0';
}

bool StateMachine::CanTransitionTo(State target) const {
    // From ERROR, only Reset() can change state
    if (state_ == State::ERROR) {
        return false;
    }

    // Valid transitions based on state diagram
    switch (state_) {
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
    state_ = target;
    return true;
}

void StateMachine::EnterError(ErrorCode code, const char* message) {
    state_ = State::ERROR;
    error_code_ = code;
    epoch_++;

    if (message) {
        strncpy(error_message_, message, sizeof(error_message_) - 1);
        error_message_[sizeof(error_message_) - 1] = '\0';
    } else {
        error_message_[0] = '\0';
    }
}

bool StateMachine::Reset() {
    if (state_ != State::ERROR) {
        return false;
    }

    state_ = State::CONNECTED;
    error_code_ = ErrorCode::NONE;
    error_message_[0] = '\0';
    return true;
}

void StateMachine::MarkConnected() {
    if (state_ == State::UNCONNECTED) {
        state_ = State::CONNECTED;
    }
}

void StateMachine::MarkDisconnected() {
    // Can disconnect from any state except ERROR
    if (state_ != State::ERROR) {
        state_ = State::UNCONNECTED;
    }
}

}  // namespace Cutter
