/**
 * @file test_state_machine.cpp
 * @brief State machine tests
 */

#include <gtest/gtest.h>
#include "CutterState.h"

using namespace Cutter;

class StateMachineTest : public ::testing::Test {
protected:
    StateMachine sm;
};

// === Initial State ===

TEST_F(StateMachineTest, InitialState) {
    EXPECT_EQ(sm.GetState(), State::UNCONNECTED);
    EXPECT_EQ(sm.GetEpoch(), 0u);
    EXPECT_EQ(sm.GetErrorCode(), ErrorCode::NONE);
}

// === State Name ===

TEST_F(StateMachineTest, StateNames) {
    EXPECT_STREQ(StateName(State::UNCONNECTED), "unconnected");
    EXPECT_STREQ(StateName(State::CONNECTED), "connected");
    EXPECT_STREQ(StateName(State::CONFIGURED), "configured");
    EXPECT_STREQ(StateName(State::ENABLING), "enabling");
    EXPECT_STREQ(StateName(State::READY), "ready");
    EXPECT_STREQ(StateName(State::WORKING), "working");
    EXPECT_STREQ(StateName(State::ERROR), "error");
}

// === Valid Transitions ===

TEST_F(StateMachineTest, UnconnectedToConnected) {
    EXPECT_TRUE(sm.CanTransitionTo(State::CONNECTED));
    EXPECT_TRUE(sm.TransitionTo(State::CONNECTED));
    EXPECT_EQ(sm.GetState(), State::CONNECTED);
}

TEST_F(StateMachineTest, ConnectedToConfigured) {
    sm.TransitionTo(State::CONNECTED);
    EXPECT_TRUE(sm.CanTransitionTo(State::CONFIGURED));
    EXPECT_TRUE(sm.TransitionTo(State::CONFIGURED));
    EXPECT_EQ(sm.GetState(), State::CONFIGURED);
}

TEST_F(StateMachineTest, ConfiguredToEnabling) {
    sm.TransitionTo(State::CONNECTED);
    sm.TransitionTo(State::CONFIGURED);
    EXPECT_TRUE(sm.CanTransitionTo(State::ENABLING));
    EXPECT_TRUE(sm.TransitionTo(State::ENABLING));
    EXPECT_EQ(sm.GetState(), State::ENABLING);
}

TEST_F(StateMachineTest, EnablingToReady) {
    sm.TransitionTo(State::CONNECTED);
    sm.TransitionTo(State::CONFIGURED);
    sm.TransitionTo(State::ENABLING);
    EXPECT_TRUE(sm.CanTransitionTo(State::READY));
    EXPECT_TRUE(sm.TransitionTo(State::READY));
    EXPECT_EQ(sm.GetState(), State::READY);
}

TEST_F(StateMachineTest, ReadyToWorking) {
    sm.TransitionTo(State::CONNECTED);
    sm.TransitionTo(State::CONFIGURED);
    sm.TransitionTo(State::ENABLING);
    sm.TransitionTo(State::READY);
    EXPECT_TRUE(sm.CanTransitionTo(State::WORKING));
    EXPECT_TRUE(sm.TransitionTo(State::WORKING));
    EXPECT_EQ(sm.GetState(), State::WORKING);
}

TEST_F(StateMachineTest, WorkingToReady) {
    sm.TransitionTo(State::CONNECTED);
    sm.TransitionTo(State::CONFIGURED);
    sm.TransitionTo(State::ENABLING);
    sm.TransitionTo(State::READY);
    sm.TransitionTo(State::WORKING);
    EXPECT_TRUE(sm.CanTransitionTo(State::READY));
    EXPECT_TRUE(sm.TransitionTo(State::READY));
    EXPECT_EQ(sm.GetState(), State::READY);
}

// === Invalid Transitions ===

TEST_F(StateMachineTest, InvalidUnconnectedToConfigured) {
    EXPECT_FALSE(sm.CanTransitionTo(State::CONFIGURED));
    EXPECT_FALSE(sm.TransitionTo(State::CONFIGURED));
    EXPECT_EQ(sm.GetState(), State::UNCONNECTED);  // No change
}

TEST_F(StateMachineTest, InvalidConnectedToReady) {
    sm.TransitionTo(State::CONNECTED);
    EXPECT_FALSE(sm.CanTransitionTo(State::READY));
    EXPECT_FALSE(sm.TransitionTo(State::READY));
}

TEST_F(StateMachineTest, InvalidConfiguredToWorking) {
    sm.TransitionTo(State::CONNECTED);
    sm.TransitionTo(State::CONFIGURED);
    EXPECT_FALSE(sm.CanTransitionTo(State::WORKING));
}

// === Error State ===

TEST_F(StateMachineTest, EnterError) {
    sm.TransitionTo(State::CONNECTED);
    sm.EnterError(ErrorCode::MOTOR_FAULT, "Motor 0 fault");

    EXPECT_EQ(sm.GetState(), State::ERROR);
    EXPECT_EQ(sm.GetErrorCode(), ErrorCode::MOTOR_FAULT);
    EXPECT_STREQ(sm.GetErrorMessage(), "Motor 0 fault");
}

TEST_F(StateMachineTest, ErrorIncrementsEpoch) {
    EXPECT_EQ(sm.GetEpoch(), 0u);
    sm.EnterError(ErrorCode::MOTOR_FAULT, "fault");
    EXPECT_EQ(sm.GetEpoch(), 1u);
}

TEST_F(StateMachineTest, CannotTransitionFromError) {
    sm.EnterError(ErrorCode::INTERNAL_ERROR, "error");
    EXPECT_FALSE(sm.CanTransitionTo(State::CONNECTED));
    EXPECT_FALSE(sm.CanTransitionTo(State::CONFIGURED));
    EXPECT_FALSE(sm.CanTransitionTo(State::READY));
}

TEST_F(StateMachineTest, ResetFromError) {
    sm.TransitionTo(State::CONNECTED);
    sm.EnterError(ErrorCode::MOTOR_FAULT, "fault");
    EXPECT_EQ(sm.GetState(), State::ERROR);

    EXPECT_TRUE(sm.Reset());
    EXPECT_EQ(sm.GetState(), State::CONNECTED);
    EXPECT_EQ(sm.GetErrorCode(), ErrorCode::NONE);
}

TEST_F(StateMachineTest, ResetNotFromNonError) {
    sm.TransitionTo(State::CONNECTED);
    EXPECT_FALSE(sm.Reset());
    EXPECT_EQ(sm.GetState(), State::CONNECTED);
}

// === Mark Connected/Disconnected ===

TEST_F(StateMachineTest, MarkConnected) {
    EXPECT_EQ(sm.GetState(), State::UNCONNECTED);
    sm.MarkConnected();
    EXPECT_EQ(sm.GetState(), State::CONNECTED);
}

TEST_F(StateMachineTest, MarkConnectedNoOpIfAlreadyConnected) {
    sm.TransitionTo(State::CONNECTED);
    sm.TransitionTo(State::CONFIGURED);
    sm.MarkConnected();
    EXPECT_EQ(sm.GetState(), State::CONFIGURED);  // No change
}

TEST_F(StateMachineTest, MarkDisconnected) {
    sm.TransitionTo(State::CONNECTED);
    sm.TransitionTo(State::CONFIGURED);
    sm.MarkDisconnected();
    EXPECT_EQ(sm.GetState(), State::UNCONNECTED);
}

TEST_F(StateMachineTest, MarkDisconnectedNoOpFromError) {
    sm.EnterError(ErrorCode::INTERNAL_ERROR, "error");
    sm.MarkDisconnected();
    EXPECT_EQ(sm.GetState(), State::ERROR);  // No change
}
