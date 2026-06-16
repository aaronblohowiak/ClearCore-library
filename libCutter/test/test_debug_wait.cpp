/**
 * @file test_debug_wait.cpp
 * @brief Tests for the DEBUG-ONLY debug_wait command. It puts the controller
 *        into a wait mode (resolved at the top of Update()) that suspends serial
 *        input until the target motor's move completes or a timeout elapses, so a
 *        script pasted into a dumb terminal steps through one move at a time.
 */

#include <gtest/gtest.h>
#include "Cutter.h"
#include "CutterHal.h"
#include "FakeHal.h"
#include "TestSerial.h"

using namespace Cutter;

class DebugWaitTest : public ::testing::Test {
protected:
    TestSerial serial;
    Controller* ctrl;

    void SetUp() override {
        RESET_HAL();
        ctrl = new Controller(&serial);
        serial.SendLine("ping");
        ctrl->Update();
        serial.SendLine("configure_stepper motor=0 homing_mode=none vel_max=10000 accel_max=50000");
        ctrl->Update();
        serial.SendLine("enable motor=0");
        ctrl->Update();
        serial.ClearOutput();
    }

    void TearDown() override {
        delete ctrl;
    }
};

// === Resolves (deferred ok) when the awaited move completes ===

TEST_F(DebugWaitTest, ResolvesOnCompletion) {
    serial.SendLine("move motor=0 steps=1000");
    ctrl->Update();
    ASSERT_TRUE(ctrl->GetMotor(0)->moving);
    serial.ClearOutput();

    // Enter wait mode: no response yet, input is now gated.
    serial.SendLine("debug_wait motor=0");
    ctrl->Update();
    EXPECT_FALSE(serial.HasOutput("ok"));        // ok is deferred until completion
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);

    // Move completes in hardware. First Update emits the move's done + clears
    // moving; the next Update resolves the wait with the deferred ok.
    COMPLETE_MOVE(0);
    ctrl->Update();
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("done"));
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("position=1000"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

// === Subsequent pasted lines are gated until the wait resolves ===

TEST_F(DebugWaitTest, InputGatedWhileWaiting) {
    // One paste: move, then wait, then a second move.
    serial.SendLine("move motor=0 steps=1000");
    serial.SendLine("debug_wait motor=0");
    serial.SendLine("move motor=0 steps=-1000");
    ctrl->Update();

    // Only the first move ran; the second is still buffered (not dispatched).
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(g_fake.motor_target[0], 1000);  // first move's target, not -1000

    // Complete the first move; the wait resolves and the second move then starts.
    COMPLETE_MOVE(0);
    ctrl->Update();   // emits done, clears moving
    ctrl->Update();   // resolves wait (ok), then ProcessInput dispatches move #2

    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(g_fake.motor_target[0], 0);     // 1000 + (-1000)
    EXPECT_EQ(ctrl->GetState(), State::WORKING);
}

// === No-op when the target isn't moving ===

TEST_F(DebugWaitTest, NoOpWhenIdle) {
    serial.SendLine("debug_wait motor=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("position="));
}

TEST_F(DebugWaitTest, NoArgWaitsForAllIdle) {
    serial.SendLine("debug_wait");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

// === Hardware E-stop still aborts the move and ends the wait ===

TEST_F(DebugWaitTest, HardwareEstopEndsWait) {
    serial.SendLine("move motor=0 steps=1000");
    ctrl->Update();
    serial.SendLine("debug_wait motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Physical E-stop input asserts (polled by CheckMotors, not a serial command).
    TRIGGER_ESTOP(0);
    ctrl->Update();   // CheckMotors emits the estop alert, clears moving
    ctrl->Update();   // wait resolves

    EXPECT_TRUE(serial.HasEvent("alert"));
    EXPECT_TRUE(serial.HasOutput("cause=estop"));
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

// === Timeout bounds the wait so it can't wedge the controller ===

TEST_F(DebugWaitTest, Timeout) {
    serial.SendLine("move motor=0 steps=100000");  // never completed in the fake
    ctrl->Update();
    serial.SendLine("debug_wait motor=0 timeout_ms=1000");
    ctrl->Update();
    serial.ClearOutput();

    ADVANCE_TIME(2000);
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("code=308"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);  // motor left as-is; wait just gave up
}
