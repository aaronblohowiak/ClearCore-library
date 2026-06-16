/**
 * @file test_integration.cpp
 * @brief Full command/response cycle integration tests
 */

#include <gtest/gtest.h>
#include "Cutter.h"
#include "FakeHal.h"
#include "TestSerial.h"

using namespace Cutter;

class IntegrationTest : public ::testing::Test {
protected:
    TestSerial serial;
    Controller* ctrl;

    void SetUp() override {
        RESET_HAL();
        ctrl = new Controller(&serial);
    }

    void TearDown() override {
        delete ctrl;
    }
};

// === Connection banner ===

TEST_F(IntegrationTest, ConnectionBannerEmittedOnPortOpenWithoutInput) {
    // Host has not opened the port yet: no banner, still unconnected.
    serial.SetPortOpen(false);
    ctrl->Update();
    EXPECT_FALSE(serial.HasOutput("cutter ready"));
    EXPECT_EQ(ctrl->GetState(), State::UNCONNECTED);

    // Host opens the port (USB DTR). The banner is emitted on the next Update
    // with NO command typed - it is connection-driven, not input-driven.
    serial.SetPortOpen(true);
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput(
        "debug message=\"cutter ready\" protocol=1 version=1.0.0 device_id=12648430"));
    EXPECT_EQ(ctrl->GetState(), State::CONNECTED);

    // Banner is emitted once per connection, not on every Update.
    serial.ClearOutput();
    ctrl->Update();
    EXPECT_FALSE(serial.HasOutput("cutter ready"));

    // Closing and reopening the port re-arms the banner for the new host.
    serial.SetPortOpen(false);
    ctrl->Update();
    serial.SetPortOpen(true);
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("cutter ready"));
}

TEST_F(IntegrationTest, ConnectionBannerPrecedesFirstResponse) {
    serial.SendLine("ping");
    ctrl->Update();

    // The banner precedes the first command's response.
    auto lines = serial.GetOutputLines();
    ASSERT_GE(lines.size(), 2u);
    // Starts-with (not exact): every line now also carries a trailing t_ms stamp.
    EXPECT_EQ(lines[0].rfind("debug message=\"cutter ready\" protocol=1 version=1.0.0 device_id=12648430", 0), 0u);
    EXPECT_EQ(lines[1].rfind("ok", 0), 0u);  // command response follows
}

TEST_F(IntegrationTest, ResponsesCarryMonotonicTimestamp) {
    // Every emitted line carries the controller's uptime as t_ms.
    serial.SendLine("ping");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("t_ms=0"));  // banner + ok both stamped at boot
    serial.ClearOutput();

    // The stamp reflects the controller clock at send time.
    ADVANCE_TIME(500);
    serial.SendLine("ping");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("t_ms=500"));
}

TEST_F(IntegrationTest, ConnectionBannerFallsBackToFirstByteWithoutDtr) {
    // A transport that never reports the port as open (e.g. a terminal that
    // does not assert DTR) still gets a banner, triggered by the first byte.
    serial.SetPortOpen(false);
    serial.SendLine("ping");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("cutter ready"));
    EXPECT_EQ(ctrl->GetState(), State::CONNECTED);
}

// === Full Connect -> Configure -> Enable -> Move -> Complete Cycle ===

TEST_F(IntegrationTest, FullMotionCycle) {
    // Connect
    serial.SendLine("ping");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetState(), State::CONNECTED);
    serial.ClearOutput();

    // Configure motor (no homing for this test)
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetState(), State::CONFIGURED);
    serial.ClearOutput();

    // Enable motor
    serial.SendLine("enable motor=0");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetState(), State::READY);
    serial.ClearOutput();

    // Start move (user seq=1 is for verification, internal seq=4 is used)
    serial.SendLine("move seq=1 motor=0 steps=1000");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    // Internal seq is assigned (command 4: ping=1, configure=2, enable=3, move=4)
    EXPECT_TRUE(serial.HasOutput("seq=4"));
    EXPECT_EQ(ctrl->GetState(), State::WORKING);
    serial.ClearOutput();

    // Complete move
    COMPLETE_MOVE(0);
    ctrl->Update();
    EXPECT_TRUE(serial.HasEvent("done"));
    // Done event uses internal seq from move command
    EXPECT_TRUE(serial.HasOutput("seq=4"));
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

// === Multi-Motor Motion ===

TEST_F(IntegrationTest, MultiMotorMotion) {
    // Setup
    serial.SendLine("ping");
    ctrl->Update();
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();
    serial.SendLine("configure_stepper motor=1 homing_mode=none");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.SendLine("enable motor=1");
    ctrl->Update();
    serial.ClearOutput();

    // Start moves on both motors
    serial.SendLine("move seq=1 motor=0 steps=1000");
    ctrl->Update();
    serial.SendLine("move seq=2 motor=1 steps=2000");
    ctrl->Update();
    serial.ClearOutput();

    EXPECT_EQ(ctrl->GetState(), State::WORKING);

    // Complete first motor
    COMPLETE_MOVE(0);
    ctrl->Update();
    EXPECT_TRUE(serial.HasEvent("done"));
    EXPECT_TRUE(serial.HasOutput("motor=0"));
    // Still in WORKING since motor 1 is moving
    EXPECT_EQ(ctrl->GetState(), State::WORKING);
    serial.ClearOutput();

    // Complete second motor
    COMPLETE_MOVE(1);
    ctrl->Update();
    EXPECT_TRUE(serial.HasEvent("done"));
    EXPECT_TRUE(serial.HasOutput("motor=1"));
    // Now back to READY
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

// === Pin Input Event During Motion ===

TEST_F(IntegrationTest, PinEventDuringMotion) {
    // Setup
    serial.SendLine("ping");
    ctrl->Update();
    serial.SendLine("configure_digital_in pin=7 report_changes=1");
    ctrl->Update();
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.SendLine("move motor=0 steps=1000");
    ctrl->Update();
    serial.ClearOutput();

    // Trigger pin change during motion
    SET_PIN(7, true);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("input"));
    EXPECT_TRUE(serial.HasOutput("pin=7"));
    // Still in WORKING
    EXPECT_EQ(ctrl->GetState(), State::WORKING);
}

// === Error Recovery Cycle ===

TEST_F(IntegrationTest, ErrorRecoveryCycle) {
    // Setup
    serial.SendLine("ping");
    ctrl->Update();
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();
    serial.ClearOutput();

    // Simulate error
    ctrl->GetStateMachine().EnterError(ErrorCode::MOTOR_FAULT, "fault");
    EXPECT_EQ(ctrl->GetState(), State::ERROR);
    EXPECT_EQ(ctrl->GetEpoch(), 1u);

    // Check status shows error
    serial.SendLine("status");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("state=error"));
    serial.ClearOutput();

    // Reset
    serial.SendLine("reset");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetState(), State::CONNECTED);

    // Motor config was cleared, need to reconfigure
    EXPECT_EQ(ctrl->GetMotor(0)->type, MotorType::UNCONFIGURED);
}

// === Internal Sequence Number ===
// Every command gets an internal seq, not the user-supplied one
// User-supplied seq is only for verification (staleness check)

TEST_F(IntegrationTest, SeqRoundTrip) {
    serial.SendLine("ping");
    ctrl->Update();
    serial.ClearOutput();

    // User-supplied seq=12345 is for verification only
    // Response includes internal seq (which increments from 1)
    serial.SendLine("ping seq=12345");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
    // Response includes internal seq=2 (ping in setup was seq=1)
    EXPECT_TRUE(serial.HasOutput("seq=2"));
}

// === Multiple Commands in Sequence ===

TEST_F(IntegrationTest, MultipleCommandsSequence) {
    serial.SendLine("ping");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    serial.ClearOutput();

    serial.SendLine("status");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("state=connected"));
    serial.ClearOutput();

    serial.SendLine("version");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("version="));
}
