/**
 * @file test_errors.cpp
 * @brief Error handling and recovery tests
 */

#include <gtest/gtest.h>
#include "Cutter.h"
#include "FakeHal.h"
#include "TestSerial.h"

using namespace Cutter;

class ErrorTest : public ::testing::Test {
protected:
    TestSerial serial;
    Controller* ctrl;

    void SetUp() override {
        RESET_HAL();
        ctrl = new Controller(&serial);
        // Connect
        serial.SendLine("ping");
        ctrl->Update();
        serial.ClearOutput();
    }

    void TearDown() override {
        delete ctrl;
    }
};

// === Unknown Command ===

TEST_F(ErrorTest, UnknownCommand) {
    serial.SendLine("foobar");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("Unknown command"));
    EXPECT_TRUE(serial.HasOutput("command=foobar"));
}

TEST_F(ErrorTest, UnknownCommandWithSeq) {
    serial.SendLine("foobar seq=42");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("seq=42"));
}

// === Epoch Mismatch ===

TEST_F(ErrorTest, EpochMismatch) {
    serial.SendLine("ping epoch=99");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("Epoch mismatch"));
    EXPECT_TRUE(serial.HasOutput("expected=0"));
    EXPECT_TRUE(serial.HasOutput("got=99"));
}

TEST_F(ErrorTest, EpochMatchAfterError) {
    // Cause an error to increment epoch
    ctrl->GetStateMachine().EnterError(ErrorCode::INTERNAL_ERROR, "test");
    EXPECT_EQ(ctrl->GetEpoch(), 1u);

    // Now commands with epoch=0 fail
    serial.SendLine("ping epoch=0");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("Epoch mismatch"));
    serial.ClearOutput();

    // Commands with epoch=1 succeed
    serial.SendLine("ping epoch=1");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
}

// === Missing Parameters ===

TEST_F(ErrorTest, MissingRequiredParam) {
    serial.SendLine("configure_digital_in");  // Missing pin
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("Invalid pin"));
}

TEST_F(ErrorTest, MissingMotorParam) {
    serial.SendLine("configure_stepper");  // Missing motor parameter
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("Invalid motor"));
}

// === Invalid State ===

TEST_F(ErrorTest, ResetNotInError) {
    serial.SendLine("reset");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("Not in error state"));
}

TEST_F(ErrorTest, EnableWithoutConfigure) {
    serial.SendLine("enable motor=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
}

// === Error Recovery ===

TEST_F(ErrorTest, ResetFromError) {
    // Enter error state
    ctrl->GetStateMachine().EnterError(ErrorCode::MOTOR_FAULT, "Motor fault");
    EXPECT_EQ(ctrl->GetState(), State::ERROR);

    // Reset
    serial.SendLine("reset");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("epoch=1"));
    EXPECT_EQ(ctrl->GetState(), State::CONNECTED);
}

// === Status Command ===

TEST_F(ErrorTest, StatusShowsError) {
    ctrl->GetStateMachine().EnterError(ErrorCode::MOTOR_FAULT, "Test fault");

    serial.SendLine("status");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("state=error"));
    EXPECT_TRUE(serial.HasOutput("error_code=302"));
    EXPECT_TRUE(serial.HasOutput("Test fault"));
}

TEST_F(ErrorTest, StatusShowsEpoch) {
    serial.SendLine("status");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
}

// === Version Command ===

TEST_F(ErrorTest, VersionCommand) {
    serial.SendLine("version");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("version="));
    EXPECT_TRUE(serial.HasOutput("protocol="));
}

// === Pin Not Configured ===

TEST_F(ErrorTest, ReadUnconfiguredPin) {
    serial.SendLine("read_pin pin=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("not configured"));
}

TEST_F(ErrorTest, WriteToUnconfiguredPin) {
    serial.SendLine("write_pin pin=0 value=1");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("not configured"));
}

// === Motor Not Ready ===

TEST_F(ErrorTest, MoveWithoutEnable) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();
    serial.ClearOutput();

    serial.SendLine("move motor=0 steps=1000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
}

// === Emergency Stop ===

TEST_F(ErrorTest, EmergencyStop) {
    // Configure and enable a motor
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Start a move
    serial.SendLine("move motor=0 steps=10000");
    ctrl->Update();
    EXPECT_EQ(ctrl->GetState(), State::WORKING);
    serial.ClearOutput();

    // Emergency stop
    serial.SendLine("emergency_stop");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetState(), State::ERROR);
    EXPECT_EQ(ctrl->GetStateMachine().GetErrorCode(), ErrorCode::EMERGENCY_STOP);
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

TEST_F(ErrorTest, EmergencyStopFromReady) {
    serial.SendLine("emergency_stop");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetState(), State::ERROR);
}

// === Stale Sequence Tests ===

TEST_F(ErrorTest, StaleSeqRejected) {
    // First command with seq=100
    serial.SendLine("ping seq=100");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    serial.ClearOutput();

    // Second command with lower seq=50 - should be rejected
    serial.SendLine("ping seq=50");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("code=105"));  // STALE_SEQ
    EXPECT_TRUE(serial.HasOutput("seq=50"));
    EXPECT_TRUE(serial.HasOutput("max_seen=100"));
    // Should NOT enter error state - just reject this command
    EXPECT_NE(ctrl->GetState(), State::ERROR);
}

TEST_F(ErrorTest, DuplicateSeqRejected) {
    // First command with seq=100
    serial.SendLine("ping seq=100");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    serial.ClearOutput();

    // Second command with same seq=100 - should be rejected
    serial.SendLine("ping seq=100");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("code=105"));  // STALE_SEQ
    EXPECT_NE(ctrl->GetState(), State::ERROR);
}

TEST_F(ErrorTest, IncreasingSeqAccepted) {
    serial.SendLine("ping seq=1");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    serial.ClearOutput();

    serial.SendLine("ping seq=2");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    serial.ClearOutput();

    serial.SendLine("ping seq=100");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
}

TEST_F(ErrorTest, SeqWrapAroundAccepted) {
    // Send command with seq near UINT32_MAX
    serial.SendLine("ping seq=4294967290");  // UINT32_MAX - 5
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    serial.ClearOutput();

    // Send command with seq that wrapped (low number after high)
    // Using serial number arithmetic, 5 is "ahead" of 4294967290
    serial.SendLine("ping seq=5");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
}

// === Input Buffer Overflow ===

TEST_F(ErrorTest, InputOverflowReportsError) {
    // Create a command longer than MAX_COMMAND_LENGTH (512)
    std::string long_cmd = "ping param=";
    long_cmd.append(600, 'x');  // Add 600 'x' characters

    serial.SendLine(long_cmd.c_str());
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("code=106"));  // INPUT_OVERFLOW
    EXPECT_TRUE(serial.HasOutput("Command too long"));
    EXPECT_TRUE(serial.HasOutput("max_length=511"));
}

TEST_F(ErrorTest, InputOverflowRecovery) {
    // Send overflow command
    std::string long_cmd = "ping param=";
    long_cmd.append(600, 'x');
    serial.SendLine(long_cmd.c_str());
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("error"));
    serial.ClearOutput();

    // Next normal command should work fine
    serial.SendLine("ping");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
}

TEST_F(ErrorTest, MaxValidCommandFits) {
    // Test that the worst-case valid command fits in the buffer
    // configure_stepper with all parameters at max values
    // Note: We omit epoch since we're just testing buffer size, not protocol
    std::string worst_case =
        "configure_stepper motor=0 vel_max=2147483647 accel_max=2147483647 "
        "enable_priority=255 homing_mode=none homing_direction=-1 "
        "homing_seek_velocity=2147483647 homing_latch_velocity=2147483647 "
        "homing_backoff=2147483647 limit_neg_pin=6 limit_pos_pin=7 "
        "soft_limits=true soft_min=-2147483648 soft_max=2147483647";

    // Verify it's under 512 bytes
    EXPECT_LT(worst_case.length(), MAX_COMMAND_LENGTH);

    serial.SendLine(worst_case.c_str());
    ctrl->Update();

    // Should get 'ok', not overflow error
    SCOPED_TRACE("Output was: " + serial.GetOutput());
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_FALSE(serial.HasOutput("code=106"));
}
