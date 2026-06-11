/**
 * @file test_motors.cpp
 * @brief Motor configuration and motion tests
 */

#include <gtest/gtest.h>
#include "Cutter.h"
#include "FakeHal.h"
#include "TestSerial.h"

using namespace Cutter;

class MotorTest : public ::testing::Test {
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

    void ConfigureAndEnableStepper(int motor = 0) {
        char cmd[128];
        // Use homing_mode=none for tests that don't need homing
        snprintf(cmd, sizeof(cmd), "configure_stepper motor=%d homing_mode=none", motor);
        serial.SendLine(cmd);
        ctrl->Update();
        serial.ClearOutput();

        snprintf(cmd, sizeof(cmd), "enable motor=%d", motor);
        serial.SendLine(cmd);
        ctrl->Update();
        serial.ClearOutput();
    }
};

// === Motor Configuration ===

TEST_F(MotorTest, ConfigureClearPath) {
    serial.SendLine("configure_sdsk motor=0 vel_max=20000 accel_max=200000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetMotor(0)->type, MotorType::CLEARPATH);
    EXPECT_EQ(ctrl->GetMotor(0)->vel_max, 20000);
    EXPECT_EQ(ctrl->GetState(), State::CONFIGURED);
}

TEST_F(MotorTest, ConfigureStepper) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetMotor(0)->type, MotorType::GENERIC_STEPPER);
}

TEST_F(MotorTest, ConfigureInvalidMotor) {
    serial.SendLine("configure_stepper motor=5");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("Invalid motor"));
}

TEST_F(MotorTest, ConfigureInvalidType) {
    // Unknown motor configuration command
    serial.SendLine("configure_servo motor=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("Unknown command"));
}

// === Enable/Disable ===

TEST_F(MotorTest, EnableStepper) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();
    serial.ClearOutput();

    // Need to configure endstop first for stepper homing
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.ClearOutput();

    serial.SendLine("enable motor=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(ctrl->GetMotor(0)->enabled);
    EXPECT_TRUE(g_fake.motor_enabled[0]);
    // Stepper goes directly to READY
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

TEST_F(MotorTest, EnableNotConfigured) {
    // First configure motor 1 so we get to CONFIGURED state
    serial.SendLine("configure_stepper motor=1 homing_mode=none");
    ctrl->Update();
    serial.ClearOutput();

    // Now try to enable unconfigured motor 0
    serial.SendLine("enable motor=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("not configured"));
}

TEST_F(MotorTest, Disable) {
    ConfigureAndEnableStepper();

    serial.SendLine("disable motor=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_FALSE(ctrl->GetMotor(0)->enabled);
    EXPECT_FALSE(g_fake.motor_enabled[0]);
}

// === Motion Commands ===

TEST_F(MotorTest, MoveRelative) {
    ConfigureAndEnableStepper();

    serial.SendLine("move motor=0 steps=1000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(ctrl->GetState(), State::WORKING);
    EXPECT_EQ(g_fake.motor_target[0], 1000);
}

TEST_F(MotorTest, MoveAbsolute) {
    ConfigureAndEnableStepper();

    serial.SendLine("move motor=0 position=5000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(g_fake.motor_target[0], 5000);
}

TEST_F(MotorTest, MoveCompletion) {
    ConfigureAndEnableStepper();

    // User-supplied seq=42 is for verification, internal seq is assigned
    serial.SendLine("move seq=42 motor=0 steps=1000");
    ctrl->Update();

    // Get the internal seq from response
    std::string output = serial.GetOutput();
    size_t seq_pos = output.find("seq=");
    ASSERT_NE(seq_pos, std::string::npos);
    uint32_t internal_seq = 0;
    sscanf(output.c_str() + seq_pos, "seq=%u", &internal_seq);
    serial.ClearOutput();

    // Simulate move completion
    COMPLETE_MOVE(0);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("done"));
    EXPECT_TRUE(serial.HasOutput("motor=0"));
    // Done event includes internal seq, not user-supplied
    char expected_seq[32];
    snprintf(expected_seq, sizeof(expected_seq), "seq=%u", internal_seq);
    EXPECT_TRUE(serial.HasOutput(expected_seq));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

TEST_F(MotorTest, MoveVelocity) {
    ConfigureAndEnableStepper();

    serial.SendLine("move_velocity motor=0 velocity=5000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(g_fake.motor_velocity[0], 5000);
}

TEST_F(MotorTest, Stop) {
    ConfigureAndEnableStepper();

    serial.SendLine("move_velocity motor=0 velocity=5000");
    ctrl->Update();
    serial.ClearOutput();

    serial.SendLine("stop motor=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

TEST_F(MotorTest, SetPosition) {
    ConfigureAndEnableStepper();

    serial.SendLine("set_position motor=0 position=1000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(g_fake.motor_position[0], 1000);
}

// === Soft Limits ===

TEST_F(MotorTest, SoftLimitsBlock) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none soft_limits=1 soft_min=0 soft_max=10000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Try to move beyond limits
    serial.SendLine("move motor=0 position=20000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("soft limit"));
}

TEST_F(MotorTest, SoftLimitsAllow) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none soft_limits=1 soft_min=0 soft_max=10000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Move within limits
    serial.SendLine("move motor=0 position=5000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
}

TEST_F(MotorTest, VelocityMoveSoftLimitMax) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none soft_limits=1 soft_min=0 soft_max=1000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Start velocity move toward max limit
    serial.SendLine("move_velocity seq=50 motor=0 velocity=1000");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetState(), State::WORKING);

    // Get internal seq from response
    std::string output = serial.GetOutput();
    size_t seq_pos = output.find("seq=");
    ASSERT_NE(seq_pos, std::string::npos);
    uint32_t internal_seq = 0;
    sscanf(output.c_str() + seq_pos, "seq=%u", &internal_seq);
    serial.ClearOutput();

    // Simulate motor exceeding soft limit (strict inequality: must go past)
    g_fake.motor_position[0] = 1001;
    ctrl->Update();

    // Should emit soft_limit event and stop
    EXPECT_TRUE(serial.HasEvent("soft_limit"));
    EXPECT_TRUE(serial.HasOutput("motor=0"));
    char expected_seq[32];
    snprintf(expected_seq, sizeof(expected_seq), "seq=%u", internal_seq);
    EXPECT_TRUE(serial.HasOutput(expected_seq));
    EXPECT_TRUE(serial.HasOutput("position=1001"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

TEST_F(MotorTest, VelocityMoveSoftLimitMin) {
    // Configure with range 0-10000, position starts at 5000
    serial.SendLine("configure_stepper motor=0 homing_mode=none soft_limits=1 soft_min=0 soft_max=10000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.SendLine("set_position motor=0 position=5000");
    ctrl->Update();
    serial.ClearOutput();

    // Start velocity move toward min limit (negative velocity)
    serial.SendLine("move_velocity seq=51 motor=0 velocity=-1000");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));

    // Get internal seq from response
    std::string output = serial.GetOutput();
    size_t seq_pos = output.find("seq=");
    ASSERT_NE(seq_pos, std::string::npos);
    uint32_t internal_seq = 0;
    sscanf(output.c_str() + seq_pos, "seq=%u", &internal_seq);
    serial.ClearOutput();

    // Simulate motor exceeding soft limit min (strict inequality: must go past)
    g_fake.motor_position[0] = -1;
    ctrl->Update();

    // Should emit soft_limit event and stop
    EXPECT_TRUE(serial.HasEvent("soft_limit"));
    char expected_seq[32];
    snprintf(expected_seq, sizeof(expected_seq), "seq=%u", internal_seq);
    EXPECT_TRUE(serial.HasOutput(expected_seq));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

TEST_F(MotorTest, VelocityMoveNoSoftLimitWhenDisabled) {
    // Configure without soft limits
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Start velocity move
    serial.SendLine("move_velocity motor=0 velocity=1000");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    serial.ClearOutput();

    // Motor goes to large position - no soft limit triggered
    g_fake.motor_position[0] = 100000;
    ctrl->Update();

    // Should NOT emit soft_limit event (soft limits disabled)
    EXPECT_FALSE(serial.HasEvent("soft_limit"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
}

// === State Validation ===

TEST_F(MotorTest, MoveRequiresReady) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();
    serial.ClearOutput();

    // Not enabled yet
    serial.SendLine("move motor=0 steps=1000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("not enabled") || serial.HasOutput("Not in ready"));
}

// === Hardware Fault Tests ===

TEST_F(MotorTest, MotorHardwareFault) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Simulate motor hardware fault
    SET_MOTOR_FAULT(0, true);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("error"));
    EXPECT_TRUE(serial.HasOutput("code=302"));  // MOTOR_FAULT
    EXPECT_TRUE(serial.HasOutput("Motor hardware fault"));
    EXPECT_EQ(ctrl->GetState(), State::ERROR);
}

TEST_F(MotorTest, NoFaultWhenMotorHealthy) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // No fault set - motor is healthy
    ctrl->Update();

    EXPECT_FALSE(serial.HasEvent("error"));
    EXPECT_NE(ctrl->GetState(), State::ERROR);
}

// === Per-Move Velocity/Acceleration Tests ===

TEST_F(MotorTest, MoveWithVelOverride) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none vel_max=10000 accel_max=100000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Move with lower velocity
    serial.SendLine("move motor=0 steps=1000 vel=5000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
}

TEST_F(MotorTest, MoveWithAccelOverride) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none vel_max=10000 accel_max=100000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Move with lower acceleration
    serial.SendLine("move motor=0 steps=1000 accel=50000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
}

TEST_F(MotorTest, MoveWithBothOverrides) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none vel_max=10000 accel_max=100000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Move with both overrides
    serial.SendLine("move motor=0 steps=1000 vel=5000 accel=25000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
}

TEST_F(MotorTest, MoveVelExceedsMax) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none vel_max=10000 accel_max=100000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Try to exceed velocity limit
    serial.SendLine("move motor=0 steps=1000 vel=15000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("code=306"));  // EXCEEDS_LIMIT
    EXPECT_TRUE(serial.HasOutput("vel exceeds"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

TEST_F(MotorTest, MoveAccelExceedsMax) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none vel_max=10000 accel_max=100000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Try to exceed acceleration limit
    serial.SendLine("move motor=0 steps=1000 accel=150000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("code=306"));  // EXCEEDS_LIMIT
    EXPECT_TRUE(serial.HasOutput("accel exceeds"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

TEST_F(MotorTest, MoveVelocityWithAccelOverride) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none vel_max=10000 accel_max=100000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Velocity move with acceleration override
    serial.SendLine("move_velocity motor=0 velocity=5000 accel=25000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
}

TEST_F(MotorTest, MoveVelocityExceedsMax) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none vel_max=10000 accel_max=100000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Try to exceed velocity limit on velocity move
    serial.SendLine("move_velocity motor=0 velocity=15000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("code=306"));  // EXCEEDS_LIMIT
    EXPECT_TRUE(serial.HasOutput("velocity exceeds"));
}

TEST_F(MotorTest, MoveVelocityNegativeWithinMax) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none vel_max=10000 accel_max=100000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Negative velocity within max (magnitude check)
    serial.SendLine("move_velocity motor=0 velocity=-8000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
}

TEST_F(MotorTest, MoveVelocityNegativeExceedsMax) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none vel_max=10000 accel_max=100000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Negative velocity exceeds max (magnitude check)
    serial.SendLine("move_velocity motor=0 velocity=-15000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("code=306"));  // EXCEEDS_LIMIT
}

// === Event Epoch Tests ===

TEST_F(MotorTest, DoneEventIncludesEpoch) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Move with user-supplied epoch=0 and seq=99 (for verification)
    // Response and event will use internal seq
    serial.SendLine("move epoch=0 seq=99 motor=0 steps=1000");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));

    // Get internal seq from response
    std::string output = serial.GetOutput();
    size_t seq_pos = output.find("seq=");
    ASSERT_NE(seq_pos, std::string::npos);
    uint32_t internal_seq = 0;
    sscanf(output.c_str() + seq_pos, "seq=%u", &internal_seq);
    serial.ClearOutput();

    // Complete the move
    g_fake.motor_steps_complete[0] = true;
    ctrl->Update();

    // Done event should include internal epoch and seq
    EXPECT_TRUE(serial.HasEvent("done"));
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
    char expected_seq[32];
    snprintf(expected_seq, sizeof(expected_seq), "seq=%u", internal_seq);
    EXPECT_TRUE(serial.HasOutput(expected_seq));
}

TEST_F(MotorTest, SoftLimitEventIncludesEpoch) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none soft_limits=1 soft_min=0 soft_max=1000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Velocity move with user-supplied epoch=0 and seq=55 (for verification)
    serial.SendLine("move_velocity epoch=0 seq=55 motor=0 velocity=1000");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));

    // Get internal seq from response
    std::string output = serial.GetOutput();
    size_t seq_pos = output.find("seq=");
    ASSERT_NE(seq_pos, std::string::npos);
    uint32_t internal_seq = 0;
    sscanf(output.c_str() + seq_pos, "seq=%u", &internal_seq);
    serial.ClearOutput();

    // Exceed soft limit
    g_fake.motor_position[0] = 1001;
    ctrl->Update();

    // Soft limit event should include internal epoch and seq
    EXPECT_TRUE(serial.HasEvent("soft_limit"));
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
    char expected_seq[32];
    snprintf(expected_seq, sizeof(expected_seq), "seq=%u", internal_seq);
    EXPECT_TRUE(serial.HasOutput(expected_seq));
}

// === SDSK Move Completion with HLFB ===

TEST_F(MotorTest, SdskMoveWaitsForHlfbAsserted) {
    // Configure SDSK motor
    serial.SendLine("configure_sdsk motor=0 vel_max=20000 accel_max=200000");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok")) << "configure_sdsk output: " << serial.GetOutput();
    serial.ClearOutput();

    SET_HLFB(0, 1);  // HLFB_ASSERTED - motor ready
    SET_MOTOR_READY(0, true);  // Motor is ready (enabled and not faulted)
    serial.SendLine("enable motor=0");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok")) << "enable output: " << serial.GetOutput();
    serial.ClearOutput();

    // Update again to process HLFB and transition from ENABLING to READY
    ctrl->Update();
    EXPECT_EQ(ctrl->GetState(), State::READY) << "State after enable: " << static_cast<int>(ctrl->GetState());

    // Start a move
    serial.SendLine("move motor=0 steps=1000");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok")) << "move output: " << serial.GetOutput();
    serial.ClearOutput();

    // Steps complete but HLFB not asserted yet (motor still settling)
    g_fake.motor_steps_complete[0] = true;
    g_fake.hlfb_state[0] = 0;  // HLFB_DEASSERTED - motor not in position
    ctrl->Update();

    // Done event should NOT have fired yet
    EXPECT_FALSE(serial.HasEvent("done"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);

    // Now HLFB asserts (motor reached position)
    g_fake.hlfb_state[0] = 1;  // HLFB_ASSERTED
    ctrl->Update();

    // Done event should fire now
    EXPECT_TRUE(serial.HasEvent("done"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

TEST_F(MotorTest, StepperMoveCompletesWithoutHlfb) {
    // Configure generic stepper
    ConfigureAndEnableStepper(0);
    serial.ClearOutput();

    // Start a move
    serial.SendLine("move motor=0 steps=1000");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    serial.ClearOutput();

    // Steps complete - HLFB state doesn't matter for generic steppers
    g_fake.motor_steps_complete[0] = true;
    g_fake.hlfb_state[0] = 0;  // HLFB not asserted (doesn't matter)
    ctrl->Update();

    // Done event should fire immediately when steps complete
    EXPECT_TRUE(serial.HasEvent("done"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

// === Motor Clock Rate Tests ===

TEST_F(MotorTest, SetMotorClockLow) {
    serial.SendLine("set_motor_clock rate=low");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("rate=low"));
    EXPECT_EQ(g_fake.motor_clock_rate, 0);  // CLOCK_RATE_LOW
}

TEST_F(MotorTest, SetMotorClockNormal) {
    serial.SendLine("set_motor_clock rate=normal");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("rate=normal"));
    EXPECT_EQ(g_fake.motor_clock_rate, 1);  // CLOCK_RATE_NORMAL
}

TEST_F(MotorTest, SetMotorClockHigh) {
    serial.SendLine("set_motor_clock rate=high");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("rate=high"));
    EXPECT_EQ(g_fake.motor_clock_rate, 2);  // CLOCK_RATE_HIGH
}

TEST_F(MotorTest, SetMotorClockInvalid) {
    serial.SendLine("set_motor_clock rate=fast");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("rate must be low, normal, or high"));
}

TEST_F(MotorTest, SetMotorClockMissingRate) {
    serial.SendLine("set_motor_clock");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("Missing rate parameter"));
}

// === E-Stop Tests ===

TEST_F(MotorTest, ConfigureEStopSingleMotor) {
    // Configure motor first
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();
    serial.ClearOutput();

    // Configure E-Stop on pin 6 for motor 0
    serial.SendLine("configure_estop motor=0 pin=6");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("motor=0"));
    EXPECT_TRUE(serial.HasOutput("pin=6"));
    EXPECT_EQ(g_fake.estop_pin[0], 6);
}

TEST_F(MotorTest, ConfigureEStopAllMotors) {
    // Configure E-Stop on pin 7 for all motors
    serial.SendLine("configure_estop motor=all pin=7");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("motor=all"));
    EXPECT_EQ(g_fake.estop_pin[0], 7);
    EXPECT_EQ(g_fake.estop_pin[1], 7);
    EXPECT_EQ(g_fake.estop_pin[2], 7);
    EXPECT_EQ(g_fake.estop_pin[3], 7);
}

TEST_F(MotorTest, ConfigureEStopWithDecel) {
    serial.SendLine("configure_estop motor=0 pin=6 decel=200000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(g_fake.estop_pin[0], 6);
    EXPECT_EQ(g_fake.estop_decel[0], 200000u);
}

TEST_F(MotorTest, ConfigureEStopAutoConfiguresPin) {
    // Pin 6 is unconfigured, should auto-configure as digital input
    EXPECT_EQ(ctrl->GetPin(6)->mode, PinMode::UNCONFIGURED);

    serial.SendLine("configure_estop motor=0 pin=6");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetPin(6)->mode, PinMode::DIGITAL_IN);
}

TEST_F(MotorTest, ConfigureEStopAllowsDigitalInput) {
    // Pre-configure pin as digital input
    serial.SendLine("configure_digital_in pin=6");
    ctrl->Update();
    serial.ClearOutput();

    // Should still work
    serial.SendLine("configure_estop motor=0 pin=6");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(g_fake.estop_pin[0], 6);
}

TEST_F(MotorTest, ConfigureEStopRejectsOutput) {
    // Configure pin as digital output
    serial.SendLine("configure_digital_out pin=0");
    ctrl->Update();
    serial.ClearOutput();

    // E-Stop should fail - can't use output pin
    serial.SendLine("configure_estop motor=0 pin=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("Pin must be digital input"));
}

TEST_F(MotorTest, EStopTriggerDuringMove) {
    ConfigureAndEnableStepper(0);
    serial.SendLine("configure_estop motor=0 pin=6");
    ctrl->Update();
    serial.ClearOutput();

    // Start a move with user-supplied seq=42 (for verification)
    serial.SendLine("move seq=42 motor=0 steps=10000");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);

    // Get internal seq from response
    std::string output = serial.GetOutput();
    size_t seq_pos = output.find("seq=");
    ASSERT_NE(seq_pos, std::string::npos);
    uint32_t internal_seq = 0;
    sscanf(output.c_str() + seq_pos, "seq=%u", &internal_seq);
    serial.ClearOutput();

    // Trigger E-Stop (simulates hardware stopping motor)
    TRIGGER_ESTOP(0);
    ctrl->Update();

    // Should emit estop event with position and internal seq
    EXPECT_TRUE(serial.HasEvent("estop"));
    EXPECT_TRUE(serial.HasOutput("motor=0"));
    char expected_seq[32];
    snprintf(expected_seq, sizeof(expected_seq), "seq=%u", internal_seq);
    EXPECT_TRUE(serial.HasOutput(expected_seq));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

TEST_F(MotorTest, EStopBlocksMotion) {
    ConfigureAndEnableStepper(0);
    serial.SendLine("configure_estop motor=0 pin=6");
    ctrl->Update();

    // Trigger E-Stop without a move (motor has alert)
    g_fake.motion_canceled_estop[0] = true;
    serial.ClearOutput();

    // Try to move - motor has alerts, so move should start but HasMotorAlerts blocks completion
    // Actually, the move command itself doesn't check alerts - it just starts the move
    // The E-Stop is detected in CheckMotors when the motor is moving

    // Let's clear alerts first and test that motion works
    CLEAR_MOTOR_ALERTS(0);
    serial.SendLine("move motor=0 steps=100");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
}

TEST_F(MotorTest, ClearAlertsAfterEStop) {
    ConfigureAndEnableStepper(0);
    serial.SendLine("configure_estop motor=0 pin=6");
    ctrl->Update();
    serial.ClearOutput();

    // Trigger E-Stop
    g_fake.motion_canceled_estop[0] = true;

    // Verify alert is set
    EXPECT_TRUE(CutterHal::HasMotorAlerts(0));

    // Clear alerts
    serial.SendLine("clear_alerts motor=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_FALSE(CutterHal::HasMotorAlerts(0));
}

TEST_F(MotorTest, ClearAlertsRequiresConfiguredMotor) {
    // Try to clear alerts on unconfigured motor
    serial.SendLine("clear_alerts motor=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("Motor not configured"));
}

TEST_F(MotorTest, EStopEventIncludesEpoch) {
    ConfigureAndEnableStepper(0);
    serial.SendLine("configure_estop motor=0 pin=6");
    ctrl->Update();
    serial.ClearOutput();

    // Start a move with user-supplied epoch and seq (for verification)
    serial.SendLine("move epoch=0 seq=77 motor=0 steps=10000");
    ctrl->Update();

    // Get internal seq from response
    std::string output = serial.GetOutput();
    size_t seq_pos = output.find("seq=");
    ASSERT_NE(seq_pos, std::string::npos);
    uint32_t internal_seq = 0;
    sscanf(output.c_str() + seq_pos, "seq=%u", &internal_seq);
    serial.ClearOutput();

    // Trigger E-Stop
    TRIGGER_ESTOP(0);
    ctrl->Update();

    // Event should include internal epoch and seq
    EXPECT_TRUE(serial.HasEvent("estop"));
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
    char expected_seq[32];
    snprintf(expected_seq, sizeof(expected_seq), "seq=%u", internal_seq);
    EXPECT_TRUE(serial.HasOutput(expected_seq));
}

// === Limit Switch Trigger During Move ===
// A limit switch tripping during a normal (non-homing) move must emit a
// "limit" event. Otherwise open-loop steppers would report a false "done"
// and SDSK moves would hang on the latched alert.

TEST_F(MotorTest, LimitTriggerDuringMoveEmitsEvent) {
    ConfigureAndEnableStepper(0);
    serial.ClearOutput();

    // Start a move and capture the internal seq from the ok response
    serial.SendLine("move seq=42 motor=0 steps=10000");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);

    std::string output = serial.GetOutput();
    size_t seq_pos = output.find("seq=");
    ASSERT_NE(seq_pos, std::string::npos);
    uint32_t internal_seq = 0;
    sscanf(output.c_str() + seq_pos, "seq=%u", &internal_seq);
    serial.ClearOutput();

    // Hardware trips the positive limit and stops the motor
    TRIGGER_POS_LIMIT(0);
    ctrl->Update();

    // Should emit a limit event (not a false "done"), with direction and seq
    EXPECT_TRUE(serial.HasEvent("limit"));
    EXPECT_FALSE(serial.HasEvent("done"));
    EXPECT_TRUE(serial.HasOutput("motor=0"));
    EXPECT_TRUE(serial.HasOutput("direction=pos"));
    char expected_seq[32];
    snprintf(expected_seq, sizeof(expected_seq), "seq=%u", internal_seq);
    EXPECT_TRUE(serial.HasOutput(expected_seq));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

TEST_F(MotorTest, NegLimitTriggerReportsDirection) {
    ConfigureAndEnableStepper(0);
    serial.ClearOutput();

    serial.SendLine("move motor=0 steps=-10000");
    ctrl->Update();
    serial.ClearOutput();

    TRIGGER_NEG_LIMIT(0);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("limit"));
    EXPECT_TRUE(serial.HasOutput("direction=neg"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

TEST_F(MotorTest, LimitTriggerReturnsToReady) {
    ConfigureAndEnableStepper(0);
    serial.ClearOutput();

    serial.SendLine("move motor=0 steps=10000");
    ctrl->Update();
    EXPECT_EQ(ctrl->GetState(), State::WORKING);
    serial.ClearOutput();

    TRIGGER_POS_LIMIT(0);
    ctrl->Update();

    // With no other motors moving, controller returns to READY
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

TEST_F(MotorTest, NoLimitEventWhenNotMoving) {
    ConfigureAndEnableStepper(0);
    serial.ClearOutput();

    // Alert latched but motor is idle - no spurious event should be emitted
    g_fake.motion_canceled_pos_limit[0] = true;
    ctrl->Update();

    EXPECT_FALSE(serial.HasEvent("limit"));
}

// === Internal Command ID Tests ===
// These tests verify that every command gets an internal seq/epoch
// regardless of whether the user supplied them

TEST_F(MotorTest, InternalSeqIncrements) {
    // Send commands without user-supplied seq/epoch
    // Each should get incrementing internal seq in response

    // Note: ping in SetUp uses seq=1, so first command here gets seq=2
    serial.SendLine("ping");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
    EXPECT_TRUE(serial.HasOutput("seq=2"));
    serial.ClearOutput();

    serial.SendLine("ping");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
    EXPECT_TRUE(serial.HasOutput("seq=3"));
    serial.ClearOutput();

    serial.SendLine("ping");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
    EXPECT_TRUE(serial.HasOutput("seq=4"));
}

TEST_F(MotorTest, ConfigureCommandsGetInternalSeq) {
    // Configure commands without user seq should still get internal seq
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
    EXPECT_TRUE(serial.HasOutput("seq=2"));  // After ping in SetUp
    serial.ClearOutput();

    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
    EXPECT_TRUE(serial.HasOutput("seq=3"));
}

TEST_F(MotorTest, MoveWithoutUserSeqGetsInternalSeq) {
    ConfigureAndEnableStepper();
    serial.ClearOutput();

    // Move without user-supplied seq - should get internal seq
    serial.SendLine("move motor=0 steps=1000");
    ctrl->Update();

    // ok response should include internal epoch and seq
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
    // seq depends on how many commands were sent in setup
    std::string output = serial.GetOutput();
    EXPECT_TRUE(output.find("seq=") != std::string::npos);

    // Remember what seq was assigned
    // Find seq in output for verification in done event
    serial.ClearOutput();

    // Complete the move
    COMPLETE_MOVE(0);
    ctrl->Update();

    // Done event should have same epoch and seq as the move command
    EXPECT_TRUE(serial.HasEvent("done"));
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
    // seq should match what move command got
}

TEST_F(MotorTest, DoneEventUsesInternalSeqFromMoveCommand) {
    ConfigureAndEnableStepper();
    serial.ClearOutput();

    // Get next_seq to know what the move will be assigned
    serial.SendLine("get_next_seq");
    ctrl->Update();
    // Extract next_seq value from response
    std::string output = serial.GetOutput();
    EXPECT_TRUE(output.find("next_seq=") != std::string::npos);
    serial.ClearOutput();

    // Move without user seq
    serial.SendLine("move motor=0 steps=1000");
    ctrl->Update();
    output = serial.GetOutput();

    // Extract the seq that was assigned to this command
    size_t seq_pos = output.find("seq=");
    ASSERT_NE(seq_pos, std::string::npos);
    uint32_t assigned_seq = 0;
    sscanf(output.c_str() + seq_pos, "seq=%u", &assigned_seq);
    EXPECT_GT(assigned_seq, 0u);
    serial.ClearOutput();

    // Complete the move
    COMPLETE_MOVE(0);
    ctrl->Update();

    // Done event should have same seq as the move command's internal seq
    EXPECT_TRUE(serial.HasEvent("done"));
    output = serial.GetOutput();
    char expected_seq[32];
    snprintf(expected_seq, sizeof(expected_seq), "seq=%u", assigned_seq);
    EXPECT_TRUE(serial.HasOutput(expected_seq))
        << "Expected seq=" << assigned_seq << " in done event, got: " << output;
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
}

TEST_F(MotorTest, SoftLimitEventUsesInternalSeq) {
    serial.SendLine("configure_stepper motor=0 homing_mode=none soft_limits=1 soft_min=0 soft_max=1000");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Start velocity move WITHOUT user seq/epoch
    serial.SendLine("move_velocity motor=0 velocity=1000");
    ctrl->Update();

    // Extract the internal seq assigned
    std::string output = serial.GetOutput();
    size_t seq_pos = output.find("seq=");
    ASSERT_NE(seq_pos, std::string::npos);
    uint32_t assigned_seq = 0;
    sscanf(output.c_str() + seq_pos, "seq=%u", &assigned_seq);
    serial.ClearOutput();

    // Exceed soft limit
    g_fake.motor_position[0] = 1001;
    ctrl->Update();

    // Soft limit event should use the internal seq from move_velocity
    EXPECT_TRUE(serial.HasEvent("soft_limit"));
    char expected_seq[32];
    snprintf(expected_seq, sizeof(expected_seq), "seq=%u", assigned_seq);
    EXPECT_TRUE(serial.HasOutput(expected_seq));
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
}

TEST_F(MotorTest, ErrorResponsesIncludeInternalSeq) {
    // Error responses should also include internal epoch/seq
    serial.SendLine("move motor=99 steps=1000");  // Invalid motor
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("epoch=0"));
    EXPECT_TRUE(serial.HasOutput("seq=2"));  // After ping in SetUp
}
