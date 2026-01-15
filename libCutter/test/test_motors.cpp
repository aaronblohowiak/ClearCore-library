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
        snprintf(cmd, sizeof(cmd), "configure_stepper motor=%d", motor);
        serial.SendLine(cmd);
        ctrl->Update();
        serial.ClearOutput();

        // Configure endstop for stepper
        serial.SendLine("configure_endstop pin=6");
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
    serial.SendLine("configure_stepper motor=0");
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
    serial.SendLine("configure_stepper motor=0");
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
    serial.SendLine("configure_stepper motor=1");
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

    serial.SendLine("move seq=42 motor=0 steps=1000");
    ctrl->Update();
    serial.ClearOutput();

    // Simulate move completion
    COMPLETE_MOVE(0);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("done"));
    EXPECT_TRUE(serial.HasOutput("motor=0"));
    EXPECT_TRUE(serial.HasOutput("seq=42"));
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
    serial.SendLine("configure_stepper motor=0 soft_limits=1 soft_min=0 soft_max=10000");
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
    serial.SendLine("configure_stepper motor=0 soft_limits=1 soft_min=0 soft_max=10000");
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

// === State Validation ===

TEST_F(MotorTest, MoveRequiresReady) {
    serial.SendLine("configure_stepper motor=0");
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
    serial.SendLine("configure_stepper motor=0");
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
    serial.SendLine("configure_stepper motor=0");
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
