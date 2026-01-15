/**
 * @file test_homing.cpp
 * @brief Homing sequence tests
 */

#include <gtest/gtest.h>
#include "Cutter.h"
#include "FakeHal.h"
#include "TestSerial.h"

using namespace Cutter;

class HomingTest : public ::testing::Test {
protected:
    TestSerial serial;
    Controller* ctrl;

    void SetUp() override {
        RESET_HAL();
        ctrl = new Controller(&serial);
        // Connect and configure stepper
        serial.SendLine("ping");
        ctrl->Update();
        serial.SendLine("configure_motor motor=0 type=stepper end_stop_pin=6 homing_seek_velocity=5000 homing_latch_velocity=500 homing_backoff=200");
        ctrl->Update();
        serial.SendLine("configure_endstop pin=6 active_high=1");
        ctrl->Update();
        serial.SendLine("enable motor=0");
        ctrl->Update();
        serial.ClearOutput();
    }

    void TearDown() override {
        delete ctrl;
    }
};

// === Homing Command ===

TEST_F(HomingTest, HomeCommand) {
    serial.SendLine("home motor=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::SEEKING);
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(ctrl->GetState(), State::WORKING);
}

TEST_F(HomingTest, HomeOnlyStepper) {
    // Configure as ClearPath
    serial.SendLine("configure_motor motor=1 type=clearpath");
    ctrl->Update();
    serial.ClearOutput();

    serial.SendLine("home motor=1");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("Only generic steppers"));
}

TEST_F(HomingTest, HomeRequiresEnabled) {
    serial.SendLine("disable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    serial.SendLine("home motor=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("not enabled"));
}

// === Homing Sequence ===

TEST_F(HomingTest, HomingSequenceComplete) {
    serial.SendLine("home seq=10 motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Phase 1: Seeking - motor moves toward endstop
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::SEEKING);

    // Trigger endstop
    SET_PIN(6, true);
    ctrl->Update();

    // Phase 2: Backing off
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::BACKING_OFF);
    serial.ClearOutput();

    // Clear endstop, complete backoff move
    SET_PIN(6, false);
    COMPLETE_MOVE(0);
    ctrl->Update();

    // Phase 3: Latching - slow approach
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::LATCHING);
    serial.ClearOutput();

    // Trigger endstop again for final latch
    SET_PIN(6, true);
    ctrl->Update();

    // Should be complete
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::COMPLETE);
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(g_fake.motor_position[0], 0);  // Position set to zero

    EXPECT_TRUE(serial.HasEvent("homed"));
    EXPECT_TRUE(serial.HasOutput("motor=0"));
    EXPECT_TRUE(serial.HasOutput("seq=10"));
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

// === Homing with Active Low Endstop ===

TEST_F(HomingTest, HomingActiveLow) {
    // Reconfigure with active_low
    serial.SendLine("configure_motor motor=0 type=stepper end_stop_pin=6 end_stop_active_high=0");
    ctrl->Update();
    serial.SendLine("configure_endstop pin=6 active_high=0");
    ctrl->Update();
    // Re-enable motor after reconfiguring (memset resets enabled flag)
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Endstop starts high (not triggered since active_low)
    SET_PIN(6, true);
    serial.SendLine("home motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Should be seeking since endstop not triggered
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::SEEKING);

    // Trigger endstop (goes low)
    SET_PIN(6, false);
    ctrl->Update();

    // Should advance to backing off
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::BACKING_OFF);
}
