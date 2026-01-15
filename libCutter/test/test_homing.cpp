/**
 * @file test_homing.cpp
 * @brief Homing sequence tests
 */

#include <gtest/gtest.h>
#include "Cutter.h"
#include "CutterHal.h"
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
        // Connect and configure stepper with limit switch on pin 6
        serial.SendLine("ping");
        ctrl->Update();
        serial.SendLine("configure_stepper motor=0 limit_neg_pin=6 homing_direction=-1 homing_seek_velocity=5000 homing_latch_velocity=500 homing_backoff=200");
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

TEST_F(HomingTest, SdskHomesWithHlfb) {
    // Configure as ClearPath (SDSK)
    serial.SendLine("configure_sdsk motor=1");
    ctrl->Update();
    SET_HLFB(1, 1);  // HLFB_ASSERTED - motor ready
    serial.SendLine("enable motor=1");
    ctrl->Update();
    serial.ClearOutput();

    serial.SendLine("home motor=1");
    ctrl->Update();

    // SDSK motors use SDSK_SEEKING state (hard-stop homing via HLFB)
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetMotor(1)->homing_state, HomingState::SDSK_SEEKING);
    EXPECT_TRUE(ctrl->GetMotor(1)->moving);
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

    // Phase 1: Seeking - motor moves toward limit switch
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::SEEKING);

    // Trigger limit switch (ClearCore stops motor and sets alert)
    TRIGGER_NEG_LIMIT(0);
    ctrl->Update();

    // Phase 2: Backing off
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::BACKING_OFF);
    serial.ClearOutput();

    // Complete backoff move
    COMPLETE_MOVE(0);
    ctrl->Update();

    // Phase 3: Latching - slow approach
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::LATCHING);
    serial.ClearOutput();

    // Trigger limit switch again for final latch
    TRIGGER_NEG_LIMIT(0);
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

// === Homing in Positive Direction ===

TEST_F(HomingTest, HomingPositiveDirection) {
    // Reconfigure with positive homing direction
    serial.SendLine("configure_stepper motor=0 limit_pos_pin=7 homing_direction=1");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    serial.SendLine("home motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Should be seeking
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::SEEKING);

    // Trigger positive limit switch
    TRIGGER_POS_LIMIT(0);
    ctrl->Update();

    // Should advance to backing off
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::BACKING_OFF);
}

// === Limit Switch Required for Homing ===

TEST_F(HomingTest, LimitSwitchRequiredForHoming) {
    // Try to configure without limit switch in homing direction
    serial.SendLine("configure_stepper motor=0 homing_direction=-1");
    ctrl->Update();

    // Should fail because limit_neg_pin is required for homing in negative direction
    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("limit_neg_pin required"));
}

TEST_F(HomingTest, NoLimitSwitchRequiredWhenNotHoming) {
    // Configure without limit switch but with home_on_enable=0
    serial.SendLine("configure_stepper motor=0 home_on_enable=0");
    ctrl->Update();

    // Should succeed because homing is disabled
    EXPECT_TRUE(serial.HasOutput("ok"));
}
