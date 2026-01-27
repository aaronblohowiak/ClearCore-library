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

TEST_F(HomingTest, SdskHomesWithLimitSwitch) {
    // Configure SDSK with limit switch homing (instead of MSP)
    serial.SendLine("configure_sdsk motor=1 homing_mode=limit_switch limit_neg_pin=7");
    ctrl->Update();
    SET_HLFB(1, 1);  // HLFB_ASSERTED - motor ready
    serial.SendLine("enable motor=1");
    ctrl->Update();
    serial.ClearOutput();

    serial.SendLine("home motor=1");
    ctrl->Update();

    // SDSK with limit_switch mode uses same SEEKING state as steppers
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetMotor(1)->homing_state, HomingState::SEEKING);
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

    // Get internal seq from response
    std::string output = serial.GetOutput();
    size_t seq_pos = output.find("seq=");
    ASSERT_NE(seq_pos, std::string::npos);
    uint32_t internal_seq = 0;
    sscanf(output.c_str() + seq_pos, "seq=%u", &internal_seq);
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
    char expected_seq[32];
    snprintf(expected_seq, sizeof(expected_seq), "seq=%u", internal_seq);
    EXPECT_TRUE(serial.HasOutput(expected_seq));
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
    // Configure without limit switch but with homing_mode=none
    serial.SendLine("configure_stepper motor=0 homing_mode=none");
    ctrl->Update();

    // Should succeed because homing is disabled
    EXPECT_TRUE(serial.HasOutput("ok"));
}
