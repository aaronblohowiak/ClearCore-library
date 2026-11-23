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
    serial.SendLine("home motor=0");
    ctrl->Update();

    // Get internal seq from response
    std::string output = serial.GetOutput();
    size_t seq_pos = output.find("seq=");
    ASSERT_NE(seq_pos, std::string::npos);
    uint32_t internal_seq = 0;
    sscanf(output.c_str() + seq_pos, "seq=%u", &internal_seq);
    serial.ClearOutput();

    // Phase 1: Seeking - motor moves fast toward the limit switch
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::SEEKING);

    // Trigger limit switch (ClearCore stops motor and sets alert + live input)
    TRIGGER_NEG_LIMIT(0);
    ctrl->Update();

    // Phase 2: Releasing - backing away until the switch goes inactive
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::RELEASING);
    serial.ClearOutput();

    // Simulate the switch releasing as the motor backs off the flag
    SET_NEG_LIMIT(0, false);
    ctrl->Update();

    // Phase 3: Latching - slow approach back toward the switch
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::LATCHING);
    serial.ClearOutput();

    // Switch trips again at the precise datum
    TRIGGER_NEG_LIMIT(0);
    ctrl->Update();

    // Phase 4: Backing off - clearance move away from the switch, then zero
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::BACKING_OFF);
    serial.ClearOutput();

    // Complete the clearance backoff move
    COMPLETE_MOVE(0);
    ctrl->Update();

    // Should be complete, zeroed at the rest position (clear of the switch)
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::COMPLETE);
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(g_fake.motor_position[0], 0);  // Zero set at rest, off the switch

    EXPECT_TRUE(serial.HasEvent("homed"));
    EXPECT_TRUE(serial.HasOutput("motor=0"));
    char expected_seq[32];
    snprintf(expected_seq, sizeof(expected_seq), "seq=%u", internal_seq);
    EXPECT_TRUE(serial.HasOutput(expected_seq));
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

// Releasing waits for the live switch to clear; it does not advance on a fixed
// distance. While the switch stays hot, homing stays in RELEASING.
TEST_F(HomingTest, ReleasingWaitsForSwitchToClear) {
    serial.SendLine("home motor=0");
    ctrl->Update();
    serial.ClearOutput();

    TRIGGER_NEG_LIMIT(0);
    ctrl->Update();
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::RELEASING);

    // Switch still asserted across several ticks -> still releasing, not latching.
    ctrl->Update();
    ctrl->Update();
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::RELEASING);

    // Only once the switch actually releases does it advance to latching.
    SET_NEG_LIMIT(0, false);
    ctrl->Update();
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::LATCHING);
}

// Soft limits must not be enforced during homing: the position counter is not yet
// referenced and the seek deliberately drives into the limit (outside the
// soft-limit range). The velocity_move flag can be left set from a prior move.
TEST_F(HomingTest, SoftLimitsNotEnforcedDuringHoming) {
    serial.SendLine("configure_stepper motor=0 limit_neg_pin=6 homing_direction=-1 "
                    "soft_limits=true soft_min=0 soft_max=100000");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    // Simulate a sticky velocity-move flag left over from a prior move_velocity.
    ctrl->GetMotor(0)->velocity_move = true;

    serial.SendLine("home motor=0");
    ctrl->Update();
    ASSERT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::SEEKING);

    // Seek drives the position below the soft-limit floor (0).
    g_fake.motor_position[0] = -500;
    ctrl->Update();

    // No soft_limit event fires and homing continues uninterrupted.
    EXPECT_FALSE(serial.HasEvent("soft_limit"));
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::SEEKING);
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
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

    // Should advance to releasing (back away until the switch clears)
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::RELEASING);

    // Release the switch -> slow latch approach
    SET_POS_LIMIT(0, false);
    ctrl->Update();
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::LATCHING);

    // Re-trigger at the datum -> clearance backoff -> zero
    TRIGGER_POS_LIMIT(0);
    ctrl->Update();
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::BACKING_OFF);
    COMPLETE_MOVE(0);
    ctrl->Update();
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::COMPLETE);
    EXPECT_EQ(g_fake.motor_position[0], 0);
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

// === Limit switch polarity (NC default, NO via limit_invert) ===

TEST_F(HomingTest, LimitSwitchesDefaultToNormallyClosed) {
    // SetUp configured motor 0 without limit_invert.
    EXPECT_FALSE(g_fake.limit_switch_invert[0]);
    EXPECT_FALSE(ctrl->GetMotor(0)->limit_invert);

    serial.SendLine("get_status");
    ctrl->Update();
    EXPECT_FALSE(serial.HasOutput("limit_invert"));
}

TEST_F(HomingTest, StepperLimitInvertSelectsNormallyOpen) {
    serial.SendLine("configure_stepper motor=0 limit_neg_pin=6 limit_invert=true");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(g_fake.limit_switch_invert[0]);
    EXPECT_TRUE(ctrl->GetMotor(0)->limit_invert);
    EXPECT_EQ(g_fake.limit_switch_neg_pin[0], 6);

    serial.ClearOutput();
    serial.SendLine("get_status");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("limit_invert=1"));

    // Reconfiguring without the flag restores NC logic.
    serial.SendLine("configure_stepper motor=0 limit_neg_pin=6");
    ctrl->Update();
    EXPECT_FALSE(g_fake.limit_switch_invert[0]);
    EXPECT_FALSE(ctrl->GetMotor(0)->limit_invert);
}

TEST_F(HomingTest, SdskLimitInvertSelectsNormallyOpen) {
    serial.SendLine("configure_sdsk motor=1 homing_mode=limit_switch limit_neg_pin=7 limit_invert=true");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(g_fake.limit_switch_invert[1]);
    EXPECT_TRUE(ctrl->GetMotor(1)->limit_invert);

    // Non-limit-switch homing modes don't use limit switches: polarity resets.
    serial.SendLine("configure_sdsk motor=1 homing_mode=msp limit_invert=true");
    ctrl->Update();
    EXPECT_FALSE(g_fake.limit_switch_invert[1]);
    EXPECT_FALSE(ctrl->GetMotor(1)->limit_invert);
}

// === Contact handling: wait for rest, check move results ===

// ClearCore latches the limit alert before it stops the motor (the stop waits
// for a tick that emits a step). SEEKING must not reverse until the motor is at
// rest, or ClearCore's deferred limit stop cancels the reverse move.
TEST_F(HomingTest, SeekingWaitsForMotorToStopBeforeReversing) {
    serial.SendLine("home motor=0");
    ctrl->Update();
    ASSERT_EQ(g_fake.motor_velocity[0], -5000);

    LATCH_NEG_LIMIT_MOVING(0);  // alert latched, motor still moving
    ctrl->Update();
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::SEEKING);
    EXPECT_EQ(g_fake.motor_velocity[0], -5000);           // no reverse yet
    EXPECT_TRUE(g_fake.motion_canceled_neg_limit[0]);     // alert left latched

    STOP_MOTOR(0);  // ClearCore's limit stop takes effect
    ctrl->Update();
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::RELEASING);
    EXPECT_EQ(g_fake.motor_velocity[0], 5000);            // backing away
    EXPECT_FALSE(g_fake.motion_canceled_neg_limit[0]);    // cleared at rest
}

TEST_F(HomingTest, LatchingWaitsForMotorToStopBeforeBackingOff) {
    serial.SendLine("home motor=0");
    ctrl->Update();
    TRIGGER_NEG_LIMIT(0);
    ctrl->Update();
    SET_NEG_LIMIT(0, false);
    ctrl->Update();
    ASSERT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::LATCHING);
    ASSERT_EQ(g_fake.motor_velocity[0], -500);

    LATCH_NEG_LIMIT_MOVING(0);
    ctrl->Update();
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::LATCHING);
    EXPECT_EQ(g_fake.motor_velocity[0], -500);            // still creeping in

    STOP_MOTOR(0);
    ctrl->Update();
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::BACKING_OFF);
    EXPECT_EQ(g_fake.motor_target[0], g_fake.motor_position[0] + 200);
}

// Stopping at the switch between phases is not the end of the move: no "done",
// and the motor stays "moving" until "homed".
TEST_F(HomingTest, StopAtSwitchDuringHomingIsNotDone) {
    serial.SendLine("home motor=0");
    ctrl->Update();
    serial.ClearOutput();

    TRIGGER_NEG_LIMIT(0);
    ctrl->Update();
    EXPECT_FALSE(serial.HasEvent("done"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(ctrl->GetState(), State::WORKING);

    SET_NEG_LIMIT(0, false);
    ctrl->Update();
    TRIGGER_NEG_LIMIT(0);
    ctrl->Update();
    EXPECT_FALSE(serial.HasEvent("done"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);

    COMPLETE_MOVE(0);
    ctrl->Update();
    EXPECT_FALSE(serial.HasEvent("done"));
    EXPECT_TRUE(serial.HasEvent("homed"));
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

TEST_F(HomingTest, RejectedSeekReportsErrorInsteadOfHanging) {
    SET_MOVE_REJECTED(0, true);
    serial.SendLine("home motor=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("code=307"));
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::IDLE);
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

// ClearCore rejects a seek into an already-active limit (latching its alert);
// that is contact, not a failure.
TEST_F(HomingTest, SeekRejectedBecauseAlreadyOnSwitchProceeds) {
    REJECT_NEXT_MOVES(0, 1);                     // only the seek is rejected
    g_fake.motion_canceled_neg_limit[0] = true;  // ClearCore latches on reject
    SET_NEG_LIMIT(0, true);
    serial.SendLine("home motor=0");
    ctrl->Update();

    // The home command succeeds, and SEEKING treats the switch as contact
    // right away: the motor is at rest, so it backs off in the same pass.
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_FALSE(serial.HasOutput("code=307"));
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::RELEASING);
    EXPECT_EQ(g_fake.motor_velocity[0], 5000);
}

TEST_F(HomingTest, RejectedReleaseMoveAbortsHoming) {
    serial.SendLine("home motor=0");
    ctrl->Update();
    serial.ClearOutput();

    TRIGGER_NEG_LIMIT(0);
    SET_MOVE_REJECTED(0, true);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("error"));
    EXPECT_TRUE(serial.HasOutput("code=307"));
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::IDLE);
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
    EXPECT_FALSE(ctrl->GetMotor(0)->homed);
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

TEST_F(HomingTest, RejectedLatchMoveAbortsHoming) {
    serial.SendLine("home motor=0");
    ctrl->Update();
    TRIGGER_NEG_LIMIT(0);
    ctrl->Update();
    serial.ClearOutput();

    SET_MOVE_REJECTED(0, true);
    SET_NEG_LIMIT(0, false);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("error"));
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::IDLE);
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

TEST_F(HomingTest, RejectedBackoffMoveAbortsHoming) {
    serial.SendLine("home motor=0");
    ctrl->Update();
    TRIGGER_NEG_LIMIT(0);
    ctrl->Update();
    SET_NEG_LIMIT(0, false);
    ctrl->Update();
    serial.ClearOutput();

    TRIGGER_NEG_LIMIT(0);
    SET_MOVE_REJECTED(0, true);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("error"));
    EXPECT_FALSE(serial.HasEvent("homed"));
    EXPECT_EQ(ctrl->GetMotor(0)->homing_state, HomingState::IDLE);
    EXPECT_EQ(ctrl->GetState(), State::READY);
}
