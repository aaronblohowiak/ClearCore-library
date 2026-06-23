/**
 * @file test_errors.cpp
 * @brief Error handling and recovery tests
 */

#include <gtest/gtest.h>
#include <cstdio>
#include "Cutter.h"
#include "FakeHal.h"
#include "TestSerial.h"

using namespace Cutter;

// Issue get_next_seq and return the device's reported next internal seq.
// This is the canonical way a host (re)synchronizes its lockstep counter.
static uint32_t QueryNextSeq(TestSerial& serial, Controller* ctrl) {
    serial.ClearOutput();
    serial.SendLine("get_next_seq");
    ctrl->Update();
    std::string out = serial.GetOutput();
    size_t pos = out.find("next_seq=");
    EXPECT_NE(pos, std::string::npos) << "get_next_seq response: " << out;
    uint32_t next = 0;
    sscanf(out.c_str() + pos, "next_seq=%u", &next);
    serial.ClearOutput();
    return next;
}

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
    // seq must be lockstep-valid (next internal seq is 2 after SetUp's ping) so
    // the command reaches dispatch; the unknown-command error echoes the seq.
    serial.SendLine("foobar seq=2");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("Unknown command"));
    EXPECT_TRUE(serial.HasOutput("seq=2"));
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

// Lockstep note: SetUp() sends one seq-less `ping`, so the device's next
// internal seq is 2 at the start of every test below. A supplied seq must
// match that counter exactly.

TEST_F(ErrorTest, StaleSeqRejected) {
    // Walk the counter forward in lockstep: seq 2 then 3.
    serial.SendLine("ping seq=2");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    serial.ClearOutput();
    serial.SendLine("ping seq=3");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    serial.ClearOutput();

    // A seq behind the highest seen (2 < max_seen 3) is stale - rejected.
    serial.SendLine("ping seq=2");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("code=105"));  // STALE_SEQ
    EXPECT_TRUE(serial.HasOutput("seq=2"));
    EXPECT_TRUE(serial.HasOutput("max_seen=3"));
    // Should NOT enter error state - just reject this command
    EXPECT_NE(ctrl->GetState(), State::ERROR);
}

TEST_F(ErrorTest, DuplicateSeqRejected) {
    // First command in lockstep with the device counter (seq=2).
    serial.SendLine("ping seq=2");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    serial.ClearOutput();

    // Re-sending the same seq=2 is now behind the counter - stale, rejected.
    serial.SendLine("ping seq=2");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("code=105"));  // STALE_SEQ
    EXPECT_NE(ctrl->GetState(), State::ERROR);
}

TEST_F(ErrorTest, LockstepSeqAccepted) {
    // Each supplied seq exactly equals the device's next internal seq.
    serial.SendLine("ping seq=2");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    serial.ClearOutput();

    serial.SendLine("ping seq=3");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    serial.ClearOutput();

    serial.SendLine("ping seq=4");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
}

TEST_F(ErrorTest, SeqAheadRejected) {
    // A supplied seq ahead of the device counter (next is 2, host sends 5)
    // means the host counted a command the device never processed: SEQ_MISMATCH.
    serial.SendLine("ping seq=5");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("code=107"));  // SEQ_MISMATCH
    EXPECT_TRUE(serial.HasOutput("seq=5"));
    EXPECT_TRUE(serial.HasOutput("expected=2"));  // resync target
    EXPECT_NE(ctrl->GetState(), State::ERROR);
    serial.ClearOutput();

    // The rejected command did not advance the counter: resyncing to the
    // device's reported next seq (2) is accepted.
    serial.SendLine("ping seq=2");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
}

// Canonical host workflow: drive the supplied seq straight from get_next_seq.
// get_next_seq reports the exact seq the very next command must carry, so a host
// can always (re)sync by querying it and echoing the value. Note get_next_seq is
// itself a command that advances the counter, so successive queries return
// strictly increasing values.
TEST_F(ErrorTest, GetNextSeqDrivesLockstep) {
    uint32_t prev = 0;
    for (int i = 0; i < 3; i++) {
        uint32_t next = QueryNextSeq(serial, ctrl);
        EXPECT_GT(next, prev) << "get_next_seq must report an advancing counter";
        prev = next;

        char cmd[32];
        snprintf(cmd, sizeof(cmd), "ping seq=%u", next);
        serial.SendLine(cmd);
        ctrl->Update();
        EXPECT_TRUE(serial.HasOutput("ok"))
            << "ping seq=" << next << " (from get_next_seq) should be in lockstep";
    }
}

// Recovery workflow: after drifting ahead (e.g. a dropped command the host still
// counted), the host re-reads get_next_seq and resyncs instead of guessing.
TEST_F(ErrorTest, ResyncViaGetNextSeqAfterMismatch) {
    uint32_t next = QueryNextSeq(serial, ctrl);  // seq the next command must carry

    // Host believes it is further along than the device and sends a seq the
    // device has not reached: rejected as SEQ_MISMATCH, not silently accepted.
    char ahead[32];
    snprintf(ahead, sizeof(ahead), "ping seq=%u", next + 5);
    serial.SendLine(ahead);
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("code=107"));  // SEQ_MISMATCH
    char expected[32];
    snprintf(expected, sizeof(expected), "expected=%u", next);
    EXPECT_TRUE(serial.HasOutput(expected));  // device reports where it actually is
    // The rejection must not advance the counter (no id assigned for it).
    EXPECT_FALSE(serial.HasOutput("ok"));
    serial.ClearOutput();

    // Recover: re-read get_next_seq and send exactly that value to resync.
    uint32_t resync = QueryNextSeq(serial, ctrl);
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "ping seq=%u", resync);
    serial.SendLine(cmd);
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
}

// The value get_next_seq returns is exactly the seq the next command must carry,
// EVEN when that next command is itself get_next_seq: get_next_seq reports the
// post-increment counter, so `get_next_seq -> N` then `get_next_seq seq=N` is ok
// and reports N+1.
TEST_F(ErrorTest, GetNextSeqAcceptsItsOwnReturnedSeq) {
    uint32_t n = QueryNextSeq(serial, ctrl);  // next command must carry n

    char cmd[40];
    snprintf(cmd, sizeof(cmd), "get_next_seq seq=%u", n);
    serial.SendLine(cmd);
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok")) << "get_next_seq seq=" << n
        << " (its own returned value) should be in lockstep";
    char nextOut[40];
    snprintf(nextOut, sizeof(nextOut), "next_seq=%u", n + 1);
    EXPECT_TRUE(serial.HasOutput(nextOut));
}

// A desync is a signal, not a silent drop: an unknown command (lockstep-valid
// seq, so it reaches dispatch) still CONSUMES a seq and returns an error. The
// host that sent it learns the firmware is not where it believed.
TEST_F(ErrorTest, UnknownCommandConsumesSeqAndErrors) {
    uint32_t n = QueryNextSeq(serial, ctrl);

    char bad[40];
    snprintf(bad, sizeof(bad), "bogus seq=%u", n);
    serial.SendLine(bad);
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("code=100"));  // UNKNOWN_COMMAND, still an error
    serial.ClearOutput();

    // The unknown command advanced the counter: the next valid command must use
    // n+1 (n is now stale).
    char stale[40];
    snprintf(stale, sizeof(stale), "ping seq=%u", n);
    serial.SendLine(stale);
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("code=105"));  // STALE_SEQ - counter moved on
    serial.ClearOutput();

    char good[40];
    snprintf(good, sizeof(good), "ping seq=%u", n + 1);
    serial.SendLine(good);
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
