/**
 * @file test_move_until.cpp
 * @brief Tests for the move_until command (run a velocity move until a
 *        non-limit sensor input trips, e.g. lowering a suction head until the
 *        vacuum sensor reports suction has formed). The motor's soft limits act
 *        as the travel bound for the "sensor never tripped" outcome.
 */

#include <gtest/gtest.h>
#include "Cutter.h"
#include "CutterHal.h"
#include "FakeHal.h"
#include "TestSerial.h"

using namespace Cutter;

// Sensor (e.g. vacuum switch) wired to a digital-input-capable pin.
static constexpr int SENSOR_PIN = 6;

class MoveUntilTest : public ::testing::Test {
protected:
    TestSerial serial;
    Controller* ctrl;

    void SetUp() override {
        RESET_HAL();
        ctrl = new Controller(&serial);
        serial.SendLine("ping");
        ctrl->Update();
        // Stepper with soft limits as the travel bound (-1000..1000).
        serial.SendLine("configure_stepper motor=0 homing_mode=none vel_max=10000 accel_max=50000 "
                        "soft_limits=1 soft_min=-1000 soft_max=1000");
        ctrl->Update();
        // Sensor pin as a digital input (configured before enable, in CONFIGURED).
        serial.SendLine("configure_digital_in pin=6");
        ctrl->Update();
        serial.SendLine("enable motor=0");
        ctrl->Update();
        serial.ClearOutput();
    }

    void TearDown() override {
        delete ctrl;
    }
};

// === Happy path: sensor trips, motor stops, position reported, no alert ===

TEST_F(MoveUntilTest, SensorTripStopsAndReportsPosition) {
    serial.SendLine("move_until motor=0 velocity=-500 until_pin=6 until_value=1");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(ctrl->GetState(), State::WORKING);
    EXPECT_EQ(ctrl->GetMotor(0)->stop_sensor_pin, SENSOR_PIN);
    serial.ClearOutput();

    // Lowering: position advances, sensor not yet tripped -> keep moving.
    g_fake.motor_position[0] = -200;
    ctrl->Update();
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
    EXPECT_FALSE(serial.HasEvent("sensor_stop"));

    // Vacuum forms: sensor trips at position -300.
    g_fake.motor_position[0] = -300;
    SET_PIN(SENSOR_PIN, true);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("sensor_stop"));
    EXPECT_TRUE(serial.HasOutput("motor=0"));
    EXPECT_TRUE(serial.HasOutput("pin=6"));
    EXPECT_TRUE(serial.HasOutput("position=-300"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(ctrl->GetMotor(0)->stop_sensor_pin, CutterHal::PIN_INVALID);
    EXPECT_EQ(ctrl->GetState(), State::READY);

    // Not a fault: no alert event, and a normal move works without clear_alerts.
    EXPECT_FALSE(serial.HasEvent("alert"));
    serial.ClearOutput();
    serial.SendLine("move motor=0 steps=10");
    ctrl->Update();
    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
}

// === Sensor never trips: soft limit bounds the move (no item found) ===

TEST_F(MoveUntilTest, SoftLimitStopsWhenSensorNeverTrips) {
    serial.SendLine("move_until motor=0 velocity=-500 until_pin=6 until_value=1");
    ctrl->Update();
    serial.ClearOutput();

    // Sensor stays low; head drives past the soft-limit floor.
    g_fake.motor_position[0] = -1500;
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("soft_limit"));
    EXPECT_TRUE(serial.HasOutput("position=-1500"));
    EXPECT_FALSE(serial.HasEvent("sensor_stop"));
    EXPECT_FALSE(serial.HasEvent("alert"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

// === Sensor already tripped at start: stop immediately, don't drive down ===

TEST_F(MoveUntilTest, StopsImmediatelyIfAlreadyTripped) {
    SET_PIN(SENSOR_PIN, true);  // vacuum already present before we command the move

    serial.SendLine("move_until motor=0 velocity=-500 until_pin=6 until_value=1");
    ctrl->Update();  // dispatch + CheckMotors run in the same Update()

    EXPECT_TRUE(serial.HasEvent("sensor_stop"));
    EXPECT_TRUE(serial.HasOutput("position=0"));  // stopped at the start position
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
    EXPECT_EQ(ctrl->GetState(), State::READY);
}

// === Inversion is honored (logical level, post-invert) ===

TEST_F(MoveUntilTest, HonorsConfiguredInversion) {
    // A fresh sensor pin configured inverted: raw LOW reads logical HIGH.
    const int inv_pin = 7;
    serial.SendLine("configure_digital_in pin=7 invert=1");
    ctrl->Update();
    SET_PIN(inv_pin, true);  // raw HIGH -> logical LOW with invert
    serial.SendLine("move_until motor=0 velocity=-500 until_pin=7 until_value=1");
    ctrl->Update();
    serial.ClearOutput();

    // Raw HIGH = logical LOW != until_value(1): keep moving.
    g_fake.motor_position[0] = -100;
    ctrl->Update();
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
    EXPECT_FALSE(serial.HasEvent("sensor_stop"));

    // Raw LOW = logical HIGH == until_value(1): trip.
    SET_PIN(inv_pin, false);
    ctrl->Update();
    EXPECT_TRUE(serial.HasEvent("sensor_stop"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

// === Validation ===

TEST_F(MoveUntilTest, RejectsNonDigitalInputPin) {
    // Pin 6 here is fine, but pin 0 was never configured as a digital input.
    serial.SendLine("move_until motor=0 velocity=-500 until_pin=0 until_value=1");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("digital input"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

TEST_F(MoveUntilTest, RequiresSoftLimits) {
    // Reconfigure motor 0 without soft limits.
    serial.SendLine("disable motor=0");
    ctrl->Update();
    serial.SendLine("configure_stepper motor=0 homing_mode=none vel_max=10000 accel_max=50000");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    serial.ClearOutput();

    serial.SendLine("move_until motor=0 velocity=-500 until_pin=6 until_value=1");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("soft limits"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

TEST_F(MoveUntilTest, RequiresUntilParams) {
    serial.SendLine("move_until motor=0 velocity=-500");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_FALSE(ctrl->GetMotor(0)->moving);
}

// === No leak: a later plain velocity move must not inherit the condition ===

TEST_F(MoveUntilTest, ConditionDoesNotLeakIntoNextMove) {
    // First move_until completes via sensor trip.
    serial.SendLine("move_until motor=0 velocity=-500 until_pin=6 until_value=1");
    ctrl->Update();
    g_fake.motor_position[0] = -100;
    SET_PIN(SENSOR_PIN, true);
    ctrl->Update();
    ASSERT_FALSE(ctrl->GetMotor(0)->moving);
    SET_PIN(SENSOR_PIN, false);
    serial.ClearOutput();

    // A plain velocity move should have no sensor stop condition...
    serial.SendLine("move_velocity motor=0 velocity=-500");
    ctrl->Update();
    EXPECT_EQ(ctrl->GetMotor(0)->stop_sensor_pin, CutterHal::PIN_INVALID);

    // ...even if the sensor trips, it keeps moving (only soft limit bounds it).
    SET_PIN(SENSOR_PIN, true);
    g_fake.motor_position[0] = -200;
    ctrl->Update();
    EXPECT_TRUE(ctrl->GetMotor(0)->moving);
    EXPECT_FALSE(serial.HasEvent("sensor_stop"));
}
