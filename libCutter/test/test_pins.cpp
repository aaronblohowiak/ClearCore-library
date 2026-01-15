/**
 * @file test_pins.cpp
 * @brief Pin configuration and operation tests
 */

#include <gtest/gtest.h>
#include "Cutter.h"
#include "FakeHal.h"
#include "TestSerial.h"

using namespace Cutter;

class PinTest : public ::testing::Test {
protected:
    TestSerial serial;
    Controller* ctrl;

    void SetUp() override {
        RESET_HAL();
        ctrl = new Controller(&serial);
        // Connect the controller
        serial.SendLine("ping");
        ctrl->Update();
        serial.ClearOutput();
    }

    void TearDown() override {
        delete ctrl;
    }
};

// === Pin Capability Tests ===

TEST(PinCapability, DigitalInAllPins) {
    for (uint8_t i = 0; i < 13; i++) {
        EXPECT_TRUE(PinSupports(i, PinCap::DIGITAL_IN)) << "Pin " << (int)i;
    }
}

TEST(PinCapability, DigitalOutPins0to5) {
    for (uint8_t i = 0; i <= 5; i++) {
        EXPECT_TRUE(PinSupports(i, PinCap::DIGITAL_OUT)) << "Pin " << (int)i;
    }
    for (uint8_t i = 6; i < 13; i++) {
        EXPECT_FALSE(PinSupports(i, PinCap::DIGITAL_OUT)) << "Pin " << (int)i;
    }
}

TEST(PinCapability, AnalogInPins9to12) {
    for (uint8_t i = 0; i <= 8; i++) {
        EXPECT_FALSE(PinSupports(i, PinCap::ANALOG_IN)) << "Pin " << (int)i;
    }
    for (uint8_t i = 9; i <= 12; i++) {
        EXPECT_TRUE(PinSupports(i, PinCap::ANALOG_IN)) << "Pin " << (int)i;
    }
}

TEST(PinCapability, HBridgePins4and5) {
    EXPECT_FALSE(PinSupports(3, PinCap::H_BRIDGE));
    EXPECT_TRUE(PinSupports(4, PinCap::H_BRIDGE));
    EXPECT_TRUE(PinSupports(5, PinCap::H_BRIDGE));
    EXPECT_FALSE(PinSupports(6, PinCap::H_BRIDGE));
}

// === Digital Input Configuration ===

TEST_F(PinTest, ConfigureDigitalIn) {
    serial.SendLine("configure_digital_in pin=6");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("pin=6"));

    PinSlot* pin = ctrl->GetPin(6);
    EXPECT_EQ(pin->mode, PinMode::DIGITAL_IN);
}

TEST_F(PinTest, ConfigureDigitalInWithReportChanges) {
    serial.SendLine("configure_digital_in pin=6 report_changes=1");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));

    PinSlot* pin = ctrl->GetPin(6);
    EXPECT_EQ(pin->mode, PinMode::DIGITAL_IN);
    EXPECT_TRUE(pin->digital_in.report_changes);
}

TEST_F(PinTest, ConfigureDigitalInInvalidPin) {
    serial.SendLine("configure_digital_in pin=99");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("Invalid pin"));
}

// === Digital Output Configuration ===

TEST_F(PinTest, ConfigureDigitalOut) {
    serial.SendLine("configure_digital_out pin=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetPin(0)->mode, PinMode::DIGITAL_OUT);
}

TEST_F(PinTest, ConfigureDigitalOutInvalidPin) {
    // Pin 6 doesn't support digital output
    serial.SendLine("configure_digital_out pin=6");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("does not support"));
}

// === Analog Input Configuration ===

TEST_F(PinTest, ConfigureAnalogIn) {
    serial.SendLine("configure_analog_in pin=9");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetPin(9)->mode, PinMode::ANALOG_IN);
}

TEST_F(PinTest, ConfigureAnalogInInvalidPin) {
    serial.SendLine("configure_analog_in pin=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("does not support"));
}

// === PWM Configuration ===

TEST_F(PinTest, ConfigurePwm) {
    serial.SendLine("configure_pwm pin=0 duty=32000 frequency=1000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetPin(0)->mode, PinMode::PWM);
    EXPECT_EQ(ctrl->GetPin(0)->pwm.duty, 32000);
}

// === H-Bridge Configuration ===

TEST_F(PinTest, ConfigureHBridge) {
    serial.SendLine("configure_hbridge pin=4");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(ctrl->GetPin(4)->mode, PinMode::H_BRIDGE);
}

TEST_F(PinTest, ConfigureHBridgeInvalidPin) {
    serial.SendLine("configure_hbridge pin=0");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("error"));
    EXPECT_TRUE(serial.HasOutput("does not support"));
}

// === Read/Write Operations ===

TEST_F(PinTest, ReadDigitalIn) {
    serial.SendLine("configure_digital_in pin=6");
    ctrl->Update();
    serial.ClearOutput();

    SET_PIN(6, true);
    serial.SendLine("read_pin pin=6");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(serial.HasOutput("value=1"));
}

TEST_F(PinTest, WriteDigitalOut) {
    serial.SendLine("configure_digital_out pin=0");
    ctrl->Update();
    serial.ClearOutput();

    serial.SendLine("write_pin pin=0 value=1");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(g_fake.digital_pins[0]);
}

TEST_F(PinTest, SetPwm) {
    serial.SendLine("configure_pwm pin=0");
    ctrl->Update();
    serial.ClearOutput();

    serial.SendLine("set_pwm pin=0 duty=50000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(g_fake.pwm_duty[0], 50000);
}

TEST_F(PinTest, SetHBridge) {
    serial.SendLine("configure_hbridge pin=4");
    ctrl->Update();
    serial.ClearOutput();

    serial.SendLine("set_hbridge pin=4 value=10000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_EQ(g_fake.hbridge_value[0], 10000);
}

// === Tone Operations ===

TEST_F(PinTest, StartStopTone) {
    serial.SendLine("configure_hbridge pin=4");
    ctrl->Update();
    serial.ClearOutput();

    serial.SendLine("start_tone pin=4 frequency=440 amplitude=16000");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_TRUE(g_fake.tone_active[0]);
    EXPECT_EQ(g_fake.tone_freq[0], 440);

    serial.ClearOutput();
    serial.SendLine("stop_tone pin=4");
    ctrl->Update();

    EXPECT_TRUE(serial.HasOutput("ok"));
    EXPECT_FALSE(g_fake.tone_active[0]);
}

// === Input Change Events ===

TEST_F(PinTest, DigitalInputChangeEvent) {
    serial.SendLine("configure_digital_in pin=6 report_changes=1");
    ctrl->Update();
    serial.ClearOutput();

    // Initial value is false
    EXPECT_FALSE(g_fake.digital_pins[6]);

    // Change to true
    SET_PIN(6, true);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("input"));
    EXPECT_TRUE(serial.HasOutput("pin=6"));
    EXPECT_TRUE(serial.HasOutput("value=1"));
}
