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

// === Digital Input Invert ===

TEST_F(PinTest, DigitalInputInvert) {
    serial.SendLine("configure_digital_in pin=6 invert=1");
    ctrl->Update();

    PinSlot* pin = ctrl->GetPin(6);
    EXPECT_TRUE(pin->digital_in.invert);
}

// === Digital Input Error Trigger ===

TEST_F(PinTest, DigitalInputErrorTrigger) {
    serial.SendLine("configure_digital_in pin=6 error_trigger=1 error_value=1");
    ctrl->Update();
    serial.ClearOutput();

    // Trigger the error condition
    SET_PIN(6, true);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("error"));
    EXPECT_TRUE(serial.HasOutput("Error trigger"));
    EXPECT_EQ(ctrl->GetState(), State::ERROR);
}

TEST_F(PinTest, DigitalInputErrorTriggerWithInvert) {
    // Error triggers when inverted value is true
    // So physical false (inverted = true) should trigger
    serial.SendLine("configure_digital_in pin=6 invert=1 error_trigger=1 error_value=1");
    ctrl->Update();
    serial.ClearOutput();

    // Physical false -> inverted true -> triggers error
    SET_PIN(6, false);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("error"));
    EXPECT_EQ(ctrl->GetState(), State::ERROR);
}

// === Digital Output Timeout ===

TEST_F(PinTest, DigitalOutputTimeout) {
    serial.SendLine("configure_digital_out pin=0 max_raised_ms=100");
    ctrl->Update();
    serial.ClearOutput();

    // Set pin high
    serial.SendLine("write_pin pin=0 value=1");
    ctrl->Update();
    serial.ClearOutput();

    // Time passes but not enough
    ADVANCE_TIME(50);
    ctrl->Update();
    EXPECT_NE(ctrl->GetState(), State::ERROR);

    // Time passes beyond timeout
    ADVANCE_TIME(60);  // Total 110ms > 100ms
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("error"));
    EXPECT_TRUE(serial.HasOutput("timeout"));
    EXPECT_EQ(ctrl->GetState(), State::ERROR);
}

TEST_F(PinTest, DigitalOutputNoTimeoutWhenLow) {
    serial.SendLine("configure_digital_out pin=0 max_raised_ms=100");
    ctrl->Update();
    serial.ClearOutput();

    // Pin stays low - no timeout should occur
    ADVANCE_TIME(200);
    ctrl->Update();

    EXPECT_NE(ctrl->GetState(), State::ERROR);
}

TEST_F(PinTest, DigitalOutputTimeoutResetsOnReRaise) {
    // Verify that setting pin low then high again resets the timeout timer
    serial.SendLine("configure_digital_out pin=0 max_raised_ms=100");
    ctrl->Update();
    serial.ClearOutput();

    // Set pin high
    serial.SendLine("write_pin pin=0 value=1");
    ctrl->Update();

    // Wait 80ms (within timeout)
    ADVANCE_TIME(80);
    ctrl->Update();
    EXPECT_NE(ctrl->GetState(), State::ERROR);

    // Pull pin low
    serial.SendLine("write_pin pin=0 value=0");
    ctrl->Update();

    // Wait another 50ms while low (total 130ms from first raise)
    ADVANCE_TIME(50);
    ctrl->Update();
    EXPECT_NE(ctrl->GetState(), State::ERROR);

    // Set pin high again - timer should reset
    serial.SendLine("write_pin pin=0 value=1");
    ctrl->Update();

    // Wait 80ms from second raise (would be 210ms from first raise)
    ADVANCE_TIME(80);
    ctrl->Update();
    EXPECT_NE(ctrl->GetState(), State::ERROR);  // Should NOT timeout yet

    // Wait another 30ms (110ms from second raise) - NOW it should timeout
    ADVANCE_TIME(30);
    ctrl->Update();
    EXPECT_EQ(ctrl->GetState(), State::ERROR);
    EXPECT_TRUE(serial.HasEvent("error"));
}

// === Digital Output On Error ===

TEST_F(PinTest, DigitalOutputOnError) {
    // Configure output with on_error=0 (turn off on error)
    serial.SendLine("configure_digital_out pin=0 on_error=0");
    ctrl->Update();
    serial.SendLine("write_pin pin=0 value=1");  // Set high
    ctrl->Update();
    EXPECT_TRUE(g_fake.digital_pins[0]);

    // Configure input with error trigger
    serial.SendLine("configure_digital_in pin=6 error_trigger=1 error_value=1");
    ctrl->Update();
    serial.ClearOutput();

    // Trigger error
    SET_PIN(6, true);
    ctrl->Update();

    // Output should be set to on_error value (false)
    EXPECT_FALSE(g_fake.digital_pins[0]);
}

// === Analog Error Thresholds ===

TEST_F(PinTest, AnalogErrorThresholdLow) {
    serial.SendLine("configure_analog_in pin=9 error_low=100 error_high=4000");
    ctrl->Update();
    serial.ClearOutput();

    // Value below error_low threshold
    SET_ANALOG(9, 50);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("error"));
    EXPECT_TRUE(serial.HasOutput("threshold"));
    EXPECT_EQ(ctrl->GetState(), State::ERROR);
}

TEST_F(PinTest, AnalogErrorThresholdHigh) {
    serial.SendLine("configure_analog_in pin=9 error_low=100 error_high=4000");
    ctrl->Update();
    serial.ClearOutput();

    // Value above error_high threshold
    SET_ANALOG(9, 4100);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("error"));
    EXPECT_EQ(ctrl->GetState(), State::ERROR);
}

TEST_F(PinTest, AnalogErrorThresholdInRange) {
    // Set value in range BEFORE configuring (initial read happens during config)
    SET_ANALOG(9, 2000);

    serial.SendLine("configure_analog_in pin=9 error_low=100 error_high=4000");
    ctrl->Update();
    serial.ClearOutput();

    // Value stays in range - no error
    ctrl->Update();

    EXPECT_NE(ctrl->GetState(), State::ERROR);
}

// === Analog Periodic Reporting ===

TEST_F(PinTest, AnalogPeriodicReporting) {
    serial.SendLine("configure_analog_in pin=9 report_interval=100");
    ctrl->Update();
    serial.ClearOutput();

    SET_ANALOG(9, 1000);

    // Advance time and update
    ADVANCE_TIME(110);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("analog"));
    EXPECT_TRUE(serial.HasOutput("pin=9"));
    EXPECT_TRUE(serial.HasOutput("value=1000"));
}

// === Time Rollover Tests ===

TEST_F(PinTest, DigitalOutputTimeoutRollover) {
    // Set time near UINT32_MAX
    g_fake.time_ms = 0xFFFFFF00;

    serial.SendLine("configure_digital_out pin=0 max_raised_ms=100");
    ctrl->Update();
    serial.SendLine("write_pin pin=0 value=1");
    ctrl->Update();
    serial.ClearOutput();

    // Advance past rollover (0xFFFFFF00 + 0x200 = 0x00000100)
    g_fake.time_ms = 0x00000100;  // Rolled over
    ctrl->Update();

    // 0x100 - 0xFFFFFF00 = 0x200 (512ms) > 100ms, should timeout
    EXPECT_TRUE(serial.HasEvent("error"));
    EXPECT_EQ(ctrl->GetState(), State::ERROR);
}

TEST_F(PinTest, AnalogReportingRollover) {
    // Set time near UINT32_MAX
    g_fake.time_ms = 0xFFFFFF00;

    serial.SendLine("configure_analog_in pin=9 report_interval=100");
    ctrl->Update();
    serial.ClearOutput();

    SET_ANALOG(9, 2000);

    // Advance past rollover
    g_fake.time_ms = 0x00000100;  // Rolled over, elapsed = 0x200 (512ms)
    ctrl->Update();

    // Should have reported since 512ms > 100ms interval
    EXPECT_TRUE(serial.HasEvent("analog"));
    EXPECT_TRUE(serial.HasOutput("value=2000"));
}

// === Hardware Fault Tests ===

TEST_F(PinTest, DigitalOutputOvercurrent) {
    serial.SendLine("configure_digital_out pin=0");
    ctrl->Update();
    serial.SendLine("write_pin pin=0 value=1");
    ctrl->Update();
    serial.ClearOutput();

    // Simulate overcurrent fault
    SET_PIN_FAULT(0, true);
    ctrl->Update();

    EXPECT_TRUE(serial.HasEvent("error"));
    EXPECT_TRUE(serial.HasOutput("code=203"));  // PIN_OVERCURRENT
    EXPECT_TRUE(serial.HasOutput("Pin overcurrent"));
    EXPECT_EQ(ctrl->GetState(), State::ERROR);
}

TEST_F(PinTest, NoFaultWhenPinNotFaulted) {
    serial.SendLine("configure_digital_out pin=0");
    ctrl->Update();
    serial.SendLine("write_pin pin=0 value=1");
    ctrl->Update();
    serial.ClearOutput();

    // No fault set
    ctrl->Update();

    EXPECT_FALSE(serial.HasEvent("error"));
    EXPECT_NE(ctrl->GetState(), State::ERROR);
}
