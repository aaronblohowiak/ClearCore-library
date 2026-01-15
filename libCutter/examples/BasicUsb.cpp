/**
 * @file BasicUsb.cpp
 * @brief Basic USB serial example for Cutter
 *
 * This example demonstrates how to use the Cutter library with
 * USB serial communication on ClearCore.
 */

#include "ClearCore.h"
#include "Cutter.h"

// USB Serial wrapper implementing ISerial
class UsbSerial : public Cutter::ISerial {
public:
    int16_t CharGet() override {
        return ConnectorUsb.CharGet();
    }

    int16_t CharPeek() override {
        return ConnectorUsb.CharPeek();
    }

    int AvailableForRead() override {
        return ConnectorUsb.AvailableForRead();
    }

    bool SendChar(char c) override {
        return ConnectorUsb.SendChar(c);
    }

    bool Send(const char* str) override {
        return ConnectorUsb.Send(str);
    }
};

UsbSerial usbSerial;
Cutter::Controller cutter(&usbSerial);

int main() {
    // Initialize ClearCore
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    // Initialize USB serial
    ConnectorUsb.Mode(Connector::USB_CDC);
    ConnectorUsb.Speed(115200);
    ConnectorUsb.PortOpen();

    // Main loop
    while (true) {
        cutter.Update();
    }

    return 0;
}
