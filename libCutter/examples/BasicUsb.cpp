/**
 * @file BasicUsb.cpp
 * @brief Basic USB serial example for Cutter
 *
 * This example demonstrates how to use the Cutter library with
 * USB serial communication on ClearCore.
 *
 * Wrap ConnectorUsb in Cutter::UsbSerial rather than passing it directly:
 * ConnectorUsb.PortIsOpen() sleeps 10 ms on every call, and Controller checks
 * it on every Update(), which would cap the main loop at ~100 Hz.
 */

#include "ClearCore.h"
#include "Cutter.h"
#include "CutterUsbSerial.h"

Cutter::UsbSerial usbSerial(ConnectorUsb);
Cutter::Controller cutter(&usbSerial);

int main() {
    // Initialize ClearCore motors in step/direction mode
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    // Initialize USB serial
    ConnectorUsb.Mode(Connector::USB_CDC);
    ConnectorUsb.Speed(115200);
    ConnectorUsb.PortOpen();

    // Main loop - just call Update() as fast as possible
    while (true) {
        cutter.Update();
    }

    return 0;
}
