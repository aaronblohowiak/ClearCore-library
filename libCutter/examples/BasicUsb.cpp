/**
 * @file BasicUsb.cpp
 * @brief Basic USB serial example for Cutter
 *
 * This example demonstrates how to use the Cutter library with
 * USB serial communication on ClearCore.
 *
 * ClearCore's ConnectorUsb implements ClearCore::ISerial, which Cutter
 * uses directly - no wrapper needed.
 */

#include "ClearCore.h"
#include "Cutter.h"

// ConnectorUsb already implements ClearCore::ISerial
Cutter::Controller cutter(&ConnectorUsb);

int main() {
    // Initialize ClearCore motors in step/direction mode
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    // Initialize USB serial
    ConnectorUsb.Mode(Connector::USB_CDC);
    ConnectorUsb.Speed(115200);
    ConnectorUsb.PortOpen();

    // Wait for the USB host to open the port (asserts DTR), then announce
    // readiness. This banner is a pure string - no integer formatting - so it
    // doubles as a diagnostic: if it appears but commands stay silent, the
    // problem is in the response path, not USB/enumeration.
    while (!ConnectorUsb) {
        continue;
    }
    ConnectorUsb.SendLine("event type=ready version=1.0.0 protocol=1");

    // Main loop - just call Update() as fast as possible
    while (true) {
        cutter.Update();
    }

    return 0;
}
