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

    // Main loop - just call Update() as fast as possible
    while (true) {
        cutter.Update();
    }

    return 0;
}
