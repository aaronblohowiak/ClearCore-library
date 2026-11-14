
// CUTTER_PLATFORM_CLEARCORE is defined by the project (Debug and Release).
#include "ClearCore.h"
#include "Cutter.h"
#include "CutterUsbSerial.h"



/**
 * Reference https://teknic-inc.github.io/ClearCore-library/_microchip_install.html
 * for more information about how this ProjectTemplate should be used.
 */


// Wrap ConnectorUsb rather than passing it directly: ConnectorUsb.PortIsOpen()
// sleeps 10 ms on every call, and Controller checks it on every Update(),
// which would cap the main loop at ~100 Hz.
Cutter::UsbSerial usbSerial(ConnectorUsb);
Cutter::Controller cutter(&usbSerial);

int main() {
// Initialize ClearCore motors in step/direction mode
MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

// Initialize USB serial
ConnectorUsb.Mode(Connector::USB_CDC);
ConnectorUsb.Speed(115200);
ConnectorUsb.PortOpen();


while(!ConnectorUsb){continue;}
ConnectorUsb.SendLine("event type=ready msg=hello");

// Main loop - just call Update() as fast as possible
while (true) {
cutter.Update();
}

return 0;
}