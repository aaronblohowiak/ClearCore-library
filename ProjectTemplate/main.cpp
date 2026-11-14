
#define CUTTER_PLATFORM_CLEARCORE true
#include "ClearCore.h"
#include "Cutter.h"



/**
 * Reference https://teknic-inc.github.io/ClearCore-library/_microchip_install.html
 * for more information about how this ProjectTemplate should be used.
 */


// ConnectorUsb already implements ClearCore::ISerial
Cutter::Controller cutter(&ConnectorUsb);

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