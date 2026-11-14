/**
 * @file CutterUsbSerial.cpp
 * @brief Non-blocking USB connection check (see CutterUsbSerial.h)
 *
 * This file is only compiled when CUTTER_PLATFORM_CLEARCORE is defined.
 */

#ifdef CUTTER_PLATFORM_CLEARCORE

#include "CutterUsbSerial.h"
#include "UsbManager.h"

namespace Cutter {

bool UsbSerial::PortIsOpen() {
    // Copied from ClearCore::UsbManager::Connected(), which is private. If
    // that function changes in a ClearCore update, update this to match.
    //   - the CDC interface is enabled (the host has configured the device)
    //   - the host has asserted DTR (it opened the port)
    //   - the USB peripheral is active, not suspended (a pulled cable may
    //     leave DTR set, so DTR alone is not enough)
    return cdcdf_acm_is_enabled() &&
           ClearCore::UsbManager::Instance().LineState().rs232.DTR &&
           USB->DEVICE.FSMSTATUS.bit.FSMSTATE == USB_FSMSTATUS_FSMSTATE_ON;
}

}  // namespace Cutter

#endif  // CUTTER_PLATFORM_CLEARCORE
