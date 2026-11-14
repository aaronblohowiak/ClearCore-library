/**
 * @file CutterUsbSerial.h
 * @brief ClearCore USB serial port with a connection check that does not block
 *
 * Pass this to Cutter::Controller instead of ConnectorUsb:
 *
 * \code{.cpp}
 * Cutter::UsbSerial usbSerial(ConnectorUsb);
 * Cutter::Controller cutter(&usbSerial);
 * \endcode
 *
 * Why: Controller::Update() checks PortIsOpen() on every pass to notice a host
 * connecting or disconnecting. ClearCore's SerialUsb::PortIsOpen() goes through
 * UsbManager::operator bool(), which sleeps Delay_ms(10) after reading the
 * state. That sleep is meant for a one-time wait at startup, but in Update()
 * it limited the whole main loop to about 100 passes per second.
 *
 * Everything except PortIsOpen() and operator bool() is passed straight
 * through to the wrapped SerialUsb.
 */

#ifndef CUTTER_USB_SERIAL_H
#define CUTTER_USB_SERIAL_H

#ifdef CUTTER_PLATFORM_CLEARCORE

#include "ClearCore.h"

namespace Cutter {

class UsbSerial : public ClearCore::ISerial {
public:
    explicit UsbSerial(ClearCore::SerialUsb& port) : m_port(port) {}

    /// True when the host has the port open: the same test as ClearCore's
    /// private UsbManager::Connected(), without the 10 ms sleep.
    bool PortIsOpen() override;

    operator bool() override {
        return PortIsOpen();
    }

    void Flush() override { m_port.Flush(); }
    void FlushInput() override { m_port.FlushInput(); }
    void PortOpen() override { m_port.PortOpen(); }
    void PortClose() override { m_port.PortClose(); }
    bool Speed(uint32_t bitsPerSecond) override { return m_port.Speed(bitsPerSecond); }
    uint32_t Speed() override { return m_port.Speed(); }
    int16_t CharGet() override { return m_port.CharGet(); }
    int16_t CharPeek() override { return m_port.CharPeek(); }
    bool SendChar(uint8_t charToSend) override { return m_port.SendChar(charToSend); }
    int32_t AvailableForRead() override { return m_port.AvailableForRead(); }
    int32_t AvailableForWrite() override { return m_port.AvailableForWrite(); }
    void WaitForTransmitIdle() override { m_port.WaitForTransmitIdle(); }
    bool Parity(Parities newParity) override { return m_port.Parity(newParity); }
    Parities Parity() override { return m_port.Parity(); }
    bool StopBits(uint8_t bits) override { return m_port.StopBits(bits); }
    bool CharSize(uint8_t size) override { return m_port.CharSize(size); }

private:
    ClearCore::SerialUsb& m_port;
};

}  // namespace Cutter

#endif  // CUTTER_PLATFORM_CLEARCORE

#endif  // CUTTER_USB_SERIAL_H
