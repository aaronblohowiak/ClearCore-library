/**
 * @file EthernetClient.cpp
 * @brief Ethernet TCP client example for Cutter
 *
 * This example demonstrates how to use Cutter with Ethernet communication
 * where ClearCore connects OUT to a host controller as a TCP client.
 *
 * Configuration:
 *   - ClearCore uses DHCP to obtain IP address
 *   - ClearCore connects to a configured host IP and port
 *   - Host runs a TCP server that sends commands and receives responses
 *
 * This is the recommended pattern for Ethernet-based control:
 *   - Single point of control (no multiple connections)
 *   - ClearCore initiates connection (easier firewall/network config)
 *   - Automatic reconnection on disconnect
 */

#include "ClearCore.h"
#include "EthernetManager.h"
#include "EthernetTcpClient.h"
#include "Cutter.h"

// === Configuration ===
// Host controller to connect to (the machine running your control software)
static const uint8_t HOST_IP[4] = {192, 168, 1, 10};
static const uint16_t HOST_PORT = 5000;

// Reconnection settings
static const uint32_t RECONNECT_INTERVAL_MS = 5000;
static const uint32_t DHCP_TIMEOUT_MS = 30000;

// === Ethernet TCP Client as ISerial ===
// EthernetTcpClient doesn't inherit from ISerial, so we wrap it
class TcpClientSerial : public ClearCore::ISerial {
public:
    void SetClient(EthernetTcpClient* client) {
        client_ = client;
    }

    bool IsConnected() const {
        return client_ != nullptr && client_->Connected();
    }

    // === ISerial interface ===

    void Flush() override {
        if (client_) client_->Flush();
    }

    void FlushInput() override {
        if (client_) client_->FlushInput();
    }

    void PortOpen() override {
        // Connection managed externally
    }

    void PortClose() override {
        if (client_) client_->Close();
    }

    bool Speed(uint32_t) override { return true; }
    uint32_t Speed() override { return 0; }

    int16_t CharGet() override {
        if (!client_) return -1;
        return client_->CharGet();
    }

    int16_t CharPeek() override {
        if (!client_) return -1;
        return client_->CharPeek();
    }

    bool SendChar(uint8_t c) override {
        if (!client_) return false;
        char buf[1] = {static_cast<char>(c)};
        return client_->Send(buf, 1);
    }

    int32_t AvailableForRead() override {
        if (!client_) return 0;
        return client_->BytesAvailable();
    }

    int32_t AvailableForWrite() override {
        return client_ ? 1024 : 0;  // Approximate
    }

    void WaitForTransmitIdle() override {
        if (client_) client_->FlushInput();
    }

    bool PortIsOpen() override {
        return IsConnected();
    }

    operator bool() override {
        return IsConnected();
    }

    bool Parity(Parities) override { return true; }
    Parities Parity() override { return PARITY_N; }
    bool StopBits(uint8_t) override { return true; }
    bool CharSize(uint8_t) override { return true; }

private:
    EthernetTcpClient* client_ = nullptr;
};

// === Global state ===
EthernetTcpClient tcpClient;
TcpClientSerial clientSerial;
Cutter::Controller* cutter = nullptr;

uint32_t lastConnectAttempt = 0;
bool dhcpConfigured = false;

// === Helper functions ===

bool WaitForDhcp() {
    uint32_t startTime = Milliseconds();

    // Start DHCP
    if (!EthernetMgr.DhcpBegin()) {
        return false;
    }

    // Wait for DHCP to complete
    while (!EthernetMgr.DhcpActive()) {
        EthernetMgr.Refresh();

        if (Milliseconds() - startTime > DHCP_TIMEOUT_MS) {
            return false;
        }

        Delay_ms(100);
    }

    return true;
}

bool TryConnect() {
    IpAddress hostIp(HOST_IP[0], HOST_IP[1], HOST_IP[2], HOST_IP[3]);
    return tcpClient.Connect(hostIp, HOST_PORT);
}

// === Main ===

int main() {
    // Initialize ClearCore motors
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    // Initialize Ethernet hardware
    EthernetMgr.Setup();

    // Wait for physical link
    while (!EthernetMgr.PhyLinkActive()) {
        Delay_ms(100);
    }

    // Configure via DHCP
    if (!WaitForDhcp()) {
        // DHCP failed - could fall back to static IP here
        // For now, just keep trying
        while (!WaitForDhcp()) {
            Delay_ms(5000);
        }
    }
    dhcpConfigured = true;

    // Create Cutter controller with TCP client serial
    clientSerial.SetClient(&tcpClient);
    cutter = new Cutter::Controller(&clientSerial);

    // Main loop
    while (true) {
        // Refresh Ethernet stack
        EthernetMgr.Refresh();

        // Connection state machine
        if (!tcpClient.Connected()) {
            // Not connected - try to connect periodically
            uint32_t now = Milliseconds();
            if (now - lastConnectAttempt >= RECONNECT_INTERVAL_MS) {
                lastConnectAttempt = now;
                TryConnect();
            }
        } else {
            // Connected - process Cutter commands
            cutter->Update();
        }
    }

    return 0;
}
