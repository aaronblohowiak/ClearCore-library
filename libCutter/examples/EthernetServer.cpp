/**
 * @file EthernetServer.cpp
 * @brief Ethernet TCP server example for Cutter
 *
 * This example demonstrates how to use the Cutter library with
 * Ethernet TCP communication on ClearCore.
 */

#include "ClearCore.h"
#include "EthernetTcpServer.h"
#include "Cutter.h"

// Network configuration
static const uint8_t IP_ADDR[4] = {192, 168, 1, 100};
static const uint8_t SUBNET[4] = {255, 255, 255, 0};
static const uint8_t GATEWAY[4] = {192, 168, 1, 1};
static const uint16_t TCP_PORT = 5000;

// Ethernet client wrapper implementing ISerial
class EthernetClientSerial : public Cutter::ISerial {
public:
    void SetClient(EthernetTcpClient* client) {
        client_ = client;
    }

    bool HasClient() const {
        return client_ != nullptr && client_->Connected();
    }

    int16_t CharGet() override {
        if (!client_) return -1;
        return client_->CharGet();
    }

    int16_t CharPeek() override {
        if (!client_) return -1;
        return client_->CharPeek();
    }

    int AvailableForRead() override {
        if (!client_) return 0;
        return client_->BytesAvailable();
    }

    bool SendChar(char c) override {
        if (!client_) return false;
        return client_->Send(&c, 1);
    }

    bool Send(const char* str) override {
        if (!client_) return false;
        return client_->Send(str);
    }

private:
    EthernetTcpClient* client_ = nullptr;
};

EthernetTcpServer server(TCP_PORT);
EthernetClientSerial clientSerial;
Cutter::Controller* cutter = nullptr;

int main() {
    // Initialize ClearCore
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    // Initialize Ethernet
    EthernetMgr.Setup();
    EthernetMgr.LocalIp(IpAddress(IP_ADDR[0], IP_ADDR[1], IP_ADDR[2], IP_ADDR[3]));
    EthernetMgr.NetmaskIp(IpAddress(SUBNET[0], SUBNET[1], SUBNET[2], SUBNET[3]));
    EthernetMgr.GatewayIp(IpAddress(GATEWAY[0], GATEWAY[1], GATEWAY[2], GATEWAY[3]));

    // Wait for link
    while (!EthernetMgr.PhyLinkActive()) {
        Delay_ms(100);
    }

    // Start server
    server.Begin();

    // Create controller
    cutter = new Cutter::Controller(&clientSerial);

    // Main loop
    while (true) {
        // Accept new connections
        EthernetTcpClient client = server.Available();
        if (client.Connected()) {
            clientSerial.SetClient(&client);

            // Handle this client until disconnected
            while (client.Connected()) {
                EthernetMgr.Refresh();
                cutter->Update();
            }

            clientSerial.SetClient(nullptr);
        }

        EthernetMgr.Refresh();
    }

    return 0;
}
