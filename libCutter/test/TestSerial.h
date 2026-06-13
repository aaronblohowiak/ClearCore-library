/**
 * @file TestSerial.h
 * @brief Fake serial port for testing
 *
 * Provides an ISerial-like interface that tests can use to send commands
 * and inspect responses without real hardware.
 */

#pragma once

#include <string>
#include <vector>
#include <cstring>
#include "Cutter.h"

/**
 * @brief Fake serial port for testing Cutter
 *
 * Usage:
 *   TestSerial serial;
 *   serial.SendLine("ping");
 *   cutter.Update();
 *   EXPECT_EQ(serial.LastResponse(), "ok");
 */
class TestSerial : public Cutter::ISerial {
public:
    ~TestSerial() override = default;

    /**
     * @brief Send a command line (as if host sent it)
     * @param line Command string (newline will be appended)
     */
    void SendLine(const char* line) {
        input_buffer_ += line;
        input_buffer_ += "\n";
    }

    /**
     * @brief Send raw data (without adding newline)
     * @param data Data to send
     */
    void SendRaw(const char* data) {
        input_buffer_ += data;
    }

    // === ISerial interface implementation ===

    /**
     * @brief Get next character from input buffer
     * @return Character or -1 if buffer empty
     */
    int16_t CharGet() override {
        if (input_pos_ >= input_buffer_.size()) {
            return -1;
        }
        return static_cast<uint8_t>(input_buffer_[input_pos_++]);
    }

    /**
     * @brief Peek at next character without consuming
     * @return Character or -1 if buffer empty
     */
    int16_t CharPeek() override {
        if (input_pos_ >= input_buffer_.size()) {
            return -1;
        }
        return static_cast<uint8_t>(input_buffer_[input_pos_]);
    }

    /**
     * @brief Check if input data is available
     * @return Number of bytes available
     */
    int32_t AvailableForRead() override {
        return static_cast<int32_t>(input_buffer_.size() - input_pos_);
    }

    /**
     * @brief Send a character to output
     * @param c Character to send
     * @return true (always succeeds in test)
     */
    bool SendChar(uint8_t c) override {
        output_buffer_ += static_cast<char>(c);
        return true;
    }

    /**
     * @brief Send a string to output
     * @param str String to send
     * @return true (always succeeds in test)
     */
    bool Send(const char* str) override {
        output_buffer_ += str;
        return true;
    }

    /**
     * @brief Whether the host has the port open (USB DTR on real hardware)
     * @return Current simulated port-open state
     */
    bool PortIsOpen() override {
        return port_open_;
    }

    // === Test inspection methods ===

    /**
     * @brief Simulate the host opening/closing the serial port
     * @param open New port-open state
     */
    void SetPortOpen(bool open) {
        port_open_ = open;
    }

    /**
     * @brief Get all output as a single string
     * @return Complete output buffer
     */
    std::string GetOutput() const {
        return output_buffer_;
    }

    /**
     * @brief Get output split into lines
     * @return Vector of output lines (newlines stripped)
     */
    std::vector<std::string> GetOutputLines() const {
        std::vector<std::string> lines;
        size_t start = 0;
        for (size_t i = 0; i < output_buffer_.size(); i++) {
            if (output_buffer_[i] == '\n') {
                lines.push_back(output_buffer_.substr(start, i - start));
                start = i + 1;
            }
        }
        // Add any remaining content without newline
        if (start < output_buffer_.size()) {
            lines.push_back(output_buffer_.substr(start));
        }
        return lines;
    }

    /**
     * @brief Get the last "ok" or "error" response
     * @return Last response line, or empty string if none
     */
    std::string LastResponse() const {
        auto lines = GetOutputLines();
        for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
            if (it->size() >= 2 && it->substr(0, 2) == "ok") {
                return *it;
            }
            if (it->size() >= 5 && it->substr(0, 5) == "error") {
                return *it;
            }
        }
        return "";
    }

    /**
     * @brief Get the last event line
     * @return Last event line, or empty string if none
     */
    std::string LastEvent() const {
        auto lines = GetOutputLines();
        for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
            if (it->size() >= 5 && it->substr(0, 5) == "event") {
                return *it;
            }
        }
        return "";
    }

    /**
     * @brief Check if any output line contains a substring
     * @param substr Substring to search for
     * @return true if found in any output line
     */
    bool HasOutput(const std::string& substr) const {
        return output_buffer_.find(substr) != std::string::npos;
    }

    /**
     * @brief Check if any event line contains a substring
     * @param substr Substring to search for
     * @return true if found in an event line
     */
    bool HasEvent(const std::string& substr) const {
        for (const auto& line : GetOutputLines()) {
            if (line.size() >= 5 && line.substr(0, 5) == "event") {
                if (line.find(substr) != std::string::npos) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief Clear input and output buffers
     */
    void Clear() {
        input_buffer_.clear();
        output_buffer_.clear();
        input_pos_ = 0;
    }

    /**
     * @brief Clear only the output buffer (keep pending input)
     */
    void ClearOutput() {
        output_buffer_.clear();
    }

private:
    std::string input_buffer_;
    std::string output_buffer_;
    size_t input_pos_ = 0;
    bool port_open_ = true;  ///< Simulated host port-open (DTR) state; open by default
};
