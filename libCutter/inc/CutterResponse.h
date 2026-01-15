/**
 * @file CutterResponse.h
 * @brief Response formatting for Cutter protocol
 *
 * Formats responses in the format:
 *   ok [key=value ...]
 *   error code=N message="..."
 *   event type=T [key=value ...]
 *   debug message="..."
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "CutterConfig.h"

namespace Cutter {

/**
 * @brief Response builder with fixed buffer
 *
 * Builds response strings without dynamic allocation.
 * All Param() calls are chained and return *this.
 *
 * Usage:
 *   char buf[256];
 *   ResponseWriter w(buf, sizeof(buf));
 *   w.Ok().Param("pin", 6).Param("value", 1);
 *   // buf now contains: "ok pin=6 value=1\n"
 */
class ResponseWriter {
public:
    /**
     * @brief Construct a response writer
     * @param buffer Output buffer
     * @param buffer_size Size of output buffer
     */
    ResponseWriter(char* buffer, size_t buffer_size);

    /**
     * @brief Reset the writer to empty state
     */
    void Reset();

    /**
     * @brief Start an "ok" response
     * @return Reference to this for chaining
     */
    ResponseWriter& Ok();

    /**
     * @brief Start an "error" response
     * @param code Error code
     * @param message Error message
     * @return Reference to this for chaining
     */
    ResponseWriter& Error(uint32_t code, const char* message);

    /**
     * @brief Start an "event" response
     * @param type Event type (e.g., "done", "input", "threshold")
     * @return Reference to this for chaining
     */
    ResponseWriter& Event(const char* type);

    /**
     * @brief Start a "debug" response
     * @param message Debug message
     * @return Reference to this for chaining
     */
    ResponseWriter& Debug(const char* message);

    /**
     * @brief Add a string parameter
     * @param key Parameter name
     * @param value Parameter value
     * @return Reference to this for chaining
     */
    ResponseWriter& Param(const char* key, const char* value);

    /**
     * @brief Add an integer parameter
     * @param key Parameter name
     * @param value Parameter value
     * @return Reference to this for chaining
     */
    ResponseWriter& Param(const char* key, int32_t value);

    /**
     * @brief Add an unsigned integer parameter
     * @param key Parameter name
     * @param value Parameter value
     * @return Reference to this for chaining
     */
    ResponseWriter& Param(const char* key, uint32_t value);

    /**
     * @brief Add a boolean parameter
     * @param key Parameter name
     * @param value Parameter value (will be "1" or "0")
     * @return Reference to this for chaining
     */
    ResponseWriter& Param(const char* key, bool value);

    /**
     * @brief Finalize the response (adds newline)
     * @return Pointer to the response string
     *
     * Must be called after building response to get the final string.
     * Adds newline terminator if not already present.
     */
    const char* Finish();

    /**
     * @brief Get current buffer contents (without finalizing)
     * @return Pointer to the response string
     */
    const char* Get() const { return buffer_; }

    /**
     * @brief Get current length of response
     * @return Number of characters written (excluding null)
     */
    size_t Length() const { return pos_; }

    /**
     * @brief Check if buffer overflowed during construction
     * @return true if overflow occurred
     */
    bool Overflowed() const { return overflowed_; }

private:
    char* buffer_;
    size_t buffer_size_;
    size_t pos_;
    bool overflowed_;
    bool finished_;

    /**
     * @brief Append a string to the buffer
     * @param str String to append
     */
    void Append(const char* str);

    /**
     * @brief Append a character to the buffer
     * @param c Character to append
     */
    void AppendChar(char c);

    /**
     * @brief Append a formatted integer
     * @param value Integer value
     */
    void AppendInt(int32_t value);

    /**
     * @brief Append a formatted unsigned integer
     * @param value Unsigned integer value
     */
    void AppendUInt(uint32_t value);
};

}  // namespace Cutter
