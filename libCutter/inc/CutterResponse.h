/**
 * \file CutterResponse.h
 * \brief Response formatting for Cutter protocol
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
    \brief Command identifier for correlating async events to commands

    Stores epoch and seq pair assigned internally by the controller.
    Every command is assigned a unique seq (incrementing counter) and the
    current epoch. User-supplied epoch/seq in commands are for verification
    only (sync checking), not for identifying the command.

    \par Usage
    CommandId is stored when initiating async operations:
    - Motor moves: Stored in MotorSlot::move_id, emitted in "done" and "soft_limit" events
    - Motor enable: Stored in MotorSlot::enable_id, emitted in "hlfb_timeout" error
    - Pin timeout: Stored in DigitalOutState::set_id, emitted in "pin_timeout" event

    \par Event Correlation
    Host sends: `move motor=0 steps=1000`
    Cutter responds: `ok epoch=0 seq=5 motor=0`
    Later: `event type=done epoch=0 seq=5 motor=0 position=1000`
    This allows the host to match async events back to the originating command.

    \note Every ok/error/event carries the id directly after the verb. Pass a
    CommandId to Ok()/Error()/Event() rather than appending epoch/seq by hand.
**/
struct CommandId {
    uint32_t epoch;
    uint32_t seq;
};

/**
 * \brief Response builder with fixed buffer
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
     * \brief Construct a response writer
     * \param buffer Output buffer
     * \param buffer_size Size of output buffer
     */
    ResponseWriter(char* buffer, size_t buffer_size);

    /**
     * \brief Reset the writer to empty state
     */
    void Reset();

    /**
     * \brief Start an "ok" response
     * \return Reference to this for chaining
     */
    ResponseWriter& Ok();

    /**
     * \brief Start an "ok" response carrying a command id
     * \param id Command id emitted as epoch/seq directly after "ok"
     * \return Reference to this for chaining
     *
     * Equivalent to Ok().Param("epoch", id.epoch).Param("seq", id.seq).
     */
    ResponseWriter& Ok(const CommandId& id);

    /**
     * \brief Start an "error" response
     * \param code Error code
     * \param message Error message
     * \return Reference to this for chaining
     */
    ResponseWriter& Error(uint32_t code, const char* message);

    /**
     * \brief Start an "error" response carrying a command id
     * \param code Error code
     * \param message Error message
     * \param id Command id emitted as epoch/seq directly after the message
     * \return Reference to this for chaining
     */
    ResponseWriter& Error(uint32_t code, const char* message, const CommandId& id);

    /**
     * \brief Start an "event" response
     * \param type Event type (e.g., "done", "input", "threshold")
     * \return Reference to this for chaining
     */
    ResponseWriter& Event(const char* type);

    /**
     * \brief Start an "event" response carrying a command id
     * \param type Event type (e.g., "done", "input", "threshold")
     * \param id Command id emitted as epoch/seq directly after the type
     * \return Reference to this for chaining
     */
    ResponseWriter& Event(const char* type, const CommandId& id);

    /**
     * \brief Start a "status" response (solicited query payload)
     * \param type Status row type (e.g., "pin", "motor", "summary")
     * \return Reference to this for chaining
     *
     * Distinct from Event: status rows are the body of a query response
     * (e.g. get_status), not an unsolicited asynchronous notification.
     */
    ResponseWriter& Status(const char* type);

    /**
     * \brief Start a "status" response carrying a command id
     * \param type Status row type (e.g., "pin", "motor", "summary")
     * \param id Command id emitted as epoch/seq directly after the type
     * \return Reference to this for chaining
     */
    ResponseWriter& Status(const char* type, const CommandId& id);

    /**
     * \brief Start a "debug" response
     * \param message Debug message
     * \return Reference to this for chaining
     */
    ResponseWriter& Debug(const char* message);

    /**
     * \brief Add a string parameter
     * \param key Parameter name
     * \param value Parameter value
     * \return Reference to this for chaining
     */
    ResponseWriter& Param(const char* key, const char* value);

    /**
     * \brief Add an integer parameter
     * \param key Parameter name
     * \param value Parameter value
     * \return Reference to this for chaining
     */
    ResponseWriter& Param(const char* key, int32_t value);

    /**
     * \brief Add an unsigned integer parameter
     * \param key Parameter name
     * \param value Parameter value
     * \return Reference to this for chaining
     */
    ResponseWriter& Param(const char* key, uint32_t value);

    /**
     * \brief Add a boolean parameter
     * \param key Parameter name
     * \param value Parameter value (will be "1" or "0")
     * \return Reference to this for chaining
     */
    ResponseWriter& Param(const char* key, bool value);

    /**
     * \brief Finalize the response (adds newline)
     * \return Pointer to the response string
     *
     * Must be called after building response to get the final string.
     * Adds newline terminator if not already present.
     */
    const char* Finish();

    /**
     * \brief Get current buffer contents (without finalizing)
     * \return Pointer to the response string
     */
    const char* Get() const { return m_buffer; }

    /**
     * \brief Get current length of response
     * \return Number of characters written (excluding null)
     */
    size_t Length() const { return m_pos; }

    /**
     * \brief Check if buffer overflowed during construction
     * \return true if overflow occurred
     */
    bool Overflowed() const { return m_overflowed; }

private:
    char* m_buffer;
    size_t m_bufferSize;
    size_t m_pos;
    bool m_overflowed;
    bool m_finished;

    /**
     * \brief Append a string to the buffer
     * \param str String to append
     */
    void Append(const char* str);

    /**
     * \brief Append a character to the buffer
     * \param c Character to append
     */
    void AppendChar(char c);

    /**
     * \brief Append a formatted integer
     * \param value Integer value
     */
    void AppendInt(int32_t value);

    /**
     * \brief Append a formatted unsigned integer
     * \param value Unsigned integer value
     */
    void AppendUInt(uint32_t value);
};

}  // namespace Cutter
