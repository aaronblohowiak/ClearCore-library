/**
 * @file ResponseWriter.cpp
 * @brief Response formatting implementation
 */

#include "CutterResponse.h"
#include <cstring>
#include <cstdio>

namespace Cutter {

ResponseWriter::ResponseWriter(char* buffer, size_t buffer_size)
    : m_buffer(buffer)
    , m_bufferSize(buffer_size)
    , m_pos(0)
    , m_overflowed(false)
    , m_finished(false)
{
    if (m_bufferSize > 0) {
        m_buffer[0] = '\0';
    }
}

void ResponseWriter::Reset() {
    m_pos = 0;
    m_overflowed = false;
    m_finished = false;
    if (m_bufferSize > 0) {
        m_buffer[0] = '\0';
    }
}

void ResponseWriter::Append(const char* str) {
    if (m_overflowed || !str) return;

    while (*str) {
        if (m_pos < m_bufferSize - 1) {
            m_buffer[m_pos++] = *str++;
        } else {
            m_overflowed = true;
            break;
        }
    }
    m_buffer[m_pos] = '\0';
}

void ResponseWriter::AppendChar(char c) {
    if (m_overflowed) return;

    if (m_pos < m_bufferSize - 1) {
        m_buffer[m_pos++] = c;
        m_buffer[m_pos] = '\0';
    } else {
        m_overflowed = true;
    }
}

void ResponseWriter::AppendInt(int32_t value) {
    char temp[16];
    snprintf(temp, sizeof(temp), "%d", static_cast<int>(value));
    Append(temp);
}

void ResponseWriter::AppendUInt(uint32_t value) {
    char temp[16];
    snprintf(temp, sizeof(temp), "%u", static_cast<unsigned>(value));
    Append(temp);
}

ResponseWriter& ResponseWriter::Ok() {
    Reset();
    Append("ok");
    return *this;
}

ResponseWriter& ResponseWriter::Error(uint32_t code, const char* message) {
    Reset();
    Append("error code=");
    AppendUInt(code);
    Append(" message=\"");
    Append(message);
    AppendChar('"');
    return *this;
}

ResponseWriter& ResponseWriter::Event(const char* type) {
    Reset();
    Append("event type=");
    Append(type);
    return *this;
}

ResponseWriter& ResponseWriter::Debug(const char* message) {
    Reset();
    Append("debug message=\"");
    Append(message);
    AppendChar('"');
    return *this;
}

ResponseWriter& ResponseWriter::Param(const char* key, const char* value) {
    AppendChar(' ');
    Append(key);
    AppendChar('=');

    // Check if value needs quoting (contains spaces or special chars)
    bool needs_quote = false;
    for (const char* p = value; *p; p++) {
        if (*p == ' ' || *p == '"' || *p == '\n' || *p == '\r') {
            needs_quote = true;
            break;
        }
    }

    if (needs_quote) {
        AppendChar('"');
        Append(value);
        AppendChar('"');
    } else {
        Append(value);
    }

    return *this;
}

ResponseWriter& ResponseWriter::Param(const char* key, int32_t value) {
    AppendChar(' ');
    Append(key);
    AppendChar('=');
    AppendInt(value);
    return *this;
}

ResponseWriter& ResponseWriter::Param(const char* key, uint32_t value) {
    AppendChar(' ');
    Append(key);
    AppendChar('=');
    AppendUInt(value);
    return *this;
}

ResponseWriter& ResponseWriter::Param(const char* key, bool value) {
    AppendChar(' ');
    Append(key);
    AppendChar('=');
    AppendChar(value ? '1' : '0');
    return *this;
}

const char* ResponseWriter::Finish() {
    if (!m_finished) {
        AppendChar('\n');
        m_finished = true;
    }
    return m_buffer;
}

}  // namespace Cutter
