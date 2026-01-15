/**
 * @file CommandParser.cpp
 * @brief Command line parsing implementation
 */

#include "CommandParser.h"
#include <cstring>
#include <cstdlib>
#include <cctype>

namespace Cutter {

// === ParsedCommand methods ===

bool ParsedCommand::GetInt(const char* key, int32_t* out) const {
    const char* val = GetString(key);
    if (!val) return false;

    char* end;
    long result = strtol(val, &end, 10);
    if (end == val || *end != '\0') return false;

    *out = static_cast<int32_t>(result);
    return true;
}

bool ParsedCommand::GetUInt(const char* key, uint32_t* out) const {
    const char* val = GetString(key);
    if (!val) return false;

    // Handle negative values as invalid for unsigned
    if (val[0] == '-') return false;

    char* end;
    unsigned long result = strtoul(val, &end, 10);
    if (end == val || *end != '\0') return false;

    *out = static_cast<uint32_t>(result);
    return true;
}

bool ParsedCommand::GetBool(const char* key, bool* out) const {
    const char* val = GetString(key);
    if (!val) return false;

    // Check for "1", "true", "0", "false"
    if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0) {
        *out = true;
        return true;
    }
    if (strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0) {
        *out = false;
        return true;
    }

    return false;
}

const char* ParsedCommand::GetString(const char* key) const {
    for (size_t i = 0; i < param_count; i++) {
        if (strcmp(params[i].key, key) == 0) {
            return params[i].value;
        }
    }
    return nullptr;
}

bool ParsedCommand::HasParam(const char* key) const {
    return GetString(key) != nullptr;
}

int32_t ParsedCommand::GetIntOr(const char* key, int32_t default_value) const {
    int32_t val;
    if (GetInt(key, &val)) {
        return val;
    }
    return default_value;
}

bool ParsedCommand::GetBoolOr(const char* key, bool default_value) const {
    bool val;
    if (GetBool(key, &val)) {
        return val;
    }
    return default_value;
}

// === CommandParser methods ===

const char* CommandParser::SkipWhitespace(const char* p) {
    while (*p && (*p == ' ' || *p == '\t')) {
        p++;
    }
    return p;
}

const char* CommandParser::ReadToken(const char* p, char* buf, size_t buf_size) {
    size_t i = 0;
    bool in_quotes = false;

    while (*p) {
        // Check for entering/exiting quotes
        if (*p == '"') {
            in_quotes = !in_quotes;
        }

        // Stop at whitespace unless inside quotes
        if (!in_quotes && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) {
            break;
        }

        if (i < buf_size - 1) {
            buf[i++] = *p;
        }
        p++;
    }
    buf[i] = '\0';
    return p;
}

bool CommandParser::ParseKeyValue(const char* token,
                                   char* key, size_t key_size,
                                   char* value, size_t value_size) {
    const char* eq = strchr(token, '=');
    if (!eq) return false;

    // Extract key
    size_t key_len = static_cast<size_t>(eq - token);
    if (key_len == 0 || key_len >= key_size) return false;
    memcpy(key, token, key_len);
    key[key_len] = '\0';

    // Extract value (everything after '=')
    const char* val_start = eq + 1;
    size_t val_len = strlen(val_start);

    // Handle quoted values
    if (val_len >= 2 && val_start[0] == '"' && val_start[val_len - 1] == '"') {
        // Strip quotes
        val_start++;
        val_len -= 2;
    }

    if (val_len >= value_size) return false;
    memcpy(value, val_start, val_len);
    value[val_len] = '\0';

    return true;
}

bool CommandParser::Parse(const char* line, ParsedCommand* cmd) {
    // Initialize output
    memset(cmd, 0, sizeof(*cmd));

    // Skip leading whitespace
    const char* p = SkipWhitespace(line);

    // Empty line or comment
    if (*p == '\0' || *p == '\n' || *p == '\r' || *p == '#') {
        return false;
    }

    // Read command name
    char token[MAX_COMMAND_LENGTH];
    p = ReadToken(p, token, sizeof(token));
    if (token[0] == '\0') {
        return false;
    }

    // Check if first token is a key=value (invalid - command name required)
    if (strchr(token, '=')) {
        return false;
    }

    // Copy command name
    strncpy(cmd->name, token, sizeof(cmd->name) - 1);
    cmd->name[sizeof(cmd->name) - 1] = '\0';

    // Parse remaining tokens as key=value pairs
    while (true) {
        p = SkipWhitespace(p);

        // End of line
        if (*p == '\0' || *p == '\n' || *p == '\r' || *p == '#') {
            break;
        }

        // Read next token
        p = ReadToken(p, token, sizeof(token));
        if (token[0] == '\0') {
            break;
        }

        // Parse as key=value
        char key[MAX_KEY_LENGTH];
        char value[MAX_VALUE_LENGTH];
        if (!ParseKeyValue(token, key, sizeof(key), value, sizeof(value))) {
            // Invalid key=value format - skip or error?
            // For now, skip invalid tokens
            continue;
        }

        // Check for special keys: epoch and seq
        if (strcmp(key, "epoch") == 0) {
            cmd->epoch = static_cast<uint32_t>(strtoul(value, nullptr, 10));
            cmd->has_epoch = true;
            continue;
        }
        if (strcmp(key, "seq") == 0) {
            cmd->seq = static_cast<uint32_t>(strtoul(value, nullptr, 10));
            cmd->has_seq = true;
            continue;
        }

        // Regular parameter - add to array if space available
        if (cmd->param_count < MAX_PARAMS) {
            strncpy(cmd->params[cmd->param_count].key, key, MAX_KEY_LENGTH - 1);
            cmd->params[cmd->param_count].key[MAX_KEY_LENGTH - 1] = '\0';
            strncpy(cmd->params[cmd->param_count].value, value, MAX_VALUE_LENGTH - 1);
            cmd->params[cmd->param_count].value[MAX_VALUE_LENGTH - 1] = '\0';
            cmd->param_count++;
        }
        // If too many params, silently ignore extras
    }

    return true;
}

}  // namespace Cutter
