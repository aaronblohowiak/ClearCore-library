/**
 * @file CommandParser.h
 * @brief Command line parser for Cutter protocol
 *
 * Parses newline-delimited commands in the format:
 *   command_name [epoch=N] [seq=N] [key=value ...]
 *
 * Examples:
 *   ping
 *   move motor=0 steps=1000
 *   move epoch=0 seq=42 motor=0 steps=-500
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "CutterConfig.h"

namespace Cutter {

/**
 * @brief A single key=value parameter
 */
struct Param {
    char key[MAX_KEY_LENGTH];
    char value[MAX_VALUE_LENGTH];
};

/**
 * @brief Parsed command structure
 *
 * Contains the command name, parameters, and optional epoch/seq.
 * All storage is inline (no pointers to external data).
 */
struct ParsedCommand {
    /// Command name (e.g., "ping", "move", "configure_digital_in")
    char name[MAX_KEY_LENGTH];

    /// Array of key=value parameters
    Param params[MAX_PARAMS];

    /// Number of parameters parsed
    size_t param_count;

    /// Whether epoch was provided
    bool has_epoch;

    /// Whether seq was provided
    bool has_seq;

    /// Epoch value (if has_epoch is true)
    uint32_t epoch;

    /// Sequence number (if has_seq is true)
    uint32_t seq;

    /**
     * @brief Get an integer parameter value
     * @param key Parameter name to look up
     * @param out Output value
     * @return true if found and parsed successfully
     */
    bool GetInt(const char* key, int32_t* out) const;

    /**
     * @brief Get an unsigned integer parameter value
     * @param key Parameter name to look up
     * @param out Output value
     * @return true if found and parsed successfully
     */
    bool GetUInt(const char* key, uint32_t* out) const;

    /**
     * @brief Get a boolean parameter value
     * @param key Parameter name to look up
     * @param out Output value
     * @return true if found and parsed successfully
     *
     * Accepts: 0, 1, true, false (case insensitive)
     */
    bool GetBool(const char* key, bool* out) const;

    /**
     * @brief Get a string parameter value
     * @param key Parameter name to look up
     * @return Pointer to value string, or nullptr if not found
     */
    const char* GetString(const char* key) const;

    /**
     * @brief Check if a parameter exists
     * @param key Parameter name to look up
     * @return true if the parameter exists
     */
    bool HasParam(const char* key) const;

    /**
     * @brief Get an integer with default value
     * @param key Parameter name to look up
     * @param default_value Value to return if not found
     * @return Parameter value or default
     */
    int32_t GetIntOr(const char* key, int32_t default_value) const;

    /**
     * @brief Get a boolean with default value
     * @param key Parameter name to look up
     * @param default_value Value to return if not found
     * @return Parameter value or default
     */
    bool GetBoolOr(const char* key, bool default_value) const;
};

/**
 * @brief Command line parser
 *
 * Parses a single line into a ParsedCommand structure.
 * Handles comments, whitespace, and key=value extraction.
 */
class CommandParser {
public:
    /**
     * @brief Parse a command line
     * @param line Input line (null-terminated, may include newline)
     * @param cmd Output parsed command
     * @return true if a valid command was parsed, false if empty/comment
     *
     * Empty lines and lines starting with '#' (comments) return false
     * but are not errors.
     */
    bool Parse(const char* line, ParsedCommand* cmd);

private:
    /**
     * @brief Skip whitespace in input
     * @param p Pointer to current position
     * @return Pointer to first non-whitespace character
     */
    static const char* SkipWhitespace(const char* p);

    /**
     * @brief Read a token (non-whitespace sequence)
     * @param p Input pointer
     * @param buf Output buffer
     * @param buf_size Buffer size
     * @return Pointer to position after token, or nullptr on error
     */
    static const char* ReadToken(const char* p, char* buf, size_t buf_size);

    /**
     * @brief Parse a key=value pair
     * @param token Token containing "key=value"
     * @param key Output key buffer
     * @param key_size Key buffer size
     * @param value Output value buffer
     * @param value_size Value buffer size
     * @return true if parsed successfully
     */
    static bool ParseKeyValue(const char* token,
                              char* key, size_t key_size,
                              char* value, size_t value_size);
};

}  // namespace Cutter
