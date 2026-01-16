/**
 * @file test_schema.cpp
 * @brief Tests that validate implementation matches protocol schema
 *
 * These tests ensure the cutter_protocol.json schema stays in sync with
 * the actual implementation. They verify:
 * - All schema commands are recognized
 * - Required params are enforced
 * - Type/range validation works
 * - State requirements are honored
 */

#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include "Cutter.h"
#include "FakeHal.h"
#include "TestSerial.h"

// Simple JSON value types for our parser
enum class JsonType { Null, Bool, Number, String, Array, Object };

// Minimal JSON parser for test purposes
// (In production, you'd use nlohmann/json or similar)
class JsonValue {
public:
    JsonType type = JsonType::Null;
    std::string str_val;
    double num_val = 0;
    bool bool_val = false;
    std::vector<JsonValue> arr_val;
    std::map<std::string, JsonValue> obj_val;

    bool is_object() const { return type == JsonType::Object; }
    bool is_array() const { return type == JsonType::Array; }
    bool is_string() const { return type == JsonType::String; }
    bool is_number() const { return type == JsonType::Number; }
    bool is_bool() const { return type == JsonType::Bool; }

    const JsonValue& operator[](const std::string& key) const {
        static JsonValue null_val;
        auto it = obj_val.find(key);
        return it != obj_val.end() ? it->second : null_val;
    }

    const JsonValue& operator[](size_t idx) const {
        static JsonValue null_val;
        return idx < arr_val.size() ? arr_val[idx] : null_val;
    }

    bool has(const std::string& key) const {
        return obj_val.find(key) != obj_val.end();
    }

    size_t size() const {
        if (type == JsonType::Array) return arr_val.size();
        if (type == JsonType::Object) return obj_val.size();
        return 0;
    }
};

// Forward declarations
static JsonValue parse_json(const std::string& json, size_t& pos);
static void skip_whitespace(const std::string& json, size_t& pos);

static std::string parse_string(const std::string& json, size_t& pos) {
    if (json[pos] != '"') return "";
    pos++; // skip opening quote
    std::string result;
    while (pos < json.size() && json[pos] != '"') {
        if (json[pos] == '\\' && pos + 1 < json.size()) {
            pos++;
            switch (json[pos]) {
                case 'n': result += '\n'; break;
                case 't': result += '\t'; break;
                case 'r': result += '\r'; break;
                case '"': result += '"'; break;
                case '\\': result += '\\'; break;
                default: result += json[pos]; break;
            }
        } else {
            result += json[pos];
        }
        pos++;
    }
    pos++; // skip closing quote
    return result;
}

static double parse_number(const std::string& json, size_t& pos) {
    size_t start = pos;
    if (json[pos] == '-') pos++;
    while (pos < json.size() && (isdigit(json[pos]) || json[pos] == '.' || json[pos] == 'e' || json[pos] == 'E' || json[pos] == '+' || json[pos] == '-')) {
        pos++;
    }
    return std::stod(json.substr(start, pos - start));
}

static void skip_whitespace(const std::string& json, size_t& pos) {
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\n' || json[pos] == '\r' || json[pos] == '\t')) {
        pos++;
    }
}

static JsonValue parse_json(const std::string& json, size_t& pos) {
    JsonValue val;
    skip_whitespace(json, pos);

    if (pos >= json.size()) return val;

    char c = json[pos];

    if (c == '"') {
        val.type = JsonType::String;
        val.str_val = parse_string(json, pos);
    } else if (c == '-' || isdigit(c)) {
        val.type = JsonType::Number;
        val.num_val = parse_number(json, pos);
    } else if (c == 't' && json.substr(pos, 4) == "true") {
        val.type = JsonType::Bool;
        val.bool_val = true;
        pos += 4;
    } else if (c == 'f' && json.substr(pos, 5) == "false") {
        val.type = JsonType::Bool;
        val.bool_val = false;
        pos += 5;
    } else if (c == 'n' && json.substr(pos, 4) == "null") {
        val.type = JsonType::Null;
        pos += 4;
    } else if (c == '[') {
        val.type = JsonType::Array;
        pos++; // skip [
        skip_whitespace(json, pos);
        while (pos < json.size() && json[pos] != ']') {
            val.arr_val.push_back(parse_json(json, pos));
            skip_whitespace(json, pos);
            if (json[pos] == ',') pos++;
            skip_whitespace(json, pos);
        }
        pos++; // skip ]
    } else if (c == '{') {
        val.type = JsonType::Object;
        pos++; // skip {
        skip_whitespace(json, pos);
        while (pos < json.size() && json[pos] != '}') {
            std::string key = parse_string(json, pos);
            skip_whitespace(json, pos);
            pos++; // skip :
            skip_whitespace(json, pos);
            val.obj_val[key] = parse_json(json, pos);
            skip_whitespace(json, pos);
            if (json[pos] == ',') pos++;
            skip_whitespace(json, pos);
        }
        pos++; // skip }
    }

    return val;
}

static JsonValue parse_json_file(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        // Try relative paths from different locations
        std::vector<std::string> paths = {
            path,
            "../spec/cutter_protocol.json",
            "../../spec/cutter_protocol.json",
            "spec/cutter_protocol.json",
            "../../../libCutter/spec/cutter_protocol.json"
        };
        for (const auto& p : paths) {
            file.open(p);
            if (file.is_open()) break;
        }
    }
    if (!file.is_open()) {
        JsonValue empty;
        return empty;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string json = buffer.str();
    size_t pos = 0;
    return parse_json(json, pos);
}

using namespace Cutter;

class SchemaTest : public ::testing::Test {
protected:
    TestSerial serial;
    Controller* ctrl;
    JsonValue schema;

    void SetUp() override {
        RESET_HAL();
        ctrl = new Controller(&serial);
        // Connect
        serial.SendLine("ping");
        ctrl->Update();
        serial.ClearOutput();

        // Load schema
        schema = parse_json_file("libCutter/spec/cutter_protocol.json");
    }

    void TearDown() override {
        delete ctrl;
    }

    // Helper to check if command is recognized (not UNKNOWN_COMMAND)
    bool CommandRecognized(const std::string& cmd) {
        serial.Clear();
        serial.SendLine(cmd.c_str());
        ctrl->Update();
        std::string output = serial.GetOutput();
        // Command is recognized if we don't get UNKNOWN_COMMAND (code=100)
        return output.find("code=100") == std::string::npos;
    }

    // Helper to check for specific error code
    bool HasErrorCode(int code) {
        std::string output = serial.GetOutput();
        return output.find("code=" + std::to_string(code)) != std::string::npos;
    }

    // Helper to check for ok response
    bool IsOk() {
        std::string output = serial.GetOutput();
        return output.find("ok") != std::string::npos &&
               output.find("error") == std::string::npos;
    }
};

// Test that schema file loads
TEST_F(SchemaTest, SchemaFileLoads) {
    ASSERT_TRUE(schema.is_object()) << "Schema file not found or invalid";
    EXPECT_TRUE(schema.has("commands"));
    EXPECT_TRUE(schema.has("events"));
    EXPECT_TRUE(schema.has("errors"));
    EXPECT_TRUE(schema.has("primitive_types"));
}

// Test that all commands in schema are recognized
TEST_F(SchemaTest, AllSchemaCommandsRecognized) {
    ASSERT_TRUE(schema.has("commands"));
    const auto& commands = schema["commands"];
    ASSERT_TRUE(commands.is_object());

    for (const auto& [name, _] : commands.obj_val) {
        // Just send command name - may fail for other reasons but shouldn't be UNKNOWN_COMMAND
        EXPECT_TRUE(CommandRecognized(name))
            << "Command '" << name << "' from schema not recognized";
    }
}

// Test that error codes in schema match implementation
TEST_F(SchemaTest, ErrorCodesMatch) {
    ASSERT_TRUE(schema.has("errors"));
    const auto& errors = schema["errors"];

    // Verify key error codes
    EXPECT_TRUE(errors.has("100")); // UNKNOWN_COMMAND
    EXPECT_TRUE(errors.has("102")); // MISSING_PARAM
    EXPECT_TRUE(errors.has("106")); // INPUT_OVERFLOW
    EXPECT_TRUE(errors.has("200")); // INVALID_PIN
    EXPECT_TRUE(errors.has("300")); // INVALID_MOTOR

    // Test that UNKNOWN_COMMAND produces code 100
    serial.Clear();
    serial.SendLine("nonexistent_command_xyz");
    ctrl->Update();
    EXPECT_TRUE(HasErrorCode(100));
}

// Test motor_index type validation (0-3)
TEST_F(SchemaTest, MotorIndexValidation) {
    // Valid motor indices
    for (int i = 0; i <= 3; i++) {
        serial.Clear();
        serial.SendLine(("configure_sdsk motor=" + std::to_string(i)).c_str());
        ctrl->Update();
        EXPECT_TRUE(IsOk()) << "Motor " << i << " should be valid";
    }

    // Invalid motor index
    serial.Clear();
    serial.SendLine("configure_sdsk motor=4");
    ctrl->Update();
    EXPECT_TRUE(HasErrorCode(300)) << "Motor 4 should be invalid";

    serial.Clear();
    serial.SendLine("configure_sdsk motor=-1");
    ctrl->Update();
    EXPECT_TRUE(HasErrorCode(300)) << "Motor -1 should be invalid";
}

// Test pin_index type validation (0-12)
TEST_F(SchemaTest, PinIndexValidation) {
    // Valid pin indices
    for (int i = 0; i <= 12; i++) {
        serial.Clear();
        serial.SendLine(("configure_digital_in pin=" + std::to_string(i)).c_str());
        ctrl->Update();
        EXPECT_TRUE(IsOk()) << "Pin " << i << " should be valid";
    }

    // Invalid pin index
    serial.Clear();
    serial.SendLine("configure_digital_in pin=13");
    ctrl->Update();
    EXPECT_TRUE(HasErrorCode(200)) << "Pin 13 should be invalid";
}

// Test required param enforcement
TEST_F(SchemaTest, RequiredParamsEnforced) {
    // configure_sdsk requires motor param
    serial.Clear();
    serial.SendLine("configure_sdsk");
    ctrl->Update();
    EXPECT_TRUE(HasErrorCode(300) || HasErrorCode(102))
        << "Missing required 'motor' param should error";

    // move requires motor param
    serial.Clear();
    // First configure a motor
    serial.SendLine("configure_sdsk motor=0");
    ctrl->Update();
    serial.SendLine("enable motor=0");
    ctrl->Update();
    SET_MOTOR_READY(0);
    ctrl->Update();
    serial.Clear();

    serial.SendLine("move steps=100");
    ctrl->Update();
    EXPECT_TRUE(HasErrorCode(300) || HasErrorCode(102))
        << "Missing required 'motor' param should error";
}

// Test state requirements from schema
TEST_F(SchemaTest, StateRequirementsHonored) {
    // 'reset' should only work in ERROR state according to schema
    serial.Clear();
    serial.SendLine("reset");
    ctrl->Update();
    // Should fail - we're not in ERROR state
    EXPECT_TRUE(HasErrorCode(103))
        << "'reset' should require ERROR state";

    // Move should require READY or WORKING state
    serial.Clear();
    serial.SendLine("move motor=0 steps=100");
    ctrl->Update();
    // Should fail - motor not configured/enabled
    EXPECT_FALSE(IsOk());
}

// Test that configure_digital_out respects pin capability (0-5 only)
TEST_F(SchemaTest, PinCapabilityEnforced) {
    // Pins 0-5 can be digital out
    for (int i = 0; i <= 5; i++) {
        serial.Clear();
        serial.SendLine(("configure_digital_out pin=" + std::to_string(i)).c_str());
        ctrl->Update();
        EXPECT_TRUE(IsOk()) << "Pin " << i << " should support digital out";
    }

    // Pins 6-12 cannot be digital out
    serial.Clear();
    serial.SendLine("configure_digital_out pin=6");
    ctrl->Update();
    EXPECT_TRUE(HasErrorCode(202))
        << "Pin 6 should not support digital output";
}

// Test that H-Bridge pins are limited (4-5 only)
TEST_F(SchemaTest, HBridgePinLimits) {
    // Pins 4-5 support H-Bridge
    serial.Clear();
    serial.SendLine("configure_hbridge pin=4");
    ctrl->Update();
    EXPECT_TRUE(IsOk());

    serial.Clear();
    serial.SendLine("configure_hbridge pin=5");
    ctrl->Update();
    EXPECT_TRUE(IsOk());

    // Pin 3 should not support H-Bridge
    serial.Clear();
    serial.SendLine("configure_hbridge pin=3");
    ctrl->Update();
    EXPECT_TRUE(HasErrorCode(202));
}

// Test that analog input pins are limited (9-12 only)
TEST_F(SchemaTest, AnalogPinLimits) {
    // Pins 9-12 support analog input
    for (int i = 9; i <= 12; i++) {
        serial.Clear();
        serial.SendLine(("configure_analog_in pin=" + std::to_string(i)).c_str());
        ctrl->Update();
        EXPECT_TRUE(IsOk()) << "Pin " << i << " should support analog input";
    }

    // Pin 0 should not support analog input
    serial.Clear();
    serial.SendLine("configure_analog_in pin=0");
    ctrl->Update();
    EXPECT_TRUE(HasErrorCode(202));
}

// Test edge_mode string values
TEST_F(SchemaTest, EdgeModeValues) {
    std::vector<std::string> valid_modes = {"none", "rising", "falling", "both"};

    for (const auto& mode : valid_modes) {
        serial.Clear();
        serial.SendLine(("configure_digital_in pin=0 report_edges=" + mode).c_str());
        ctrl->Update();
        EXPECT_TRUE(IsOk()) << "Edge mode '" << mode << "' should be valid";
    }
}

// Test clock_rate string values
TEST_F(SchemaTest, ClockRateValues) {
    std::vector<std::string> valid_rates = {"low", "normal", "high"};

    for (const auto& rate : valid_rates) {
        serial.Clear();
        serial.SendLine(("set_motor_clock rate=" + rate).c_str());
        ctrl->Update();
        EXPECT_TRUE(IsOk()) << "Clock rate '" << rate << "' should be valid";
    }

    // Invalid rate
    serial.Clear();
    serial.SendLine("set_motor_clock rate=invalid");
    ctrl->Update();
    EXPECT_TRUE(HasErrorCode(101));
}

// Test direction type (-1 or 1)
TEST_F(SchemaTest, DirectionValues) {
    // Valid directions
    serial.Clear();
    serial.SendLine("configure_stepper motor=0 homing_direction=-1 limit_neg_pin=6");
    ctrl->Update();
    EXPECT_TRUE(IsOk());

    serial.Clear();
    serial.SendLine("configure_stepper motor=1 homing_direction=1 limit_pos_pin=7");
    ctrl->Update();
    EXPECT_TRUE(IsOk());

    // Invalid direction
    serial.Clear();
    serial.SendLine("configure_stepper motor=2 homing_direction=0");
    ctrl->Update();
    EXPECT_TRUE(HasErrorCode(101));
}

// Test that protocol version matches
TEST_F(SchemaTest, ProtocolVersionMatches) {
    ASSERT_TRUE(schema.has("protocol_version"));
    std::string schema_version = schema["protocol_version"].str_val;

    serial.Clear();
    serial.SendLine("version");
    ctrl->Update();
    std::string output = serial.GetOutput();

    EXPECT_TRUE(output.find("protocol=" + schema_version) != std::string::npos ||
                output.find("protocol=\"" + schema_version + "\"") != std::string::npos)
        << "Protocol version in schema should match implementation";
}
