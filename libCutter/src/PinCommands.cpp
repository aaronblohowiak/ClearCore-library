/**
 * @file PinCommands.cpp
 * @brief Pin command handlers
 */

#include "Cutter.h"
#include <cstring>

namespace Cutter {

// Helper to send error response
static void SendError(Controller* ctrl, const ParsedCommand& cmd,
                      ErrorCode code, const char* message) {
    (void)cmd;  // User-supplied params not used; we use internal command ID
    const CommandId& id = ctrl->GetCurrentCommandId();
    ctrl->Response().Error(static_cast<uint32_t>(code), message, id);
    ctrl->SendResponse();
}

// Helper to check state for configuration commands
static bool RequireConnected(Controller* ctrl, const ParsedCommand& cmd) {
    State state = ctrl->GetState();
    if (state == State::UNCONNECTED) {
        SendError(ctrl, cmd, ErrorCode::INVALID_STATE, "Not connected");
        return false;
    }
    return true;
}

// Helper to check if pin is available for configuration
static bool CheckPinAvailable(Controller* ctrl, const ParsedCommand& cmd, int32_t pin) {
    PinSlot* slot = ctrl->GetPin(static_cast<uint8_t>(pin));
    if (slot->mode != PinMode::UNCONFIGURED) {
        if (slot->mode == PinMode::MOTOR_LIMIT) {
            SendError(ctrl, cmd, ErrorCode::PIN_CONFLICT,
                     "Pin is reserved for motor limit switch");
        } else {
            SendError(ctrl, cmd, ErrorCode::PIN_CONFLICT,
                     "Pin is already configured");
        }
        return false;
    }
    return true;
}

// === Configuration Commands ===

static void CmdConfigureDigitalIn(Controller* ctrl, const ParsedCommand& cmd) {
    if (!RequireConnected(ctrl, cmd)) return;

    int32_t pin;
    if (!cmd.GetInt("pin", &pin) || pin < 0 || pin >= static_cast<int32_t>(NUM_PINS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_PIN, "Invalid pin");
        return;
    }

    if (!PinSupports(static_cast<uint8_t>(pin), PinCap::DIGITAL_IN)) {
        SendError(ctrl, cmd, ErrorCode::PIN_CAPABILITY, "Pin does not support digital input");
        return;
    }

    if (!CheckPinAvailable(ctrl, cmd, pin)) return;

    // Parse report_edges parameter
    EdgeMode edge_mode = EdgeMode::NONE;
    const char* edges_str = cmd.GetString("report_edges");
    if (edges_str) {
        if (strcmp(edges_str, "rising") == 0) {
            edge_mode = EdgeMode::RISING;
        } else if (strcmp(edges_str, "falling") == 0) {
            edge_mode = EdgeMode::FALLING;
        } else if (strcmp(edges_str, "both") == 0) {
            edge_mode = EdgeMode::BOTH;
        } else if (strcmp(edges_str, "none") != 0) {
            SendError(ctrl, cmd, ErrorCode::INVALID_PARAM,
                     "report_edges must be none, rising, falling, or both");
            return;
        }
    }

    PinSlot* slot = ctrl->GetPin(static_cast<uint8_t>(pin));
    slot->mode = PinMode::DIGITAL_IN;
    slot->digital_in.report_changes = cmd.GetBoolOr("report_changes", false);
    slot->digital_in.invert = cmd.GetBoolOr("invert", false);
    slot->digital_in.error_trigger_enabled = cmd.GetBoolOr("error_trigger", false);
    slot->digital_in.error_trigger_value = cmd.GetBoolOr("error_value", true);
    slot->digital_in.report_edges = edge_mode;
    slot->digital_in.config_id = ctrl->GetCurrentCommandId();

    // Set hardware pin mode
    CutterHal::ConfigurePinMode(slot->pin_index, CutterHal::PIN_MODE_INPUT_DIGITAL);

    bool raw_val = CutterHal::ReadDigitalPin(slot->pin_index);
    slot->digital_in.last_value = slot->digital_in.invert ? !raw_val : raw_val;

    // Clear any pending edge flags
    CutterHal::InputRisen(static_cast<uint8_t>(pin));
    CutterHal::InputFallen(static_cast<uint8_t>(pin));

    const CommandId& id = ctrl->GetCurrentCommandId();
    ctrl->Response().Ok(id);
    ctrl->Response().Param("pin", pin);
    ctrl->SendResponse();
}

static void CmdConfigureDigitalOut(Controller* ctrl, const ParsedCommand& cmd) {
    if (!RequireConnected(ctrl, cmd)) return;

    int32_t pin;
    if (!cmd.GetInt("pin", &pin) || pin < 0 || pin >= static_cast<int32_t>(NUM_PINS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_PIN, "Invalid pin");
        return;
    }

    if (!PinSupports(static_cast<uint8_t>(pin), PinCap::DIGITAL_OUT)) {
        SendError(ctrl, cmd, ErrorCode::PIN_CAPABILITY, "Pin does not support digital output");
        return;
    }

    if (!CheckPinAvailable(ctrl, cmd, pin)) return;

    PinSlot* slot = ctrl->GetPin(static_cast<uint8_t>(pin));
    slot->mode = PinMode::DIGITAL_OUT;
    slot->digital_out.current_value = cmd.GetBoolOr("initial", false);
    slot->digital_out.on_error_enabled = cmd.HasParam("on_error");
    slot->digital_out.on_error_value = cmd.GetBoolOr("on_error", false);
    slot->digital_out.default_max_ms = static_cast<uint32_t>(cmd.GetIntOr("max_raised_ms", 0));
    slot->digital_out.max_raised_ms = 0;
    slot->digital_out.raise_start_time = 0;
    slot->digital_out.config_id = ctrl->GetCurrentCommandId();

    // Set hardware pin mode and initial value
    CutterHal::ConfigurePinMode(slot->pin_index, CutterHal::PIN_MODE_OUTPUT_DIGITAL);
    CutterHal::WriteDigitalPin(slot->pin_index, slot->digital_out.current_value);

    const CommandId& id = ctrl->GetCurrentCommandId();
    ctrl->Response().Ok(id);
    ctrl->Response().Param("pin", pin);
    ctrl->SendResponse();
}

static void CmdConfigureAnalogIn(Controller* ctrl, const ParsedCommand& cmd) {
    if (!RequireConnected(ctrl, cmd)) return;

    int32_t pin;
    if (!cmd.GetInt("pin", &pin) || pin < 0 || pin >= static_cast<int32_t>(NUM_PINS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_PIN, "Invalid pin");
        return;
    }

    if (!PinSupports(static_cast<uint8_t>(pin), PinCap::ANALOG_IN)) {
        SendError(ctrl, cmd, ErrorCode::PIN_CAPABILITY, "Pin does not support analog input");
        return;
    }

    if (!CheckPinAvailable(ctrl, cmd, pin)) return;

    PinSlot* slot = ctrl->GetPin(static_cast<uint8_t>(pin));
    slot->mode = PinMode::ANALOG_IN;

    // Error thresholds (enter error state if outside range)
    slot->analog_in.error_threshold_enabled = cmd.HasParam("error_low") || cmd.HasParam("error_high");
    slot->analog_in.error_threshold_low = static_cast<int16_t>(cmd.GetIntOr("error_low", INT16_MIN));
    slot->analog_in.error_threshold_high = static_cast<int16_t>(cmd.GetIntOr("error_high", INT16_MAX));

    // Reporting
    slot->analog_in.report_interval_ms = static_cast<uint32_t>(cmd.GetIntOr("report_interval", 0));

    // Store config command ID for event correlation
    slot->analog_in.config_id = ctrl->GetCurrentCommandId();

    // Set hardware pin mode
    CutterHal::ConfigurePinMode(slot->pin_index, CutterHal::PIN_MODE_INPUT_ANALOG);

    slot->analog_in.last_value = CutterHal::ReadAnalogPin(slot->pin_index);
    slot->analog_in.last_report_time = CutterHal::Milliseconds();

    const CommandId& id = ctrl->GetCurrentCommandId();
    ctrl->Response().Ok(id);
    ctrl->Response().Param("pin", pin);
    ctrl->SendResponse();
}

static void CmdConfigurePwm(Controller* ctrl, const ParsedCommand& cmd) {
    if (!RequireConnected(ctrl, cmd)) return;

    int32_t pin;
    if (!cmd.GetInt("pin", &pin) || pin < 0 || pin >= static_cast<int32_t>(NUM_PINS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_PIN, "Invalid pin");
        return;
    }

    if (!PinSupports(static_cast<uint8_t>(pin), PinCap::PWM)) {
        SendError(ctrl, cmd, ErrorCode::PIN_CAPABILITY, "Pin does not support PWM");
        return;
    }

    if (!CheckPinAvailable(ctrl, cmd, pin)) return;

    PinSlot* slot = ctrl->GetPin(static_cast<uint8_t>(pin));
    slot->mode = PinMode::PWM;
    slot->pwm.duty = static_cast<uint16_t>(cmd.GetIntOr("duty", 0));

    // Set hardware pin mode and duty
    CutterHal::ConfigurePinMode(slot->pin_index, CutterHal::PIN_MODE_OUTPUT_PWM);
    CutterHal::SetPwmDuty(slot->pin_index, slot->pwm.duty);

    const CommandId& id = ctrl->GetCurrentCommandId();
    ctrl->Response().Ok(id);
    ctrl->Response().Param("pin", pin);
    ctrl->SendResponse();
}

static void CmdConfigureHBridge(Controller* ctrl, const ParsedCommand& cmd) {
    if (!RequireConnected(ctrl, cmd)) return;

    int32_t pin;
    if (!cmd.GetInt("pin", &pin) || pin < 0 || pin >= static_cast<int32_t>(NUM_PINS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_PIN, "Invalid pin");
        return;
    }

    if (!PinSupports(static_cast<uint8_t>(pin), PinCap::H_BRIDGE)) {
        SendError(ctrl, cmd, ErrorCode::PIN_CAPABILITY, "Pin does not support H-Bridge");
        return;
    }

    if (!CheckPinAvailable(ctrl, cmd, pin)) return;

    PinSlot* slot = ctrl->GetPin(static_cast<uint8_t>(pin));
    slot->mode = PinMode::H_BRIDGE;
    slot->hbridge.value = static_cast<int16_t>(cmd.GetIntOr("value", 0));
    slot->hbridge.tone_active = false;
    slot->hbridge.tone_freq = 0;
    slot->hbridge.tone_amplitude = 0;

    // Set hardware pin mode and value
    CutterHal::ConfigurePinMode(slot->pin_index, CutterHal::PIN_MODE_OUTPUT_H_BRIDGE);
    CutterHal::SetHBridgeValue(slot->pin_index, slot->hbridge.value);

    const CommandId& id = ctrl->GetCurrentCommandId();
    ctrl->Response().Ok(id);
    ctrl->Response().Param("pin", pin);
    ctrl->SendResponse();
}

static void CmdConfigureEndstop(Controller* ctrl, const ParsedCommand& cmd) {
    if (!RequireConnected(ctrl, cmd)) return;

    int32_t pin;
    if (!cmd.GetInt("pin", &pin) || pin < 0 || pin >= static_cast<int32_t>(NUM_PINS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_PIN, "Invalid pin");
        return;
    }

    if (!PinSupports(static_cast<uint8_t>(pin), PinCap::DIGITAL_IN)) {
        SendError(ctrl, cmd, ErrorCode::PIN_CAPABILITY, "Pin does not support digital input");
        return;
    }

    if (!CheckPinAvailable(ctrl, cmd, pin)) return;

    PinSlot* slot = ctrl->GetPin(static_cast<uint8_t>(pin));
    slot->mode = PinMode::END_STOP;
    // triggered=0 means triggered when LOW (NC switch, fail-safe default)
    // triggered=1 means triggered when HIGH (NO switch)
    slot->end_stop.triggered_value = static_cast<uint8_t>(cmd.GetIntOr("triggered", 0));
    slot->end_stop.last_value = CutterHal::ReadDigitalPin(slot->pin_index);

    const CommandId& id = ctrl->GetCurrentCommandId();
    ctrl->Response().Ok(id);
    ctrl->Response().Param("pin", pin);
    ctrl->SendResponse();
}

// === Operation Commands ===

static void CmdReadPin(Controller* ctrl, const ParsedCommand& cmd) {
    int32_t pin;
    if (!cmd.GetInt("pin", &pin) || pin < 0 || pin >= static_cast<int32_t>(NUM_PINS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_PIN, "Invalid pin");
        return;
    }

    PinSlot* slot = ctrl->GetPin(static_cast<uint8_t>(pin));
    if (slot->mode == PinMode::UNCONFIGURED) {
        SendError(ctrl, cmd, ErrorCode::PIN_NOT_CONFIGURED, "Pin not configured");
        return;
    }

    const CommandId& id = ctrl->GetCurrentCommandId();
    ctrl->Response().Ok(id);
    ctrl->Response().Param("pin", pin);

    switch (slot->mode) {
        case PinMode::DIGITAL_IN:
        case PinMode::END_STOP:
            ctrl->Response().Param("value", CutterHal::ReadDigitalPin(slot->pin_index));
            break;
        case PinMode::DIGITAL_OUT:
            ctrl->Response().Param("value", slot->digital_out.current_value);
            break;
        case PinMode::ANALOG_IN:
            ctrl->Response().Param("value", static_cast<int32_t>(CutterHal::ReadAnalogPin(slot->pin_index)));
            break;
        case PinMode::PWM:
            ctrl->Response().Param("duty", static_cast<int32_t>(slot->pwm.duty));
            break;
        case PinMode::H_BRIDGE:
            ctrl->Response().Param("value", static_cast<int32_t>(slot->hbridge.value));
            break;
        default:
            break;
    }
    ctrl->SendResponse();
}

static void CmdWritePin(Controller* ctrl, const ParsedCommand& cmd) {
    int32_t pin;
    if (!cmd.GetInt("pin", &pin) || pin < 0 || pin >= static_cast<int32_t>(NUM_PINS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_PIN, "Invalid pin");
        return;
    }

    PinSlot* slot = ctrl->GetPin(static_cast<uint8_t>(pin));
    if (slot->mode != PinMode::DIGITAL_OUT) {
        SendError(ctrl, cmd, ErrorCode::PIN_NOT_CONFIGURED, "Pin not configured as digital output");
        return;
    }

    bool value;
    if (!cmd.GetBool("value", &value)) {
        SendError(ctrl, cmd, ErrorCode::MISSING_PARAM, "Missing value parameter");
        return;
    }

    // Get timeout: use explicit max_ms if provided, otherwise use configured default
    uint32_t max_ms = slot->digital_out.default_max_ms;
    if (cmd.HasParam("max_ms")) {
        cmd.GetUInt("max_ms", &max_ms);
    }

    slot->digital_out.current_value = value;
    CutterHal::WriteDigitalPin(slot->pin_index, value);

    // Update timeout tracking
    const CommandId& id = ctrl->GetCurrentCommandId();
    if (value) {
        slot->digital_out.max_raised_ms = max_ms;
        if (max_ms > 0) {
            slot->digital_out.raise_start_time = CutterHal::Milliseconds();
            slot->digital_out.set_id = id;
        } else {
            slot->digital_out.raise_start_time = 0;
        }
    } else {
        slot->digital_out.raise_start_time = 0;
    }

    ctrl->Response().Ok(id);
    ctrl->Response().Param("pin", pin).Param("value", value);
    ctrl->SendResponse();
}

static void CmdSetPwm(Controller* ctrl, const ParsedCommand& cmd) {
    int32_t pin;
    if (!cmd.GetInt("pin", &pin) || pin < 0 || pin >= static_cast<int32_t>(NUM_PINS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_PIN, "Invalid pin");
        return;
    }

    PinSlot* slot = ctrl->GetPin(static_cast<uint8_t>(pin));
    if (slot->mode != PinMode::PWM) {
        SendError(ctrl, cmd, ErrorCode::PIN_NOT_CONFIGURED, "Pin not configured as PWM");
        return;
    }

    int32_t duty;
    if (cmd.GetInt("duty", &duty)) {
        slot->pwm.duty = static_cast<uint16_t>(duty);
        CutterHal::SetPwmDuty(slot->pin_index, slot->pwm.duty);
    }

    const CommandId& id = ctrl->GetCurrentCommandId();
    ctrl->Response().Ok(id);
    ctrl->Response().Param("pin", pin);
    ctrl->SendResponse();
}

static void CmdSetHBridge(Controller* ctrl, const ParsedCommand& cmd) {
    int32_t pin;
    if (!cmd.GetInt("pin", &pin) || pin < 0 || pin >= static_cast<int32_t>(NUM_PINS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_PIN, "Invalid pin");
        return;
    }

    PinSlot* slot = ctrl->GetPin(static_cast<uint8_t>(pin));
    if (slot->mode != PinMode::H_BRIDGE) {
        SendError(ctrl, cmd, ErrorCode::PIN_NOT_CONFIGURED, "Pin not configured as H-Bridge");
        return;
    }

    int32_t value;
    if (!cmd.GetInt("value", &value)) {
        SendError(ctrl, cmd, ErrorCode::MISSING_PARAM, "Missing value parameter");
        return;
    }

    slot->hbridge.value = static_cast<int16_t>(value);
    CutterHal::SetHBridgeValue(slot->pin_index, slot->hbridge.value);

    const CommandId& id = ctrl->GetCurrentCommandId();
    ctrl->Response().Ok(id);
    ctrl->Response().Param("pin", pin);
    ctrl->SendResponse();
}

static void CmdStartTone(Controller* ctrl, const ParsedCommand& cmd) {
    int32_t pin;
    if (!cmd.GetInt("pin", &pin) || pin < 0 || pin >= static_cast<int32_t>(NUM_PINS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_PIN, "Invalid pin");
        return;
    }

    PinSlot* slot = ctrl->GetPin(static_cast<uint8_t>(pin));
    if (slot->mode != PinMode::H_BRIDGE) {
        SendError(ctrl, cmd, ErrorCode::PIN_NOT_CONFIGURED, "Pin not configured as H-Bridge");
        return;
    }

    int32_t freq, amplitude;
    if (!cmd.GetInt("frequency", &freq)) {
        SendError(ctrl, cmd, ErrorCode::MISSING_PARAM, "Missing frequency parameter");
        return;
    }
    if (!cmd.GetInt("amplitude", &amplitude)) {
        SendError(ctrl, cmd, ErrorCode::MISSING_PARAM, "Missing amplitude parameter");
        return;
    }

    slot->hbridge.tone_active = true;
    slot->hbridge.tone_freq = static_cast<uint16_t>(freq);
    slot->hbridge.tone_amplitude = static_cast<int16_t>(amplitude);
    CutterHal::StartTone(slot->pin_index, slot->hbridge.tone_freq, slot->hbridge.tone_amplitude);

    const CommandId& id = ctrl->GetCurrentCommandId();
    ctrl->Response().Ok(id);
    ctrl->Response().Param("pin", pin);
    ctrl->SendResponse();
}

static void CmdStopTone(Controller* ctrl, const ParsedCommand& cmd) {
    int32_t pin;
    if (!cmd.GetInt("pin", &pin) || pin < 0 || pin >= static_cast<int32_t>(NUM_PINS)) {
        SendError(ctrl, cmd, ErrorCode::INVALID_PIN, "Invalid pin");
        return;
    }

    PinSlot* slot = ctrl->GetPin(static_cast<uint8_t>(pin));
    if (slot->mode != PinMode::H_BRIDGE) {
        SendError(ctrl, cmd, ErrorCode::PIN_NOT_CONFIGURED, "Pin not configured as H-Bridge");
        return;
    }

    slot->hbridge.tone_active = false;
    CutterHal::StopTone(slot->pin_index);

    const CommandId& id = ctrl->GetCurrentCommandId();
    ctrl->Response().Ok(id);
    ctrl->Response().Param("pin", pin);
    ctrl->SendResponse();
}

// === Dispatch Function ===

void DispatchPinCommand(Controller* ctrl, const ParsedCommand& cmd) {
    if (strcmp(cmd.name, "configure_digital_in") == 0) {
        CmdConfigureDigitalIn(ctrl, cmd);
    } else if (strcmp(cmd.name, "configure_digital_out") == 0) {
        CmdConfigureDigitalOut(ctrl, cmd);
    } else if (strcmp(cmd.name, "configure_analog_in") == 0) {
        CmdConfigureAnalogIn(ctrl, cmd);
    } else if (strcmp(cmd.name, "configure_pwm") == 0) {
        CmdConfigurePwm(ctrl, cmd);
    } else if (strcmp(cmd.name, "configure_hbridge") == 0) {
        CmdConfigureHBridge(ctrl, cmd);
    } else if (strcmp(cmd.name, "configure_endstop") == 0) {
        CmdConfigureEndstop(ctrl, cmd);
    } else if (strcmp(cmd.name, "read_pin") == 0) {
        CmdReadPin(ctrl, cmd);
    } else if (strcmp(cmd.name, "write_pin") == 0) {
        CmdWritePin(ctrl, cmd);
    } else if (strcmp(cmd.name, "set_pwm") == 0) {
        CmdSetPwm(ctrl, cmd);
    } else if (strcmp(cmd.name, "set_hbridge") == 0) {
        CmdSetHBridge(ctrl, cmd);
    } else if (strcmp(cmd.name, "start_tone") == 0) {
        CmdStartTone(ctrl, cmd);
    } else if (strcmp(cmd.name, "stop_tone") == 0) {
        CmdStopTone(ctrl, cmd);
    }
}

}  // namespace Cutter
