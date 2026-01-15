# Cutter Library Design Document

## Overview

Cutter is a host-controlled command layer for ClearCore that exposes high-level motor and I/O commands over USB or Ethernet. Unlike Klipper, Cutter uses motor commands (not G-code), executes commands immediately when received, and provides completion notifications by default.

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                           HOST COMPUTER                              │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │                    Application Logic                         │    │
│  │  - Motion sequencing      - Error handling                   │    │
│  │  - I/O monitoring         - State management                 │    │
│  └──────────────────────────────┬──────────────────────────────┘    │
│                                 │ USB/Ethernet                       │
└─────────────────────────────────┼───────────────────────────────────┘
                                  │
┌─────────────────────────────────┼───────────────────────────────────┐
│                           CLEARCORE                                  │
│  ┌──────────────────────────────┴──────────────────────────────┐    │
│  │                      Cutter Library                          │    │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │    │
│  │  │   Command   │  │   State     │  │   Response          │  │    │
│  │  │   Parser    │  │   Machine   │  │   Generator         │  │    │
│  │  └──────┬──────┘  └──────┬──────┘  └──────────┬──────────┘  │    │
│  │         │                │                     │             │    │
│  │  ┌──────┴─────────────────┴─────────────────────┴──────┐     │    │
│  │  │                  Command Dispatcher                  │     │    │
│  │  └──────┬──────────┬───────────┬───────────┬───────────┘     │    │
│  │         │          │           │           │                 │    │
│  │  ┌──────┴───┐ ┌────┴────┐ ┌────┴────┐ ┌────┴─────┐          │    │
│  │  │  Motor   │ │   Pin   │ │ Monitor │ │  System  │          │    │
│  │  │ Manager  │ │ Manager │ │ Manager │ │ Commands │          │    │
│  │  └──────────┘ └─────────┘ └─────────┘ └──────────┘          │    │
│  └──────────────────────────────────────────────────────────────┘    │
│                                 │                                    │
│  ┌──────────────────────────────┴──────────────────────────────┐    │
│  │                   libClearCore (existing)                    │    │
│  │   MotorDriver | DigitalIn/Out | ADC | Serial | Ethernet     │    │
│  └──────────────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────────────┘
```

## State Machine

```
                    ┌──────────────┐
                    │  UNCONNECTED │
                    └──────┬───────┘
                           │ Connection established
                           ▼
                    ┌──────────────┐
         ┌──────────│  CONNECTED   │
         │          └──────┬───────┘
         │                 │ All required config received
         │                 ▼
         │          ┌──────────────┐
         │          │  CONFIGURED  │
         │          └──────┬───────┘
         │                 │ enable command
         │                 ▼
         │          ┌──────────────┐
         │          │   ENABLING   │──────────────────┐
         │          └──────┬───────┘                  │
         │                 │ All motors report ready  │
         │                 ▼                          │
         │          ┌──────────────┐                  │
         │          │    READY     │◄─────────┐      │
         │          └──────┬───────┘          │      │
         │                 │ Command received │      │
         │                 ▼                  │      │
         │          ┌──────────────┐          │      │
         │          │   WORKING    │──────────┘      │
         │          └──────┬───────┘ All commands    │
         │                 │         complete        │
         │                 │                         │
         │                 │ Error condition         │
         │                 ▼                         │
         │          ┌──────────────┐                 │
         └─────────►│    ERROR     │◄────────────────┘
                    └──────┬───────┘
                           │ reset command (increments epoch)
                           ▼
                    ┌──────────────┐
                    │  CONNECTED   │
                    └──────────────┘
```

## Memory Layout (No malloc)

All data structures use static allocation with fixed maximum sizes.

```cpp
// Compile-time configuration limits
constexpr size_t MAX_COMMAND_LENGTH = 256;
constexpr size_t MAX_KEY_LENGTH = 32;
constexpr size_t MAX_VALUE_LENGTH = 64;
constexpr size_t MAX_PARAMS = 16;
constexpr size_t NUM_PINS = 13;            // IO-0 through A-12
constexpr size_t NUM_MOTORS = 4;           // M-0 through M-3
constexpr size_t MAX_RESPONSE_LENGTH = 512;
```

## File Structure

```
libCutter/
├── inc/
│   ├── Cutter.h                 # Main library header, CutterController class
│   ├── CutterConfig.h           # Compile-time configuration constants
│   ├── CutterState.h            # State machine definition
│   ├── CutterCommand.h          # Command structures and enums
│   ├── CutterResponse.h         # Response formatting
│   ├── CommandParser.h          # Line parser, key-value extraction
│   ├── MotorState.h             # Motor config + runtime state
│   ├── PinState.h               # Pin config + runtime state (includes monitoring)
│   └── ErrorHandler.h           # Error state management
├── src/
│   ├── Cutter.cpp               # Main controller implementation
│   ├── CutterState.cpp          # State transitions
│   ├── CommandParser.cpp        # Parsing logic
│   ├── CommandDispatcher.cpp    # Command routing
│   ├── MotorCommands.cpp        # Motor command handlers
│   ├── PinCommands.cpp          # Pin command handlers
│   ├── ResponseWriter.cpp       # Response formatting
│   └── ErrorHandler.cpp         # Error handling
└── examples/
    ├── BasicUsb/                # USB serial example
    ├── EthernetServer/          # TCP server example
    └── FullSystem/              # Complete machine example
```

## Core Data Structures

### Command Structure

```cpp
// Parsed command representation (stack allocated)
struct ParsedCommand {
    char name[MAX_KEY_LENGTH];           // Command name
    uint32_t epoch;                       // Error epoch (optional)
    uint32_t seq;                         // Sequence number (optional)
    bool has_epoch;
    bool has_seq;

    struct Param {
        char key[MAX_KEY_LENGTH];
        char value[MAX_VALUE_LENGTH];
        bool is_set;
    } params[MAX_PARAMS];

    uint8_t param_count;

    // Helper methods
    bool GetInt(const char* key, int32_t* out) const;
    bool GetUInt(const char* key, uint32_t* out) const;
    bool GetFloat(const char* key, float* out) const;  // Parsed from string
    bool GetBool(const char* key, bool* out) const;
    const char* GetString(const char* key) const;
};
```

### Pin State (Tagged Union with Type-Specific Structs)

Each of the 13 pins has a `PinSlot` that contains only the fields relevant to its configured mode. This avoids a "god struct" while maintaining static allocation.

```cpp
enum class PinMode : uint8_t {
    UNCONFIGURED = 0,
    DIGITAL_IN,
    DIGITAL_OUT,
    ANALOG_IN,
    PWM,
    H_BRIDGE,
    END_STOP
};

// === Type-specific state structs ===

struct DigitalInState {
    bool invert;
    bool error_trigger_enabled;
    bool error_trigger_value;
    bool report_changes;
    // Runtime
    bool last_value;
};

struct DigitalOutState {
    uint32_t max_raised_ms;          // 0 = no limit
    int8_t on_error;                 // -1 = no change, 0 = low, 1 = high
    // Runtime
    bool current_value;
    uint32_t raised_at_ms;           // When output was set high
};

struct AnalogInState {
    int16_t error_threshold_low;     // INT16_MIN = disabled
    int16_t error_threshold_high;    // INT16_MAX = disabled
    int16_t stop_threshold_low;
    int16_t stop_threshold_high;
    uint32_t report_interval_ms;     // 0 = disabled
    bool report_threshold_cross;
    // Runtime
    int16_t last_value;
    uint32_t last_report_ms;
    bool threshold_triggered;
};

struct PwmState {
    bool stop_on_error;
    uint16_t duty;                   // Current duty cycle (0-65535)
    uint32_t frequency;
};

struct HBridgeState {
    bool stop_on_error;
    int16_t value;                   // -32767 to +32767 (bidirectional)
    // Tone state
    int16_t tone_amplitude;          // Amplitude for tone generation (0 to INT16_MAX)
    uint16_t tone_frequency;         // Current tone frequency (Hz)
    bool tone_active;
    uint32_t tone_end_ms;            // 0 = no auto-stop
};

struct EndStopState {
    uint8_t motor_index;
    int8_t direction;                // -1 or 1
    bool active_low;
    // Runtime
    bool last_value;
    bool is_triggered;
};

// === Pin slot: tagged union ===

struct PinSlot {
    PinMode mode;
    uint8_t pin_index;

    union {
        DigitalInState digital_in;
        DigitalOutState digital_out;
        AnalogInState analog_in;
        PwmState pwm;
        HBridgeState hbridge;
        EndStopState end_stop;
    };

    // Type-dispatched methods
    void Check(uint32_t now_ms, CutterController* ctrl);
    void ApplyErrorState();
    int16_t Read();
    void Write(int16_t value);
};

// Static array - each slot sized to largest union member
PinSlot pins[NUM_PINS];  // 13 pins
```

### Type-Specific Check Functions

```cpp
// Each type has its own check logic - no god-method
namespace PinCheck {

void DigitalIn(PinSlot* slot, uint32_t now, CutterController* ctrl) {
    auto& s = slot->digital_in;
    bool value = ReadDigitalPin(slot->pin_index);
    if (s.invert) value = !value;

    if (s.report_changes && value != s.last_value) {
        ctrl->SendEvent("input", "pin", slot->pin_index, "value", value);
    }
    if (s.error_trigger_enabled && value == s.error_trigger_value) {
        ctrl->EnterError(CutterError::ERROR_INPUT_TRIGGERED,
                         "Pin %d triggered", slot->pin_index);
    }
    s.last_value = value;
}

void DigitalOut(PinSlot* slot, uint32_t now, CutterController* ctrl) {
    auto& s = slot->digital_out;
    if (s.max_raised_ms > 0 && s.current_value) {
        if ((now - s.raised_at_ms) >= s.max_raised_ms) {
            WriteDigitalPin(slot->pin_index, false);
            s.current_value = false;
            ctrl->SendEvent("timeout", "pin", slot->pin_index);
        }
    }
}

void AnalogIn(PinSlot* slot, uint32_t now, CutterController* ctrl) {
    auto& s = slot->analog_in;
    int16_t value = ReadAnalogPin(slot->pin_index);

    // Error thresholds - enter error state
    if (value < s.error_threshold_low || value > s.error_threshold_high) {
        ctrl->EnterError(CutterError::ANALOG_THRESHOLD_EXCEEDED,
                         "Pin %d: %d", slot->pin_index, value);
        return;
    }

    // Stop thresholds - stop motors but don't error
    bool outside_stop = (value < s.stop_threshold_low ||
                         value > s.stop_threshold_high);
    if (outside_stop && !s.threshold_triggered) {
        ctrl->StopAllMotors();
        s.threshold_triggered = true;
        if (s.report_threshold_cross) {
            ctrl->SendEvent("threshold", "pin", slot->pin_index, "value", value);
        }
    } else if (!outside_stop) {
        s.threshold_triggered = false;
    }

    // Interval reporting
    if (s.report_interval_ms > 0 &&
        (now - s.last_report_ms) >= s.report_interval_ms) {
        ctrl->SendEvent("analog", "pin", slot->pin_index, "value", value);
        s.last_report_ms = now;
    }

    s.last_value = value;
}

void EndStop(PinSlot* slot, uint32_t now, CutterController* ctrl) {
    auto& s = slot->end_stop;
    bool raw = ReadDigitalPin(slot->pin_index);
    bool triggered = s.active_low ? !raw : raw;

    if (triggered && !s.is_triggered) {
        // End stop just activated
        auto& motor = ctrl->GetMotor(s.motor_index);
        if (motor.is_homing) {
            // Expected during homing - signal to homing logic
            motor.home_stop_hit = true;
        } else if (motor.is_moving) {
            // Unexpected - error
            ctrl->EnterError(CutterError::END_STOP_TRIGGERED,
                             "End stop pin %d hit", slot->pin_index);
        }
    }
    s.is_triggered = triggered;
    s.last_value = raw;
}

// PWM and HBridge checks are simpler - mainly tone timeout
void HBridge(PinSlot* slot, uint32_t now, CutterController* ctrl) {
    auto& s = slot->hbridge;
    if (s.tone_active && s.tone_end_ms > 0 &&
        (now - s.tone_end_ms) >= 0) {  // tone_end_ms is absolute time
        StopTone(slot->pin_index);
        s.tone_active = false;
        ctrl->SendEvent("tone_done", "pin", slot->pin_index);
    }
}

} // namespace PinCheck

// Dispatcher
void PinSlot::Check(uint32_t now, CutterController* ctrl) {
    switch (mode) {
        case PinMode::DIGITAL_IN:  PinCheck::DigitalIn(this, now, ctrl); break;
        case PinMode::DIGITAL_OUT: PinCheck::DigitalOut(this, now, ctrl); break;
        case PinMode::ANALOG_IN:   PinCheck::AnalogIn(this, now, ctrl); break;
        case PinMode::END_STOP:    PinCheck::EndStop(this, now, ctrl); break;
        case PinMode::H_BRIDGE:    PinCheck::HBridge(this, now, ctrl); break;
        case PinMode::PWM:         /* No periodic check needed */ break;
        case PinMode::UNCONFIGURED: break;
    }
}
```

### Motor State

Motors share more common behavior than pins, so a single struct with a type discriminator works well here.

```cpp
enum class MotorType : uint8_t {
    UNCONFIGURED = 0,
    CLEARPATH_SDSK,    // ClearPath with HLFB
    GENERIC_STEPPER    // Step/dir stepper
};

struct MotorState {
    // === Configuration ===
    MotorType type;
    uint8_t motor_index;
    uint8_t enable_priority;         // Order for enabling (0 = first)
    bool enable_on_ready;            // Auto-enable when entering Ready state

    // Homing configuration
    int8_t homing_end_stop_pin;      // Pin index for homing (-1 = none)
    int8_t far_end_stop_pin;         // Pin index for far limit (-1 = none)
    int8_t homing_dir;               // Direction for homing (-1 or 1)
    int32_t homing_velocity;
    int32_t homing_backoff;          // Steps to back off after hitting stop

    // Motion parameters
    int32_t vel_max;
    int32_t accel_max;

    // Stepper-specific (ignored for ClearPath)
    uint32_t steps_per_unit;         // 0 = use steps directly

    // === Runtime State ===
    bool is_enabled;
    bool is_homing;
    bool has_homed;
    bool is_moving;
    bool home_stop_hit;              // Set by EndStop check during homing
    int32_t position;                // Current position in steps

    // Move tracking for completion events
    uint32_t active_move_seq;        // Sequence number of in-progress move

    // HLFB state (ClearPath only)
    uint8_t hlfb_state;
    uint8_t last_hlfb_state;

    // === Methods ===
    void Check(uint32_t now_ms, CutterController* ctrl);
    bool IsReady();                   // HLFB asserted or stepper ready
    bool IsMoveComplete();            // StepsComplete() or HLFB
};

// Static array
MotorState motors[NUM_MOTORS];  // 4 motors
```

### MotorState::Check() Implementation

```cpp
void MotorState::Check(uint32_t now, CutterController* ctrl) {
    if (type == MotorType::UNCONFIGURED) return;

    // Update position from hardware
    position = GetMotorPosition(motor_index);

    // HLFB monitoring for ClearPath
    if (type == MotorType::CLEARPATH_SDSK) {
        hlfb_state = GetHlfbState(motor_index);
        if (hlfb_state != last_hlfb_state) {
            ctrl->SendEvent("hlfb", "motor", motor_index, "state", hlfb_state);
            last_hlfb_state = hlfb_state;
        }
    }

    // Homing state machine
    if (is_homing && home_stop_hit) {
        // End stop was hit - stop motor, back off, zero position
        StopMotorImmediate(motor_index);
        // Start backoff move in opposite direction
        MoveMotor(motor_index, -homing_dir * homing_backoff);
        is_homing = false;
        has_homed = true;
        home_stop_hit = false;
        // Position will be zeroed when backoff completes
        // (tracked via is_moving + !is_homing)
    }

    // Move completion detection
    if (is_moving && IsMoveComplete()) {
        is_moving = false;
        if (active_move_seq != 0) {
            ctrl->SendEvent("done", "motor", motor_index, "seq", active_move_seq);
            active_move_seq = 0;
        }
        // If we just finished homing backoff, zero position
        if (has_homed && position != 0) {
            SetMotorPosition(motor_index, 0);
            position = 0;
            ctrl->SendEvent("homed", "motor", motor_index, "position", 0);
        }
    }
}

bool MotorState::IsMoveComplete() {
    if (type == MotorType::CLEARPATH_SDSK) {
        // HLFB asserted means move complete (mode-dependent)
        return hlfb_state == HLFB_ASSERTED;
    } else {
        return StepsComplete(motor_index);
    }
}
```

### Cutter State

```cpp
enum class CutterState : uint8_t {
    UNCONNECTED = 0,
    CONNECTED,
    CONFIGURED,
    ENABLING,
    READY,
    WORKING,
    ERROR
};

struct CutterStatus {
    CutterState state;
    uint32_t epoch;                // Error epoch counter
    uint32_t seq;                  // Last received sequence number
    uint32_t active_commands;      // Number of commands in progress
    uint32_t error_code;           // Last error code
    char error_message[64];        // Human-readable error description
};
```

## Command Protocol

### Wire Format

```
# Comments start with '#' and are ignored
# Empty lines are ignored

# Basic format:
command_name [key=value] [key2=value2] ...

# All commands may include epoch and seq:
command_name epoch=0 seq=42 key=value

# Values can be:
#   - Integers: 123, -456
#   - Floats: 3.14, -0.5
#   - Booleans: 0, 1, true, false
#   - Strings: unquoted (no spaces) or "quoted with spaces"
```

### Response Format

```
# Success response:
ok [key=value] ...

# Success with command echo:
ok seq=42 command=move

# Error response:
error code=101 message="Invalid parameter"

# Async notifications:
event type=done motor=0 seq=42
event type=input pin=6 value=1
event type=hlfb motor=0 state=2
event type=threshold pin=9 value=2048
event type=error code=500 message="End stop triggered"

# Debug output:
debug message="Motor 0 enabled"
```

## Command Reference

### System Commands

```
# Get protocol version and capabilities
get_version
→ ok version=1.0 protocol=cutter motors=4 pins=13

# Get current state
get_state
→ ok state=ready epoch=0 seq=123

# Get next expected sequence number
get_next_seq
→ ok epoch=0 seq=124

# Reset from error state (increments epoch)
reset
→ ok epoch=1 seq=0

# Enter error state manually
emergency_stop [message="reason"]
→ ok

# Ping/keepalive
ping
→ ok

# Enable debug output
set_debug enabled=1
→ ok
```

### Configuration Commands

#### Pin Configuration

```
# Configure digital input
configure_digital_in pin=6 [error_trigger=0] [invert=0]
→ ok

# Configure digital output
configure_digital_out pin=0 [max_raised_ms=0] [on_error=0]
→ ok

# Configure analog input
configure_analog_in pin=9 [error_low=-1] [error_high=-1]
→ ok

# Configure PWM output (pins 4, 5 only)
configure_pwm pin=4 [stop_on_error=1] [amplitude=32767]
→ ok

# Configure H-Bridge (pins 4, 5 only)
configure_hbridge pin=4 [stop_on_error=1]
→ ok

# Configure end stop
configure_end_stop pin=7 motor=0 [direction=-1] [active_low=1]
→ ok
```

#### Motor Configuration

```
# Configure ClearPath motor with HLFB
configure_sdsk motor=0 [enable_priority=0] [enable_on_ready=1] \
               [homing_end_stop=-1] [far_end_stop=-1] [homing_dir=-1]
→ ok

# Configure generic stepper
configure_stepper motor=1 [enable_priority=0] [enable_on_ready=1] \
                  [homing_end_stop=-1] [far_end_stop=-1] [homing_dir=-1] \
                  [steps_per_unit=200]
→ ok

# Set motor motion parameters
set_motor_params motor=0 vel_max=10000 accel_max=100000 [vel_limit=50000]
→ ok

# Mark configuration complete and transition to CONFIGURED state
configuration_done
→ ok state=configured
```

### Motor Commands

```
# Enable motors (transitions to ENABLING, then READY)
enable
→ ok state=enabling
→ event type=state state=ready  # Async when all motors ready

# Disable motors
disable
→ ok

# Enable/disable individual motor
enable_motor motor=0
disable_motor motor=0
→ ok

# Relative move (steps from current position)
move motor=0 steps=1000 [vel=5000] [accel=50000]
→ ok seq=42
→ event type=done motor=0 seq=42  # Async when complete

# Absolute move (to position)
move_to motor=0 position=5000 [vel=5000] [accel=50000]
→ ok seq=43
→ event type=done motor=0 seq=43

# Velocity move (continuous until stopped)
move_velocity motor=0 velocity=2000
→ ok seq=44

# Stop motor (decelerate to stop)
stop motor=0 [accel=100000]
→ ok

# Stop motor immediately
stop_immediate motor=0
→ ok

# Stop all motors
stop_all [immediate=0]
→ ok

# Home motor
home motor=0 [velocity=1000] [backoff=100]
→ ok seq=45
→ event type=homed motor=0 seq=45 position=0

# Set position without moving
set_position motor=0 position=0
→ ok

# Get motor status
get_motor_status motor=0
→ ok motor=0 enabled=1 position=1234 velocity=0 hlfb=1 moving=0 homed=1
```

### Pin Commands

```
# Set digital output
set_output pin=0 value=1
→ ok

# Read digital input
get_input pin=6
→ ok pin=6 value=1

# Read analog input
get_analog pin=9
→ ok pin=9 value=2048 voltage=1.65

# Set PWM duty cycle (0-65535)
set_pwm pin=4 duty=32768
→ ok

# Set PWM frequency
set_pwm_freq pin=4 freq=1000
→ ok

# Set H-Bridge output (-100 to 100, or duty cycle)
set_hbridge pin=4 value=50
→ ok

# Generate tone on H-Bridge
tone pin=4 freq=440 [duration_ms=1000]
→ ok
→ event type=tone_done pin=4  # If duration specified
```

### Reporting Options (Integrated into Configuration)

Monitoring is configured as part of pin setup, not as a separate system.

```
# Digital input with change reporting
configure_digital_in pin=6 report_changes=1
→ ok
→ event type=input pin=6 value=1  # Async on change

# Analog input with interval reporting
configure_analog_in pin=9 report_interval_ms=100
→ ok
→ event type=analog pin=9 value=2048  # Async at interval

# Analog input with threshold monitoring
configure_analog_in pin=9 stop_low=1000 stop_high=3000 report_threshold=1
→ ok
→ event type=threshold pin=9 value=950 direction=low  # When crossed

# Modify reporting on already-configured pin
set_reporting pin=6 report_changes=0
→ ok

set_reporting pin=9 report_interval_ms=0 report_threshold=0
→ ok
```

## Implementation Phases

### Phase 1: Core Infrastructure

**Files to create:**
- `CutterConfig.h` - Constants, limits, pin capability tables
- `CutterState.h/cpp` - State machine
- `CommandParser.h/cpp` - Line parsing
- `CutterResponse.h/cpp` - Response formatting
- `ErrorHandler.h` - Error codes

**Functionality:**
- State machine with valid transition checks
- Command line parsing (no malloc, fixed buffers)
- Key-value extraction with type conversion
- Response string building
- Error code enum

### Phase 2: Communication & Main Loop

**Files to create:**
- `Cutter.h/cpp` - Main controller class
- `CommandDispatcher.cpp` - Command routing

**Functionality:**
- USB Serial integration
- Ethernet TCP server integration
- Input line buffering and accumulation
- Output buffering
- Sequence number tracking and epoch validation
- Main `Update()` loop structure
- System commands: `ping`, `get_version`, `get_state`, `get_next_seq`

### Phase 3: Pin State & Configuration

**Files to create:**
- `PinState.h/cpp` - Unified pin config + runtime + monitoring

**Functionality:**
- `PinState` struct with all fields
- `PinState::Check()` method for self-monitoring
- Configuration commands: `configure_digital_in`, `configure_digital_out`, `configure_analog_in`, `configure_pwm`, `configure_hbridge`
- Control commands: `set_output`, `get_input`, `get_analog`, `set_pwm`, `set_hbridge`
- Digital output timeout logic
- Analog threshold checking (error and stop triggers)
- Change reporting for digital inputs
- Interval reporting for analog inputs

### Phase 4: Motor State & Configuration

**Files to create:**
- `MotorState.h/cpp` - Unified motor config + runtime

**Functionality:**
- `MotorState` struct with all fields
- `MotorState::Check()` method for completion detection
- Configuration commands: `configure_sdsk`, `configure_stepper`, `set_motor_params`
- `configuration_done` command to transition to CONFIGURED

### Phase 5: Motor Control & Completion

**Functionality:**
- Enable/disable: `enable`, `disable`, `enable_motor`, `disable_motor`
- Priority-based enable sequencing
- Move commands: `move`, `move_to`, `move_velocity`
- Stop commands: `stop`, `stop_immediate`, `stop_all`
- Status query: `get_motor_status`, `set_position`
- HLFB monitoring for ClearPath motors
- StepsComplete monitoring for generic steppers
- Async "done" event generation with seq correlation

### Phase 6: Homing & End Stops

**Functionality:**
- `configure_end_stop` command linking pins to motors
- `home` command implementation
- Homing sequence: move toward stop, detect, back off, zero position
- End stop monitoring during normal operation
- Error triggering on unexpected end stop activation
- `event type=homed` generation

### Phase 7: Error Handling & Recovery

**Functionality:**
- `EnterError()` method with state transition
- Apply `on_error` values to digital outputs
- Stop PWM/H-Bridge if `stop_on_error` set
- Disable all motors on error
- `emergency_stop` command
- `reset` command (increment epoch, return to CONNECTED)
- `event type=error` generation

### Phase 8: Examples & Testing

**Files to create:**
- `examples/BasicUsb/BasicUsb.cpp` - Minimal USB setup
- `examples/EthernetServer/EthernetServer.cpp` - TCP server
- `examples/FullSystem/FullSystem.cpp` - Complete machine

**Testing:**
- Manual testing via USB serial
- Python test scripts for automated command sequences
- Stress testing for sequence handling

## Error Codes

```cpp
enum class CutterError : uint32_t {
    NONE = 0,

    // Protocol errors (100-199)
    UNKNOWN_COMMAND = 100,
    INVALID_PARAMETER = 101,
    MISSING_PARAMETER = 102,
    INVALID_VALUE = 103,
    SEQUENCE_ERROR = 104,
    EPOCH_MISMATCH = 105,
    COMMAND_TOO_LONG = 106,

    // State errors (200-299)
    INVALID_STATE = 200,
    NOT_CONFIGURED = 201,
    NOT_ENABLED = 202,
    ALREADY_CONFIGURED = 203,
    MOTOR_NOT_HOMED = 204,

    // Configuration errors (300-399)
    INVALID_PIN = 300,
    INVALID_MOTOR = 301,
    PIN_NOT_CAPABLE = 302,
    PIN_ALREADY_CONFIGURED = 303,
    MOTOR_ALREADY_CONFIGURED = 304,

    // Hardware errors (400-499)
    MOTOR_FAULT = 400,
    MOTOR_NOT_READY = 401,
    HLFB_ERROR = 402,
    SUPPLY_VOLTAGE_LOW = 403,
    SUPPLY_VOLTAGE_HIGH = 404,
    OVERTEMPERATURE = 405,

    // Runtime errors (500-599)
    END_STOP_TRIGGERED = 500,
    ERROR_INPUT_TRIGGERED = 501,
    ANALOG_THRESHOLD_EXCEEDED = 502,
    TIMEOUT = 503,
    EMERGENCY_STOP = 504,

    // Communication errors (600-699)
    CONNECTION_LOST = 600,
    BUFFER_OVERFLOW = 601,
};
```

## Main Loop Integration

```cpp
// Example main.cpp
#include "ClearCore.h"
#include "Cutter.h"

// Static allocation of Cutter controller
static CutterController cutter;

int main() {
    // Initialize ClearCore
    SysManager.Initialize();

    // Initialize Cutter with USB serial
    cutter.Initialize(&ConnectorUsb);

    // Or with Ethernet:
    // EthernetTcpServer server;
    // server.Open(5000);
    // cutter.Initialize(&server);

    while (true) {
        cutter.Update();
    }
}
```

### Update() Implementation

```cpp
void CutterController::Update() {
    uint32_t now = Milliseconds();

    // 1. Read available serial/ethernet input, accumulate lines
    ReadInput();

    // 2. If we have a complete line, parse and dispatch
    if (HasCompleteLine()) {
        ParsedCommand cmd;
        if (parser_.Parse(line_buffer_, &cmd)) {
            DispatchCommand(cmd);
        }
        ClearLineBuffer();
    }

    // 3. Check each pin - dispatches to type-specific PinCheck::* functions
    for (uint8_t i = 0; i < NUM_PINS; i++) {
        pins_[i].Check(now, this);  // No-op if UNCONFIGURED
    }

    // 4. Check each motor
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors_[i].Check(now, this);  // No-op if UNCONFIGURED
    }

    // 5. Flush any queued responses/events
    FlushOutput();
}
```

See "Type-Specific Check Functions" and "MotorState::Check() Implementation" sections above for the detailed check logic.

## Timing

ClearCore provides two timing functions (see `SysTiming.h`):

| Function | Type | Rollover |
|----------|------|----------|
| `Milliseconds()` | `uint32_t` | ~49.7 days |
| `Microseconds()` | `uint32_t` | ~71.5 minutes |

**Use `Milliseconds()` for Cutter** - 49 days is plenty for any timeout or interval.

**Rollover-safe pattern:**
```cpp
// This works correctly even when now < start (rollover occurred)
// due to unsigned integer arithmetic
uint32_t start = Milliseconds();
// ... later ...
uint32_t now = Milliseconds();
if ((now - start) >= timeout_ms) {
    // Timeout elapsed
}
```

The subtraction `now - start` produces the correct elapsed time even across rollover, as long as the actual elapsed time is less than `UINT32_MAX` milliseconds (~49.7 days).

## Thread Safety Considerations

The ClearCore runs single-threaded with a 5kHz ISR for hardware updates. Cutter integrates with this model:

1. **Main loop**: All command processing happens in `Update()` called from main loop
2. **ISR interaction**: Motor state and I/O are updated by ClearCore's ISR
3. **Atomic reads**: Motor position, HLFB state, and pin values are read atomically
4. **No locking needed**: Single-threaded execution model

## Performance Considerations

1. **Command parsing**: O(n) where n is command length, no allocations
2. **Response generation**: Fixed buffer, snprintf-style formatting
3. **Pin checks**: O(13) per `Update()` call - iterate all pins, only process configured ones
4. **Motor checks**: O(4) per `Update()` call - HLFB and StepsComplete checked

## Memory Budget

```
Static allocations:
- Command buffer:        256 bytes
- Response buffer:       512 bytes
- PinSlot (13):          ~416 bytes (32 bytes each, sized to largest union member)
- MotorState (4):        ~200 bytes (50 bytes each)
- CutterStatus:          ~80 bytes
- Parsing workspace:     ~512 bytes
- Line buffer:           256 bytes
─────────────────────────────────────
Total:                   ~2.2 KB

ClearCore has 256KB SRAM, so this is well within budget.
```

**PinSlot union sizes:**
- `DigitalInState`: 5 bytes
- `DigitalOutState`: 10 bytes
- `AnalogInState`: 21 bytes (largest)
- `PwmState`: 7 bytes
- `HBridgeState`: 12 bytes
- `EndStopState`: 5 bytes

Each `PinSlot` = 2 bytes (mode + pin_index) + 21 bytes (union) + padding ≈ 32 bytes

## Future Extensions (Not in Initial Scope)

- Multiple simultaneous connections
- Command queuing with timing
- Coordinated multi-axis moves
- Macro/script storage in flash
- Configuration persistence
- CAN bus support for CCIO-8 expansion
