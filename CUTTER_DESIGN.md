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
constexpr size_t MAX_MOTORS = 4;
constexpr size_t MAX_DIGITAL_PINS = 13;    // IO-0 through A-12
constexpr size_t MAX_MONITORS = 8;
constexpr size_t MAX_RESPONSE_LENGTH = 512;
constexpr size_t MAX_PENDING_COMMANDS = 4; // Small queue for burst handling
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
│   ├── MotorConfig.h            # Motor configuration structures
│   ├── PinConfig.h              # Pin configuration structures
│   ├── Monitor.h                # I/O monitoring with thresholds
│   └── ErrorHandler.h           # Error state management
├── src/
│   ├── Cutter.cpp               # Main controller implementation
│   ├── CutterState.cpp          # State transitions
│   ├── CommandParser.cpp        # Parsing logic
│   ├── CommandDispatcher.cpp    # Command routing
│   ├── MotorCommands.cpp        # Motor command handlers
│   ├── PinCommands.cpp          # Pin command handlers
│   ├── ConfigCommands.cpp       # Configuration command handlers
│   ├── MonitorManager.cpp       # Monitoring logic
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

### Pin Configuration

```cpp
enum class PinMode : uint8_t {
    UNCONFIGURED = 0,
    DIGITAL_IN,
    DIGITAL_OUT,
    ANALOG_IN,
    ANALOG_OUT,
    PWM,
    H_BRIDGE,
    END_STOP
};

struct DigitalInConfig {
    uint8_t pin;
    bool error_trigger_enabled;
    bool error_trigger_value;      // Trigger error when pin equals this value
    bool invert;
};

struct DigitalOutConfig {
    uint8_t pin;
    uint32_t max_raised_ms;        // 0 = no limit
    int8_t on_error;               // -1 = don't change, 0 = low, 1 = high
    uint32_t raised_at_ms;         // Timestamp when raised (for timeout)
};

struct AnalogInConfig {
    uint8_t pin;                   // 9, 10, 11, or 12
    int16_t error_threshold_low;   // Trigger error if below (or INT16_MIN to disable)
    int16_t error_threshold_high;  // Trigger error if above (or INT16_MAX to disable)
};

struct PwmConfig {
    uint8_t pin;                   // 4 or 5
    bool stop_on_error;
    int16_t amplitude;             // Scaling factor
};

struct EndStopConfig {
    uint8_t pin;
    uint8_t motor;
    int8_t direction;              // -1 or 1, direction where this stop applies
    bool active_low;               // Pin state that indicates activated
};

// Union of all pin configs with discriminator
struct PinConfiguration {
    PinMode mode;
    uint8_t pin_index;
    union {
        DigitalInConfig digital_in;
        DigitalOutConfig digital_out;
        AnalogInConfig analog_in;
        PwmConfig pwm;
        EndStopConfig end_stop;
    };
};
```

### Motor Configuration

```cpp
enum class MotorType : uint8_t {
    UNCONFIGURED = 0,
    CLEARPATH_SDSK,    // ClearPath with HLFB
    GENERIC_STEPPER    // Step/dir stepper
};

struct ClearPathConfig {
    uint8_t motor;                 // 0-3
    uint8_t enable_priority;       // Order for enabling (0 = first)
    bool enable_on_ready;          // Auto-enable when entering Ready state
    int8_t homing_end_stop;        // Pin for homing (-1 = none)
    int8_t far_end_stop;           // Pin for far limit (-1 = none)
    int8_t homing_dir;             // Direction for homing (-1 or 1)
};

struct GenericStepperConfig {
    uint8_t motor;
    uint8_t enable_priority;
    bool enable_on_ready;
    int8_t homing_end_stop;
    int8_t far_end_stop;
    int8_t homing_dir;
    uint32_t steps_per_unit;       // For unit conversion (optional)
};

struct MotorConfiguration {
    MotorType type;
    uint8_t motor_index;
    union {
        ClearPathConfig clearpath;
        GenericStepperConfig stepper;
    };

    // Runtime state
    bool is_enabled;
    bool is_homing;
    bool has_homed;
    int32_t position;              // Current position in steps
};
```

### Monitor Configuration

```cpp
enum class MonitorType : uint8_t {
    NONE = 0,
    DIGITAL_POLL,          // Report value changes
    ANALOG_POLL,           // Report value at interval
    ANALOG_THRESHOLD,      // Report when crossing threshold
    ANALOG_STOP_THRESHOLD  // Stop motors when crossing threshold
};

struct MonitorConfig {
    MonitorType type;
    uint8_t pin;
    uint32_t interval_ms;          // Polling interval
    int16_t threshold_low;
    int16_t threshold_high;
    bool trigger_error;            // Enter error state on trigger
    bool stop_motors;              // Stop all motors on trigger

    // Runtime state
    uint32_t last_report_ms;
    int16_t last_value;
    bool triggered;
};
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

### Monitor Commands

```
# Set up polling monitor for digital input
monitor_digital pin=6 [interval_ms=100]
→ ok
→ event type=input pin=6 value=1  # Async on change

# Set up polling monitor for analog input
monitor_analog pin=9 interval_ms=100
→ ok
→ event type=analog pin=9 value=2048  # Async at interval

# Set up threshold monitor
monitor_threshold pin=9 low=1000 high=3000 [trigger_error=0] [stop_motors=1]
→ ok
→ event type=threshold pin=9 value=950 trigger=low  # When crossed

# Remove monitor
unmonitor pin=6
→ ok

# List active monitors
list_monitors
→ ok count=2 monitors="6,9"
```

## Implementation Phases

### Phase 1: Core Infrastructure

**Files to create:**
- `CutterConfig.h` - Constants and limits
- `CutterState.h/cpp` - State machine
- `CommandParser.h/cpp` - Line parsing
- `CutterResponse.h/cpp` - Response formatting

**Functionality:**
- State machine transitions
- Command line parsing (no malloc)
- Key-value extraction
- Response string building
- Basic error handling

**Testing:**
- Unit tests for parser
- State transition tests

### Phase 2: Communication Layer

**Files to create:**
- `Cutter.h/cpp` - Main controller class
- `CommandDispatcher.cpp` - Command routing

**Functionality:**
- USB Serial integration
- Ethernet TCP server integration
- Input buffering (line accumulation)
- Output buffering
- Sequence number tracking
- Epoch validation
- Connection state management

**Testing:**
- USB echo test
- Ethernet connection test
- Sequence validation test

### Phase 3: Pin Configuration & Control

**Files to create:**
- `PinConfig.h`
- `PinCommands.cpp`

**Functionality:**
- Digital input configuration
- Digital output configuration (with timeout)
- Analog input configuration
- PWM configuration (pins 4, 5)
- H-Bridge configuration (pins 4, 5)
- End stop configuration
- Pin state reading/writing

**Testing:**
- Each pin type configuration
- Timeout behavior for digital outputs
- End stop triggering

### Phase 4: Motor Configuration & Basic Control

**Files to create:**
- `MotorConfig.h`
- `MotorCommands.cpp`

**Functionality:**
- ClearPath motor configuration
- Generic stepper configuration
- Motor parameter setting (vel, accel)
- Enable/disable sequencing
- Basic move commands (relative, absolute, velocity)
- Stop commands
- Position queries

**Testing:**
- Motor configuration
- Enable sequence with priorities
- Basic moves
- Stop behavior

### Phase 5: HLFB & Completion Notifications

**Functionality:**
- HLFB state monitoring for ClearPath
- StepsComplete monitoring for generic steppers
- Async "done" event generation
- Command completion correlation (seq numbers)

**Testing:**
- HLFB state changes
- Move completion events
- Sequence correlation

### Phase 6: Homing & End Stops

**Functionality:**
- Homing sequence implementation
- End stop monitoring during normal operation
- Error triggering on unexpected end stop activation
- Position zeroing after home

**Testing:**
- Homing sequence
- End stop error triggering
- Position after homing

### Phase 7: Monitoring System

**Files to create:**
- `Monitor.h`
- `MonitorManager.cpp`

**Functionality:**
- Digital input change monitoring
- Analog input polling
- Threshold monitoring with callbacks
- Error triggering from monitors
- Motor stop triggering from monitors

**Testing:**
- Digital change events
- Analog polling events
- Threshold crossing events
- Error and stop triggering

### Phase 8: Error Handling & Recovery

**Files to create:**
- `ErrorHandler.h/cpp`

**Functionality:**
- Error state management
- Error code definitions
- On-error pin behavior
- Reset and epoch increment
- Error recovery sequence

**Testing:**
- Error state entry from various triggers
- Pin behavior on error
- Reset and epoch handling

### Phase 9: Documentation & Examples

**Files to create:**
- Example applications
- API documentation
- Protocol specification document

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
        // Process incoming commands and generate responses
        cutter.Update();

        // Update is non-blocking, handles:
        // - Reading input from serial/ethernet
        // - Parsing complete lines
        // - Executing commands
        // - Generating responses
        // - Monitoring I/O
        // - Checking motor completion
        // - Managing timeouts
    }
}
```

## Thread Safety Considerations

The ClearCore runs single-threaded with a 5kHz ISR for hardware updates. Cutter integrates with this model:

1. **Main loop**: All command processing happens in `Update()` called from main loop
2. **ISR interaction**: Motor state and I/O are updated by ClearCore's ISR
3. **Atomic reads**: Motor position, HLFB state, and pin values are read atomically
4. **No locking needed**: Single-threaded execution model

## Performance Considerations

1. **Command parsing**: O(n) where n is command length, no allocations
2. **Response generation**: Fixed buffer, snprintf-style formatting
3. **Monitor updates**: Checked every `Update()` call, O(m) where m is monitor count
4. **Motor polling**: HLFB and StepsComplete checked every `Update()` call

## Memory Budget

```
Static allocations:
- Command buffer:        256 bytes
- Response buffer:       512 bytes
- Pin configs (13):      ~520 bytes (40 bytes each)
- Motor configs (4):     ~160 bytes (40 bytes each)
- Monitor configs (8):   ~192 bytes (24 bytes each)
- State/status:          ~128 bytes
- Parsing workspace:     ~512 bytes
─────────────────────────────────────
Total:                   ~2.3 KB

ClearCore has 256KB SRAM, so this is well within budget.
```

## Future Extensions (Not in Initial Scope)

- Multiple simultaneous connections
- Command queuing with timing
- Coordinated multi-axis moves
- Macro/script storage in flash
- Configuration persistence
- CAN bus support for CCIO-8 expansion
