# Cutter Implementation Tasks

## Phase 1: Core Infrastructure

### 1.1 Configuration Constants
- [ ] Create `libCutter/inc/CutterConfig.h`
  - Define `MAX_COMMAND_LENGTH`, `MAX_KEY_LENGTH`, `MAX_VALUE_LENGTH`
  - Define `MAX_PARAMS`, `MAX_MOTORS`, `MAX_DIGITAL_PINS`
  - Define `MAX_MONITORS`, `MAX_RESPONSE_LENGTH`
  - Pin capability mappings (which pins support which modes)

### 1.2 State Machine
- [ ] Create `libCutter/inc/CutterState.h`
  - `CutterState` enum
  - `CutterStatus` struct
  - State transition validation function declarations
- [ ] Create `libCutter/src/CutterState.cpp`
  - State transition logic
  - State name strings for responses
  - Valid transition matrix

### 1.3 Command Parser
- [ ] Create `libCutter/inc/CommandParser.h`
  - `ParsedCommand` struct with fixed-size arrays
  - `CommandParser` class declaration
  - Helper methods for type extraction
- [ ] Create `libCutter/src/CommandParser.cpp`
  - Line tokenization (handle comments, whitespace)
  - Key-value parsing
  - Integer/float/bool extraction
  - Command name extraction

### 1.4 Response Writer
- [ ] Create `libCutter/inc/CutterResponse.h`
  - `ResponseWriter` class with fixed buffer
  - Methods: `Ok()`, `Error()`, `Event()`, `Debug()`
  - Parameter appending methods
- [ ] Create `libCutter/src/ResponseWriter.cpp`
  - String formatting without allocation
  - Buffer management
  - Newline handling

---

## Phase 2: Communication Layer

### 2.1 Main Controller Class
- [ ] Create `libCutter/inc/Cutter.h`
  - `CutterController` class
  - Initialize methods for Serial/Ethernet
  - `Update()` method
  - State accessors
- [ ] Create `libCutter/src/Cutter.cpp`
  - Input line buffering
  - Command dispatch loop
  - Response sending
  - Connection state management

### 2.2 Command Dispatcher
- [ ] Create `libCutter/src/CommandDispatcher.cpp`
  - Command name lookup table
  - Dispatch to appropriate handler
  - Unknown command handling
  - Epoch/seq validation

### 2.3 System Commands
- [ ] Implement in `Cutter.cpp` or separate file:
  - `get_version` - Return protocol version
  - `get_state` - Return current state
  - `get_next_seq` - Return expected sequence
  - `ping` - Keepalive response
  - `reset` - Error recovery
  - `emergency_stop` - Manual error trigger
  - `set_debug` - Enable/disable debug output

---

## Phase 3: Pin Configuration & Control

### 3.1 Pin Configuration Structures
- [ ] Create `libCutter/inc/PinConfig.h`
  - `PinMode` enum
  - `DigitalInConfig`, `DigitalOutConfig` structs
  - `AnalogInConfig`, `PwmConfig` structs
  - `EndStopConfig` struct
  - `PinConfiguration` union wrapper

### 3.2 Pin Manager
- [ ] Create `libCutter/src/PinCommands.cpp`
  - `configure_digital_in` handler
  - `configure_digital_out` handler
  - `configure_analog_in` handler
  - `configure_pwm` handler
  - `configure_hbridge` handler
  - `configure_end_stop` handler

### 3.3 Pin Control Commands
- [ ] Add to `PinCommands.cpp`:
  - `set_output` - Set digital output
  - `get_input` - Read digital input
  - `get_analog` - Read analog value
  - `set_pwm` - Set PWM duty
  - `set_pwm_freq` - Set PWM frequency
  - `set_hbridge` - Set H-bridge output
  - `tone` - Generate tone

### 3.4 Digital Output Timeout
- [ ] Implement max_raised_ms timeout logic
  - Track when pin was raised
  - Check timeout in Update() loop
  - Auto-lower when expired

---

## Phase 4: Motor Configuration & Control

### 4.1 Motor Configuration Structures
- [ ] Create `libCutter/inc/MotorConfig.h`
  - `MotorType` enum
  - `ClearPathConfig`, `GenericStepperConfig` structs
  - `MotorConfiguration` union wrapper
  - Runtime state fields

### 4.2 Motor Configuration Commands
- [ ] Create `libCutter/src/MotorCommands.cpp`
  - `configure_sdsk` - ClearPath with HLFB
  - `configure_stepper` - Generic stepper
  - `set_motor_params` - Velocity/acceleration
  - `configuration_done` - Finalize config

### 4.3 Motor Enable Commands
- [ ] Add to `MotorCommands.cpp`:
  - `enable` - Enable all configured motors
  - `disable` - Disable all motors
  - `enable_motor` - Enable single motor
  - `disable_motor` - Disable single motor
  - Priority-based enable sequencing

### 4.4 Motor Move Commands
- [ ] Add to `MotorCommands.cpp`:
  - `move` - Relative move
  - `move_to` - Absolute move
  - `move_velocity` - Continuous velocity
  - `stop` - Decelerate to stop
  - `stop_immediate` - Hard stop
  - `stop_all` - Stop all motors

### 4.5 Motor Status Commands
- [ ] Add to `MotorCommands.cpp`:
  - `get_motor_status` - Query motor state
  - `set_position` - Set position without moving

---

## Phase 5: HLFB & Completion Notifications

### 5.1 HLFB Monitoring
- [ ] Add HLFB state tracking to motor runtime state
- [ ] Poll HLFB in Update() for ClearPath motors
- [ ] Generate `event type=hlfb` on state changes

### 5.2 Move Completion Detection
- [ ] Track in-progress moves with sequence numbers
- [ ] Check StepsComplete() for generic steppers
- [ ] Check HLFB for ClearPath move completion
- [ ] Generate `event type=done` when move completes

### 5.3 Sequence Correlation
- [ ] Associate seq number with each move command
- [ ] Include seq in completion events
- [ ] Handle multiple concurrent moves (different motors)

---

## Phase 6: Homing & End Stops

### 6.1 End Stop Monitoring
- [ ] Check end stop pins in Update()
- [ ] Trigger error if activated outside homing
- [ ] Direction-aware activation (only trigger if moving toward stop)

### 6.2 Homing Sequence
- [ ] Implement `home` command handler
- [ ] Move toward homing end stop at homing velocity
- [ ] Detect end stop activation
- [ ] Back off from end stop
- [ ] Zero position
- [ ] Generate `event type=homed`

### 6.3 Far End Stop
- [ ] Monitor far end stop during normal moves
- [ ] Trigger error if activated
- [ ] Include in error message which stop triggered

---

## Phase 7: Monitoring System

### 7.1 Monitor Configuration
- [ ] Create `libCutter/inc/Monitor.h`
  - `MonitorType` enum
  - `MonitorConfig` struct
- [ ] Create `libCutter/src/MonitorManager.cpp`
  - Monitor storage array
  - Add/remove monitor functions

### 7.2 Monitor Commands
- [ ] Add to MonitorManager.cpp:
  - `monitor_digital` - Digital input changes
  - `monitor_analog` - Analog polling
  - `monitor_threshold` - Threshold crossing
  - `unmonitor` - Remove monitor
  - `list_monitors` - List active monitors

### 7.3 Monitor Processing
- [ ] Poll monitors in Update()
- [ ] Check digital changes
- [ ] Check polling intervals
- [ ] Check threshold crossings
- [ ] Generate appropriate events
- [ ] Trigger errors/stops as configured

---

## Phase 8: Error Handling & Recovery

### 8.1 Error Handler
- [ ] Create `libCutter/inc/ErrorHandler.h`
  - `CutterError` enum with all error codes
  - Error state structure
- [ ] Create `libCutter/src/ErrorHandler.cpp`
  - Enter error state function
  - Error message formatting
  - Error code to string mapping

### 8.2 Error Triggers
- [ ] Error input pin triggering
- [ ] Analog threshold triggering
- [ ] End stop triggering
- [ ] Motor fault detection
- [ ] Supply voltage monitoring
- [ ] HLFB error states

### 8.3 Error Response
- [ ] Apply on_error values to digital outputs
- [ ] Stop PWM if configured
- [ ] Disable motors
- [ ] Clear pending commands
- [ ] Generate error event

### 8.4 Recovery
- [ ] Implement reset command
- [ ] Increment epoch on reset
- [ ] Reset sequence counter
- [ ] Transition to CONNECTED state
- [ ] Allow reconfiguration

---

## Phase 9: Examples & Testing

### 9.1 Basic USB Example
- [ ] Create `libCutter/examples/BasicUsb/BasicUsb.cpp`
  - Minimal setup
  - USB serial connection
  - Echo commands for testing

### 9.2 Ethernet Server Example
- [ ] Create `libCutter/examples/EthernetServer/EthernetServer.cpp`
  - TCP server setup
  - Client connection handling
  - Command processing

### 9.3 Full System Example
- [ ] Create `libCutter/examples/FullSystem/FullSystem.cpp`
  - Complete machine configuration
  - Multiple motors
  - End stops
  - Digital I/O
  - Analog monitoring

### 9.4 Host-Side Test Scripts
- [ ] Python test script for serial communication
- [ ] Test script for each command category
- [ ] Stress test for sequence handling

---

## Implementation Notes

### Static Allocation Pattern
```cpp
// All arrays are fixed size
PinConfiguration pin_configs[MAX_DIGITAL_PINS];
MotorConfiguration motor_configs[MAX_MOTORS];
MonitorConfig monitors[MAX_MONITORS];

// Strings use fixed buffers
char command_buffer[MAX_COMMAND_LENGTH];
char response_buffer[MAX_RESPONSE_LENGTH];
```

### Parser Pattern (No malloc)
```cpp
bool CommandParser::Parse(const char* line, ParsedCommand* cmd) {
    // Zero-initialize output struct
    memset(cmd, 0, sizeof(ParsedCommand));

    // Tokenize in-place using indices
    size_t pos = 0;
    // ... parse into fixed arrays
}
```

### Response Pattern
```cpp
ResponseWriter response(response_buffer, sizeof(response_buffer));
response.Ok()
    .Param("motor", motor_id)
    .Param("position", position)
    .Send(serial);
```

### Update Loop Pattern
```cpp
void CutterController::Update() {
    // 1. Read available input
    ReadInput();

    // 2. Parse complete lines
    if (HasCompleteLine()) {
        ProcessCommand();
    }

    // 3. Check motor completion
    CheckMotorCompletion();

    // 4. Check monitors
    CheckMonitors();

    // 5. Check timeouts
    CheckTimeouts();

    // 6. Send pending responses
    FlushOutput();
}
```
