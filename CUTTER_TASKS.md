# Cutter Implementation Tasks

## Phase 1: Core Infrastructure

### 1.1 Configuration Constants
- [ ] Create `libCutter/inc/CutterConfig.h`
  - Define `MAX_COMMAND_LENGTH`, `MAX_KEY_LENGTH`, `MAX_VALUE_LENGTH`
  - Define `MAX_PARAMS`, `NUM_PINS`, `NUM_MOTORS`
  - Define `MAX_RESPONSE_LENGTH`
  - Pin capability tables (which pins support which modes)

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
  - Helper methods for type extraction (GetInt, GetBool, GetString)
- [ ] Create `libCutter/src/CommandParser.cpp`
  - Skip comments and empty lines
  - Tokenize by whitespace
  - Extract command name
  - Parse key=value pairs into params array
  - Integer/float/bool extraction helpers

### 1.4 Response Writer
- [ ] Create `libCutter/inc/CutterResponse.h`
  - `ResponseWriter` class with fixed buffer
  - Methods: `Ok()`, `Error()`, `Event()`, `Debug()`
  - Chained parameter appending
- [ ] Create `libCutter/src/ResponseWriter.cpp`
  - String formatting without allocation
  - Buffer management with overflow protection
  - Newline termination

### 1.5 Error Codes
- [ ] Create `libCutter/inc/ErrorHandler.h`
  - `CutterError` enum with all error codes (100-699 ranges)
  - Error code to string helper

---

## Phase 2: Communication & Main Loop

### 2.1 Main Controller Class
- [ ] Create `libCutter/inc/Cutter.h`
  - `CutterController` class
  - `PinState pins_[NUM_PINS]` array
  - `MotorState motors_[NUM_MOTORS]` array
  - `CutterStatus status_`
  - `Initialize(ISerial*)` for USB
  - `Initialize(EthernetTcpServer*)` for Ethernet
  - `Update()` method
- [ ] Create `libCutter/src/Cutter.cpp`
  - Input line buffering (accumulate until newline)
  - `Update()` loop: ReadInput → ParseLine → CheckPins → CheckMotors → FlushOutput
  - Connection state detection

### 2.2 Command Dispatcher
- [ ] Create `libCutter/src/CommandDispatcher.cpp`
  - Command name to handler mapping (static table or switch)
  - Epoch validation (reject if mismatch)
  - Sequence number validation (reject if not expected)
  - Unknown command handling

### 2.3 System Commands
- [ ] Implement in `Cutter.cpp`:
  - `ping` → `ok`
  - `get_version` → `ok version=1.0 protocol=cutter ...`
  - `get_state` → `ok state=ready epoch=0 seq=123`
  - `get_next_seq` → `ok epoch=0 seq=124`
  - `set_debug enabled=1` → enable/disable debug output

---

## Phase 3: Pin State & Configuration

### 3.1 Pin State Structure (Tagged Union)
- [ ] Create `libCutter/inc/PinState.h`
  - `PinMode` enum
  - Type-specific state structs:
    - `DigitalInState` (invert, error_trigger, report_changes, last_value)
    - `DigitalOutState` (max_raised_ms, on_error, current_value, raised_at_ms)
    - `AnalogInState` (thresholds, report_interval, last_value, etc.)
    - `PwmState` (stop_on_error, duty, frequency)
    - `HBridgeState` (stop_on_error, value, tone state)
    - `EndStopState` (motor_index, direction, active_low, is_triggered)
  - `PinSlot` struct with mode + pin_index + union of above
  - Method declarations: `Check()`, `ApplyErrorState()`, `Read()`, `Write()`
- [ ] Create `libCutter/src/PinState.cpp`
  - `namespace PinCheck` with type-specific functions:
    - `DigitalIn()` - change detection, error triggers
    - `DigitalOut()` - timeout handling
    - `AnalogIn()` - threshold checks, interval reporting
    - `EndStop()` - homing coordination, unexpected activation
    - `HBridge()` - tone timeout
  - `PinSlot::Check()` - dispatches to type-specific function
  - `PinSlot::ApplyErrorState()` - apply on_error for outputs

### 3.2 Pin Configuration Commands
- [ ] Add to `PinCommands.cpp`:
  - `configure_digital_in pin=N [error_trigger=V] [invert=0] [report_changes=0]`
  - `configure_digital_out pin=N [max_raised_ms=0] [on_error=-1]`
  - `configure_analog_in pin=N [error_low=INT16_MIN] [error_high=INT16_MAX] [stop_low=INT16_MIN] [stop_high=INT16_MAX] [report_interval_ms=0] [report_threshold=0]`
  - `configure_pwm pin=N [stop_on_error=1] [amplitude=INT16_MAX]`
  - `configure_hbridge pin=N [stop_on_error=1]`
  - Validate pin capabilities for each mode

### 3.3 Pin Control Commands
- [ ] Add to `PinCommands.cpp`:
  - `set_output pin=N value=V` - set digital output
  - `get_input pin=N` → `ok pin=N value=V`
  - `get_analog pin=N` → `ok pin=N value=V`
  - `set_pwm pin=N duty=V`
  - `set_pwm_freq pin=N freq=V`
  - `set_hbridge pin=N value=V`
  - `tone pin=N freq=F [duration_ms=D]`
  - `set_reporting pin=N [report_changes=...] [report_interval_ms=...]`

### 3.4 Pin Check Logic (in Update loop)
- [ ] Digital input: detect changes, send events, check error triggers
- [ ] Digital output: check max_raised_ms timeout
- [ ] Analog input: check thresholds, send interval reports

---

## Phase 4: Motor State & Configuration

### 4.1 Motor State Structure
- [ ] Create `libCutter/inc/MotorState.h`
  - `MotorType` enum
  - `MotorState` struct with config + runtime fields
  - `Check()` method declaration
- [ ] Create `libCutter/src/MotorState.cpp`
  - `Check()` implementation
  - `IsReady()` - check HLFB or stepper ready state
  - `IsMoveComplete()` - check StepsComplete or HLFB

### 4.2 Motor Configuration Commands
- [ ] Add to `MotorCommands.cpp`:
  - `configure_sdsk motor=N [enable_priority=0] [enable_on_ready=1] [homing_end_stop=-1] [far_end_stop=-1] [homing_dir=-1]`
  - `configure_stepper motor=N [enable_priority=0] [enable_on_ready=1] [homing_end_stop=-1] [far_end_stop=-1] [homing_dir=-1] [steps_per_unit=0]`
  - `set_motor_params motor=N vel_max=V accel_max=A [vel_limit=L]`
  - `configuration_done` - validate config, transition to CONFIGURED

---

## Phase 5: Motor Control & Completion

### 5.1 Enable/Disable
- [ ] `enable` - enable all motors with enable_on_ready=1, in priority order
- [ ] `disable` - disable all motors
- [ ] `enable_motor motor=N` - enable single motor
- [ ] `disable_motor motor=N` - disable single motor
- [ ] Track ENABLING state, transition to READY when all motors ready

### 5.2 Move Commands
- [ ] `move motor=N steps=S [vel=V] [accel=A]` - relative move
- [ ] `move_to motor=N position=P [vel=V] [accel=A]` - absolute move
- [ ] `move_velocity motor=N velocity=V` - continuous velocity
- [ ] Track active_move_seq for completion correlation
- [ ] Set is_moving flag

### 5.3 Stop Commands
- [ ] `stop motor=N [accel=A]` - decelerate to stop
- [ ] `stop_immediate motor=N` - hard stop
- [ ] `stop_all [immediate=0]` - stop all motors

### 5.4 Status Commands
- [ ] `get_motor_status motor=N` → position, velocity, enabled, homed, moving, hlfb
- [ ] `set_position motor=N position=P` - set position without moving

### 5.5 Completion Detection (in Check loop)
- [ ] Poll HLFB for ClearPath motors
- [ ] Poll StepsComplete() for generic steppers
- [ ] Send `event type=done motor=N seq=S` when move completes
- [ ] Send `event type=hlfb motor=N state=S` on HLFB changes

---

## Phase 6: Homing & End Stops

### 6.1 End Stop Configuration
- [ ] `configure_end_stop pin=N motor=M [direction=D] [active_low=1]`
- [ ] Link pin to motor in PinState
- [ ] Set pin mode to END_STOP

### 6.2 End Stop Monitoring
- [ ] In PinState::Check(), if mode==END_STOP:
  - Check if motor is moving toward this stop
  - If activated outside homing, trigger error
  - During homing, signal stop reached

### 6.3 Homing Command
- [ ] `home motor=N [velocity=V] [backoff=B]`
- [ ] Set is_homing flag
- [ ] Start velocity move toward homing_end_stop
- [ ] Detect end stop activation
- [ ] Stop motor
- [ ] Back off by backoff steps
- [ ] Zero position
- [ ] Clear is_homing, set has_homed
- [ ] Send `event type=homed motor=N seq=S position=0`

---

## Phase 7: Error Handling & Recovery

### 7.1 Enter Error State
- [ ] `EnterError(code, message)` method
- [ ] Transition to ERROR state
- [ ] Stop all motors
- [ ] Apply on_error values to all digital outputs
- [ ] Stop PWM/H-Bridge where stop_on_error=1
- [ ] Send `event type=error code=N message="..."`

### 7.2 Error Triggers
- [ ] Digital input error_trigger
- [ ] Analog input error_threshold
- [ ] End stop activated outside homing
- [ ] Motor fault detection
- [ ] HLFB error states

### 7.3 Manual Error
- [ ] `emergency_stop [message="reason"]` - enter error state manually

### 7.4 Recovery
- [ ] `reset` command
- [ ] Increment epoch
- [ ] Reset sequence counter to 0
- [ ] Clear error state
- [ ] Transition to CONNECTED state
- [ ] All pins/motors marked unconfigured (or just disabled?)

---

## Phase 8: Examples & Testing

### 8.1 Basic USB Example
- [ ] Create `libCutter/examples/BasicUsb/BasicUsb.cpp`
  - Initialize SysManager
  - Initialize Cutter with ConnectorUsb
  - Main loop calls Update()

### 8.2 Ethernet Server Example
- [ ] Create `libCutter/examples/EthernetServer/EthernetServer.cpp`
  - Configure static IP or DHCP
  - Open TCP server on port
  - Accept connections
  - Initialize Cutter with client

### 8.3 Full System Example
- [ ] Create `libCutter/examples/FullSystem/FullSystem.cpp`
  - Configure 2 ClearPath motors
  - Configure end stops
  - Configure digital I/O
  - Configure analog input with thresholds
  - Demonstrate homing sequence
  - Demonstrate move with completion

### 8.4 Python Test Scripts
- [ ] Create `test/test_basic.py` - ping, version, state
- [ ] Create `test/test_pins.py` - digital/analog config and control
- [ ] Create `test/test_motors.py` - motor config, moves, completion
- [ ] Create `test/test_errors.py` - error triggers, recovery
- [ ] Create `test/test_stress.py` - sequence handling under load

---

## Implementation Notes

### Static Allocation Pattern
```cpp
// All arrays are fixed size, no malloc
PinSlot pins_[NUM_PINS];         // 13 pins (tagged union)
MotorState motors_[NUM_MOTORS];  // 4 motors

// Strings use fixed buffers
char line_buffer_[MAX_COMMAND_LENGTH];
char response_buffer_[MAX_RESPONSE_LENGTH];
```

### Tagged Union Pattern
```cpp
struct PinSlot {
    PinMode mode;
    uint8_t pin_index;
    union {
        DigitalInState digital_in;
        DigitalOutState digital_out;
        AnalogInState analog_in;
        // ... etc
    };
};

// Type-specific check functions in namespace
namespace PinCheck {
    void DigitalIn(PinSlot* slot, uint32_t now, CutterController* ctrl);
    void DigitalOut(PinSlot* slot, uint32_t now, CutterController* ctrl);
    // ... etc
}

// Dispatcher
void PinSlot::Check(uint32_t now, CutterController* ctrl) {
    switch (mode) {
        case PinMode::DIGITAL_IN: PinCheck::DigitalIn(this, now, ctrl); break;
        // ... etc
    }
}
```

### Timing Pattern (Rollover-Safe)
```cpp
// Use subtraction for elapsed time - handles uint32_t rollover correctly
uint32_t start = Milliseconds();
// ... later ...
if ((Milliseconds() - start) >= timeout_ms) {
    // Timeout elapsed
}
```

### Main Loop Pattern
```cpp
void CutterController::Update() {
    uint32_t now = Milliseconds();

    // Read/parse input...

    // Each configured object checks itself
    for (uint8_t i = 0; i < NUM_PINS; i++) {
        pins_[i].Check(now, this);  // No-op if UNCONFIGURED
    }
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors_[i].Check(now, this);
    }

    FlushOutput();
}
```
