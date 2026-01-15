# Cutter Development Plan

## Overview

This plan sequences the implementation to enable continuous testing throughout development. Each phase builds on the previous and includes tests before moving forward.

## Phase 0: Project Scaffolding

**Goal:** Set up build system and test infrastructure so we can test from day one.

### 0.1 Directory Structure
```
libCutter/
├── inc/
├── src/
├── test/
└── examples/
```

### 0.2 Build System
- [ ] Create `Makefile` with targets:
  - `test` - Build and run tests on host PC
  - `clearcore` - Cross-compile for ClearCore
  - `coverage` - Tests with coverage reporting
- [ ] Set up Google Test dependency
- [ ] Verify build works with empty test file

### 0.3 HAL Skeleton
- [ ] Create `inc/CutterHal.h` - Function declarations
- [ ] Create `src/CutterHal_ClearCore.cpp` - Stub implementations
- [ ] Create `test/CutterHal_Fake.cpp` - Fake implementations
- [ ] Create `test/FakeHal.h` - FakeHalState struct and helpers
- [ ] Create `test/TestSerial.h` - Fake serial port class
- [ ] Create `test/test_main.cpp` - Google Test entry point

**Exit Criteria:** `make test` runs and passes (with no real tests yet)

---

## Phase 1: Command Parser & Response Writer

**Goal:** Parse commands and format responses. Pure logic, fully testable.

### 1.1 Configuration Constants
- [ ] Create `inc/CutterConfig.h`
  - Buffer sizes, limits, pin capability tables

### 1.2 Command Parser
- [ ] Create `inc/CommandParser.h`
  - `ParsedCommand` struct
  - `CommandParser` class
- [ ] Create `src/CommandParser.cpp`
  - Skip comments and empty lines
  - Tokenize by whitespace
  - Extract command name
  - Parse `key=value` pairs
  - Extract epoch/seq if present
  - `GetInt()`, `GetBool()`, `GetString()` helpers

### 1.3 Parser Tests
- [ ] Create `test/test_parser.cpp`
  - Empty line, whitespace-only
  - Comments (`# ...`)
  - Simple command (`ping`)
  - Command with params (`move motor=0 steps=1000`)
  - Negative values (`steps=-500`)
  - Epoch and seq extraction
  - Missing param returns false
  - Too many params (overflow)

### 1.4 Response Writer
- [ ] Create `inc/CutterResponse.h`
  - `ResponseWriter` class with fixed buffer
- [ ] Create `src/ResponseWriter.cpp`
  - `Ok()`, `Error()`, `Event()`, `Debug()`
  - Chained `.Param(key, value)` methods
  - Buffer overflow protection

### 1.5 Response Tests
- [ ] Create `test/test_response.cpp`
  - `ok` response
  - `ok` with params
  - `error` with code and message
  - `event` with type and params
  - Buffer overflow handling

**Exit Criteria:** All parser and response tests pass

---

## Phase 2: State Machine & Controller Shell

**Goal:** Skeleton controller with state machine, can respond to `ping`.

### 2.1 State Machine
- [ ] Create `inc/CutterState.h`
  - `CutterState` enum
  - `CutterStatus` struct
- [ ] Create `src/CutterState.cpp`
  - State transition validation
  - State name strings

### 2.2 State Machine Tests
- [ ] Create `test/test_state_machine.cpp`
  - Valid transitions
  - Invalid transitions rejected
  - Error state reachable from all states

### 2.3 Error Codes
- [ ] Create `inc/ErrorHandler.h`
  - `CutterError` enum (100-699 ranges)

### 2.4 Controller Shell
- [ ] Create `inc/Cutter.h`
  - `CutterController` class declaration
  - `Initialize()`, `Update()`, `GetState()`
- [ ] Create `src/Cutter.cpp`
  - Line buffering (accumulate until `\n`)
  - Parse and dispatch skeleton
  - System commands: `ping`, `get_version`, `get_state`, `get_next_seq`

### 2.5 Controller Tests
- [ ] Add to `test/test_integration.cpp`
  - Initialize controller
  - `ping` → `ok`
  - `get_version` → `ok version=...`
  - `get_state` → `ok state=connected`
  - Unknown command → `error`

**Exit Criteria:** Controller responds to basic system commands

---

## Phase 3: Pin Configuration & Digital I/O

**Goal:** Configure and control digital pins.

### 3.1 Pin State Structures
- [ ] Create `inc/PinState.h`
  - `PinMode` enum
  - Type-specific structs: `DigitalInState`, `DigitalOutState`, etc.
  - `PinSlot` tagged union
- [ ] Create `src/PinState.cpp`
  - `PinCheck::DigitalIn()` - change detection, error triggers
  - `PinCheck::DigitalOut()` - timeout handling
  - `PinSlot::Check()` dispatcher

### 3.2 Pin Commands
- [ ] Create `src/PinCommands.cpp`
  - `configure_digital_in`
  - `configure_digital_out`
  - `set_output`
  - `get_input`
  - `configuration_done` (partial - for pins only)

### 3.3 Pin Tests
- [ ] Create `test/test_pins.cpp`
  - Configure digital input
  - Configure digital output
  - Read input value
  - Set output value
  - Digital input change reporting
  - Digital input error trigger
  - Digital output timeout

**Exit Criteria:** Can configure and control digital I/O with all tests passing

---

## Phase 4: Analog Input & Thresholds

**Goal:** Analog input with threshold monitoring.

### 4.1 Analog Implementation
- [ ] Add `AnalogInState` to `PinState.h`
- [ ] Add `PinCheck::AnalogIn()` to `PinState.cpp`
  - Error threshold checking
  - Stop threshold checking
  - Interval reporting

### 4.2 Analog Commands
- [ ] Add to `PinCommands.cpp`:
  - `configure_analog_in`
  - `get_analog`
  - `set_reporting`

### 4.3 Analog Tests
- [ ] Add to `test/test_pins.cpp`
  - Configure analog input
  - Read analog value
  - Error threshold triggers error state
  - Stop threshold stops motors (stub for now)
  - Interval reporting generates events

**Exit Criteria:** Analog input with thresholds works

---

## Phase 5: PWM & H-Bridge

**Goal:** PWM and H-Bridge output including tone generation.

### 5.1 PWM/H-Bridge Implementation
- [ ] Add `PwmState`, `HBridgeState` to `PinState.h`
- [ ] Add `PinCheck::HBridge()` to `PinState.cpp`
  - Tone timeout handling

### 5.2 PWM/H-Bridge Commands
- [ ] Add to `PinCommands.cpp`:
  - `configure_pwm`
  - `configure_hbridge`
  - `set_pwm`
  - `set_pwm_freq`
  - `set_hbridge`
  - `tone`

### 5.3 PWM/H-Bridge Tests
- [ ] Add to `test/test_pins.cpp`
  - Configure PWM (pins 0-5)
  - Configure H-Bridge (pins 4-5 only)
  - Reject H-Bridge on pins 0-3
  - Set PWM duty
  - Set H-Bridge value
  - Tone with duration generates done event

**Exit Criteria:** All pin types work

---

## Phase 6: Motor Configuration

**Goal:** Configure motors (no movement yet).

### 6.1 Motor State
- [ ] Create `inc/MotorState.h`
  - `MotorType` enum
  - `MotorState` struct
- [ ] Create `src/MotorState.cpp`
  - `MotorState::Check()` skeleton

### 6.2 Motor Configuration Commands
- [ ] Create `src/MotorCommands.cpp`
  - `configure_sdsk`
  - `configure_stepper`
  - `set_motor_params`
  - Update `configuration_done` for motors

### 6.3 Motor Configuration Tests
- [ ] Create `test/test_motors.cpp`
  - Configure ClearPath motor
  - Configure generic stepper
  - Set motor parameters
  - `configuration_done` transitions to CONFIGURED

**Exit Criteria:** Motors can be configured, state transitions to CONFIGURED

---

## Phase 7: Motor Enable & Basic Motion

**Goal:** Enable motors and execute moves.

### 7.1 Enable Logic
- [ ] Add to `MotorCommands.cpp`:
  - `enable` - enable all, priority order
  - `disable` - disable all
  - `enable_motor` - single motor
  - `disable_motor` - single motor
- [ ] Add to `MotorState.cpp`:
  - Ready detection (HLFB or stepper ready)
  - ENABLING → READY transition

### 7.2 Move Commands
- [ ] Add to `MotorCommands.cpp`:
  - `move` - relative
  - `move_to` - absolute
  - `move_velocity` - continuous
  - `stop` - decelerate
  - `stop_immediate` - hard stop
  - `stop_all`

### 7.3 Completion Detection
- [ ] Add to `MotorState::Check()`:
  - Poll StepsComplete() / HLFB
  - Generate `event type=done`
  - HLFB change events

### 7.4 Status Commands
- [ ] Add to `MotorCommands.cpp`:
  - `get_motor_status`
  - `set_position`

### 7.5 Motor Motion Tests
- [ ] Add to `test/test_motors.cpp`
  - Enable sequence
  - Move and completion event
  - Velocity move and stop
  - Get motor status
  - HLFB state changes (ClearPath)

**Exit Criteria:** Motors can be enabled, moved, and report completion

---

## Phase 8: End Stops & Homing

**Goal:** End stop configuration and homing sequence.

### 8.1 End Stop Implementation
- [ ] Add `EndStopState` handling to `PinState.cpp`
  - `PinCheck::EndStop()` - detect activation
  - Signal to motor during homing
  - Error if activated outside homing

### 8.2 End Stop Commands
- [ ] Add to `PinCommands.cpp`:
  - `configure_end_stop`

### 8.3 Homing Implementation
- [ ] Add to `MotorState::Check()`:
  - Homing state machine
  - Seek → hit → backoff → (latch → hit →) zero
- [ ] Add homing parameters to `configure_stepper`:
  - `homing_seek_velocity`
  - `homing_latch_velocity`
  - `homing_accel`

### 8.4 Homing Command
- [ ] Add to `MotorCommands.cpp`:
  - `home`

### 8.5 Homing Tests
- [ ] Create `test/test_homing.cpp`
  - Configure end stop
  - Start homing
  - Simulate end stop hit
  - Verify backoff
  - Verify position zeroed
  - End stop during normal move → error

**Exit Criteria:** Homing works for generic steppers

---

## Phase 9: Error Handling & Recovery

**Goal:** Complete error handling system.

### 9.1 Error Entry
- [ ] Create `src/ErrorHandler.cpp`
  - `EnterError()` method
  - Stop all motors
  - Apply `on_error` to digital outputs
  - Stop PWM/H-Bridge where configured
  - Generate error event

### 9.2 Error Commands
- [ ] Add to `Cutter.cpp`:
  - `emergency_stop`
  - `reset`

### 9.3 Epoch/Sequence Validation
- [ ] Add to command dispatcher:
  - Validate epoch matches
  - Validate seq is expected
  - Reject stale commands

### 9.4 Error Tests
- [ ] Create `test/test_errors.cpp`
  - Digital input error trigger → error state
  - Analog threshold exceeded → error state
  - End stop unexpected → error state
  - `emergency_stop` → error state
  - `on_error` values applied
  - `reset` → CONNECTED, epoch incremented
  - Wrong epoch rejected
  - Wrong seq rejected

**Exit Criteria:** Full error handling and recovery works

---

## Phase 10: Soft Limits

**Goal:** Software position limits.

### 10.1 Soft Limit Implementation
- [ ] Add to `MotorState`:
  - `soft_limit_min`, `soft_limit_max`
- [ ] Add to move commands:
  - Reject moves outside limits
- [ ] Add to `MotorState::Check()`:
  - Stop velocity moves at limits

### 10.2 Soft Limit Commands
- [ ] Add to `MotorCommands.cpp`:
  - `set_soft_limits motor=N min=M max=X`
  - `clear_soft_limits motor=N`

### 10.3 Soft Limit Tests
- [ ] Add to `test/test_motors.cpp`
  - Configure soft limits
  - Move within limits succeeds
  - Move outside limits rejected
  - Velocity move stops at limit

**Exit Criteria:** Soft limits work

---

## Phase 11: Integration & Examples

**Goal:** Complete integration testing and examples.

### 11.1 Full Integration Tests
- [ ] Expand `test/test_integration.cpp`
  - Full workflow: configure → enable → home → move → done
  - Error and recovery cycle
  - Multiple motors
  - Pin monitoring during motion

### 11.2 Examples
- [ ] Create `examples/BasicUsb/BasicUsb.cpp`
- [ ] Create `examples/EthernetServer/EthernetServer.cpp`
- [ ] Create `examples/FullSystem/FullSystem.cpp`

### 11.3 Hardware Testing
- [ ] Test on actual ClearCore hardware
- [ ] Verify timing behavior
- [ ] Verify HLFB detection
- [ ] Verify end stop behavior

**Exit Criteria:** All tests pass, examples work on hardware

---

## Phase 12: Documentation & Polish

**Goal:** Final documentation and cleanup.

### 12.1 Documentation
- [ ] API reference
- [ ] Protocol specification
- [ ] Example configurations

### 12.2 Code Cleanup
- [ ] Review all TODOs
- [ ] Consistent error messages
- [ ] Code comments where needed

---

## Dependency Graph

```
Phase 0 (Scaffolding)
    │
    ▼
Phase 1 (Parser/Response)
    │
    ▼
Phase 2 (State Machine/Shell)
    │
    ├────────────────┬────────────────┐
    ▼                ▼                ▼
Phase 3          Phase 6          Phase 9
(Digital I/O)    (Motor Config)   (Error Handling)
    │                │                │
    ▼                ▼                │
Phase 4          Phase 7 ◄───────────┘
(Analog)         (Motor Motion)
    │                │
    ▼                ▼
Phase 5          Phase 8
(PWM/H-Bridge)   (End Stops/Homing)
    │                │
    └────────┬───────┘
             ▼
         Phase 10
         (Soft Limits)
             │
             ▼
         Phase 11
         (Integration)
             │
             ▼
         Phase 12
         (Documentation)
```

## Estimated Complexity

| Phase | Files | Tests | Complexity |
|-------|-------|-------|------------|
| 0. Scaffolding | 6 | 0 | Low |
| 1. Parser/Response | 4 | 15+ | Low |
| 2. State Machine | 4 | 10+ | Low |
| 3. Digital I/O | 2 | 10+ | Medium |
| 4. Analog | 1 | 8+ | Low |
| 5. PWM/H-Bridge | 1 | 8+ | Medium |
| 6. Motor Config | 2 | 6+ | Low |
| 7. Motor Motion | 1 | 12+ | High |
| 8. Homing | 1 | 8+ | High |
| 9. Error Handling | 1 | 10+ | Medium |
| 10. Soft Limits | 1 | 5+ | Low |
| 11. Integration | 3 | 10+ | Medium |
| 12. Documentation | - | - | Low |

## Success Metrics

- All tests pass on host PC
- All tests pass equivalent scenarios on hardware
- No memory allocation after initialization
- Response latency < 1ms for simple commands
- Motor completion events within one Update() cycle of hardware completion
