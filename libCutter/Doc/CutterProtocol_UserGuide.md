# Cutter Protocol User Guide

## Introduction

The Cutter Protocol is a text-based command protocol for the Teknic ClearCore I/O and motion controller, designed for controlling ClearPath-SD/SK integrated servo motors, generic stepper motors, and ClearCore I/O points. Commands are sent as human-readable text lines, and responses are returned in a structured format suitable for both human operators and automated control systems.

**Important Note:** The Cutter firmware must be downloaded onto a ClearCore before use. ClearCore is not pre-loaded with any program.

### Features

The Cutter Protocol implements the following functionality:

**Motor Control**
- Controlling ClearPath-SD/SK motors and generic stepper motors
- Enabling and disabling motors (individually or all at once)
- Commanding absolute or relative positional moves
- Commanding velocity moves with soft limit protection
- Automatic homing with configurable parameters
- Per-move velocity and acceleration overrides
- Native ClearCore limit switch support
- E-Stop (emergency stop) integration

**Motor Monitoring**
- Real-time position tracking
- HLFB (High-Level Feedback) monitoring for ClearPath motors
- Soft limit enforcement during velocity moves
- Automatic fault detection and reporting

**I/O Control**
- Digital input with change reporting and edge detection
- Digital output with timeout protection
- Analog input with threshold monitoring
- PWM output with configurable duty cycle
- H-Bridge output for bidirectional DC motor control
- Tone generation for audio feedback

**Protocol Features**
- Command correlation via epoch/sequence numbers
- Asynchronous event notification
- State machine with error recovery
- Pin conflict detection

### Communication

The protocol accepts input and sends output via USB serial by default. It can be configured to use other ClearCore communication interfaces including COM ports, Ethernet, or XBee connections.

Commands are sent as single lines terminated by newline (`\n`). Responses are sent immediately after command processing. Asynchronous events (move completion, errors, etc.) are sent when they occur.

---

## Additional Resources

**ClearCore Software Documentation:**
https://teknic-inc.github.io/ClearCore-library/

**ClearCore Manual:**
https://www.teknic.com/files/downloads/clearcore_user_manual.pdf

**ClearPath Manual (DC Power):**
https://www.teknic.com/files/downloads/clearpath_user_manual.pdf

**ClearPath Manual (AC Power):**
https://www.teknic.com/files/downloads/ac_clearpath-mc-sd_manual.pdf

---

## Command Format

### General Syntax

Commands follow a `command_name param=value param=value ...` format:

```
command_name [param1=value1] [param2=value2] ...
```

**Key formatting rules:**

- Command names are lowercase with underscores (e.g., `configure_digital_in`)
- Parameters are `key=value` pairs separated by spaces
- String values are unquoted (e.g., `rate=normal`)
- Boolean values are `true` or `false`
- Numeric values are integers (no decimal support)
- Parameter order does not matter
- Unknown parameters are ignored

### Command Correlation

Commands support optional `epoch` and `seq` (sequence) parameters for tracking asynchronous operations:

```
move motor=0 steps=1000 epoch=5 seq=42
```

**Epoch** - Increments on each error. Commands with mismatched epoch are rejected.

**Sequence (seq)** - User-provided command identifier. Returned in async events to correlate results with commands.

When a motor move completes, the event includes the original epoch/seq:
```
event type=done motor=0 epoch=5 seq=42 position=1000
```

### Response Format

**Success Response:**
```
ok [param=value ...]
```

**Error Response:**
```
error code=<error_code> message="<error_message>" [param=value ...]
```

**Event (Asynchronous):**
```
event type=<event_type> [param=value ...]
```

---

## State Machine

The controller operates through the following states:

| State | Description |
|-------|-------------|
| UNCONNECTED | No host connection established |
| CONNECTED | Host connected, awaiting configuration |
| CONFIGURED | Pins/motors configured, not yet enabled |
| ENABLING | Motors enabling (waiting for HLFB on ClearPath) |
| READY | System ready for motion commands |
| WORKING | Motion in progress |
| ERROR | Error state, requires reset |

### State Transitions

```
UNCONNECTED -> CONNECTED (on first command)
CONNECTED -> CONFIGURED (on first configure command)
CONFIGURED -> ENABLING (on enable command)
ENABLING -> READY (when all motors ready)
READY <-> WORKING (on motion start/complete)
Any -> ERROR (on fault)
ERROR -> CONNECTED (on reset command)
```

---

## Commands Reference

### System Commands

| Command | Description | Example |
|---------|-------------|---------|
| `ping` | Test connection | `ping` |
| `reset` | Reset from error state | `reset` |
| `status` | Query system status | `status` |
| `version` | Get firmware version | `version` |
| `emergency_stop` | Emergency stop all motors | `emergency_stop` |
| `get_next_seq` | Get next sequence number | `get_next_seq` |

#### ping

Test connection to the controller. Returns `ok` if connected.

```
-> ping
<- ok
```

#### reset

Reset from ERROR state. Clears the error and transitions to CONNECTED state. The epoch is incremented.

```
-> reset
<- ok epoch=2
```

#### status

Query system status including state, epoch, and active motors.

```
-> status
<- ok state=READY epoch=1
```

#### version

Get firmware and protocol version.

```
-> version
<- ok version="1.0.0" protocol="1"
```

#### emergency_stop

Immediately stop all motors and enter ERROR state.

```
-> emergency_stop
<- ok
```

---

### Motor Configuration Commands

| Command | Description | Required Params |
|---------|-------------|-----------------|
| `configure_sdsk` | Configure ClearPath-SD/SK motor | `motor` |
| `configure_stepper` | Configure generic stepper | `motor` |
| `set_motor_clock` | Set step clock rate (global) | `rate` |
| `configure_estop` | Configure E-Stop pin | `pin`, `motor` |

#### configure_sdsk

Configure a ClearPath-SD/SK servo motor. Uses HLFB for position confirmation.

**MSP Configuration Requirements:**

The motor must be configured in Teknic's MSP (Motor Setup Program) with:
- **HLFB Mode**: "ASG - w/Measured Torque" (recommended) or "All Systems Go (ASG)"
- **Input Mode**: "Step and Direction"

The "ASG - w/Measured Torque" mode provides bipolar PWM feedback that indicates both
position confirmation (ASG) and torque level. This allows Cutter to detect when moves
complete and when the motor hits torque limits during homing.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor` | int | required | Motor index (0-3) |
| `vel_max` | int | 10000 | Maximum velocity (steps/sec) |
| `accel_max` | int | 100000 | Maximum acceleration (steps/sec²) |
| `hlfb_timeout` | int | 5000 | HLFB timeout in ms |
| `enable_priority` | int | motor | Enable/home order (lower = first) |
| `homing_mode` | string | msp | Homing mode (see below) |
| `homing_direction` | int | -1 | Homing direction (-1 or 1) |
| `homing_seek_velocity` | int | 5000 | Fast approach velocity (limit_switch mode) |
| `homing_latch_velocity` | int | 500 | Slow precision velocity (limit_switch mode) |
| `homing_backoff` | int | 200 | Backoff distance after contact (limit_switch mode) |
| `limit_neg_pin` | int | none | Negative limit switch pin (limit_switch mode) |
| `limit_pos_pin` | int | none | Positive limit switch pin (limit_switch mode) |
| `soft_limits` | bool | false | Enable soft limits |
| `soft_min` | int | INT32_MIN | Minimum position |
| `soft_max` | int | INT32_MAX | Maximum position |

**Homing Modes for SDSK:**
- `none`: No homing performed. Motor is ready immediately after HLFB asserts.
- `msp`: Motor homes itself via MSP (Motion Setup Program) configuration. Cutter just enables and waits for HLFB. This is the default.
- `limit_switch`: Cutter performs homing using a limit switch, same as stepper motors.

```
-> configure_sdsk motor=0 vel_max=20000 accel_max=100000
<- ok motor=0

-> configure_sdsk motor=0 homing_mode=none
<- ok motor=0

-> configure_sdsk motor=0 homing_mode=limit_switch limit_neg_pin=6
<- ok motor=0
```

#### configure_stepper

Configure a generic stepper motor. Uses ClearCore native limit switches for homing.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor` | int | required | Motor index (0-3) |
| `vel_max` | int | 10000 | Maximum velocity (steps/sec) |
| `accel_max` | int | 100000 | Maximum acceleration (steps/sec²) |
| `enable_priority` | int | motor | Enable/home order |
| `homing_mode` | string | limit_switch | Homing mode (see below) |
| `homing_direction` | int | -1 | Homing direction (-1 or 1) |
| `homing_seek_velocity` | int | 5000 | Fast approach velocity |
| `homing_latch_velocity` | int | 500 | Slow precision velocity |
| `homing_backoff` | int | 200 | Backoff distance after contact |
| `limit_neg_pin` | int | none | Negative limit switch pin |
| `limit_pos_pin` | int | none | Positive limit switch pin |
| `soft_limits` | bool | false | Enable soft limits |
| `soft_min` | int | INT32_MIN | Minimum position |
| `soft_max` | int | INT32_MAX | Maximum position |

**Homing Modes for Stepper:**
- `none`: No homing performed. Motor is ready immediately when enabled.
- `limit_switch`: Cutter performs homing using a limit switch. This is the default.

**Note:** If `homing_mode=limit_switch`, the limit switch in the homing direction must be configured.

```
-> configure_stepper motor=1 vel_max=5000 limit_neg_pin=6 homing_direction=-1
<- ok motor=1

-> configure_stepper motor=1 homing_mode=none
<- ok motor=1
```

#### set_motor_clock

Set the global motor step clock rate. Affects all motors.

| Parameter | Type | Options | Description |
|-----------|------|---------|-------------|
| `rate` | string | required | `low`, `normal`, or `high` |

- **low**: 100 kHz, 5µs pulse width (for slower steppers)
- **normal**: 500 kHz, 1µs pulse width (recommended for ClearPath)
- **high**: 2 MHz, 250ns pulse width (may cause timing issues)

```
-> set_motor_clock rate=normal
<- ok rate=normal
```

#### configure_estop

Configure an emergency stop pin for one or all motors. The pin must be (or will be auto-configured as) digital input. Uses NC (normally-closed) logic - motion stops when pin goes low.

| Parameter | Type | Description |
|-----------|------|-------------|
| `pin` | int | Pin index (0-12) |
| `motor` | int/string | Motor index (0-3) or `all` |
| `decel` | int | Deceleration rate (optional) |

```
-> configure_estop pin=8 motor=all
<- ok pin=8 motor=all

-> configure_estop pin=7 motor=0 decel=500000
<- ok pin=7 motor=0
```

---

### Motor Control Commands

| Command | Description | Required Params |
|---------|-------------|-----------------|
| `enable` | Enable single motor | `motor` |
| `enable_all` | Enable all configured motors | none |
| `disable` | Disable motor | `motor` |
| `move` | Command positional move | `motor`, `steps` or `position` |
| `move_velocity` | Command velocity move | `motor`, `velocity` |
| `stop` | Stop motor | `motor` |
| `home` | Start homing sequence | `motor` |
| `set_position` | Set position counter | `motor`, `position` |
| `clear_alerts` | Clear motor alerts | `motor` |

#### enable

Enable a single motor. For ClearPath motors, waits for HLFB assertion.

```
-> enable motor=0 seq=1
<- ok seq=1 motor=0
```

#### enable_all

Enable all configured motors in priority order. Motors with lower `enable_priority` are enabled first. Motors with `homing_mode` other than `none` are homed sequentially.

```
-> enable_all seq=1
<- ok seq=1 count=2
<- event type=homed motor=0 seq=1
<- event type=homed motor=1 seq=1
<- event type=enable_all_complete seq=1
```

#### disable

Disable a motor. Stops any motion in progress.

```
-> disable motor=0
<- ok motor=0
```

#### move

Command a positional move. Supports both relative (`steps`) and absolute (`position`) moves.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor` | int | required | Motor index (0-3) |
| `steps` | int | - | Relative move distance |
| `position` | int | - | Absolute target position |
| `vel` | int | motor vel_max | Per-move velocity limit |
| `accel` | int | motor accel_max | Per-move acceleration limit |

**Note:** `vel` and `accel` must not exceed the motor's configured maximums.

```
-> move motor=0 steps=1000 seq=1
<- ok seq=1 motor=0
<- event type=done motor=0 seq=1 position=1000

-> move motor=0 position=5000 vel=5000 accel=25000 seq=2
<- ok seq=2 motor=0
<- event type=done motor=0 seq=2 position=5000
```

#### move_velocity

Command a velocity move. Motor continues at specified velocity until stopped or soft limit is reached.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor` | int | required | Motor index (0-3) |
| `velocity` | int | required | Target velocity (signed, steps/sec) |
| `accel` | int | motor accel_max | Acceleration to target velocity |

```
-> move_velocity motor=0 velocity=10000 seq=1
<- ok seq=1 motor=0

-> stop motor=0
<- ok motor=0 position=15432
```

If a soft limit is reached during a velocity move:
```
<- event type=soft_limit motor=0 seq=1 position=10000
```

#### stop

Stop a motor.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor` | int | required | Motor index (0-3) |
| `immediate` | bool | false | If true, hard stop; if false, decelerate |

```
-> stop motor=0
<- ok motor=0 position=1234

-> stop motor=0 immediate=true
<- ok motor=0 position=1234
```

#### home

Start homing sequence for a motor. The motor must be enabled.

- **Generic steppers**: Move toward limit switch at seek velocity, back off, then approach slowly for precision
- **ClearPath-SD/SK**: Move toward hard stop, detect via HLFB torque feedback

```
-> home motor=0 seq=1
<- ok seq=1 motor=0
<- event type=homed motor=0 seq=1
```

#### set_position

Set the motor's position counter without moving. Useful for defining a new reference point.

```
-> set_position motor=0 position=0
<- ok motor=0 position=0
```

#### clear_alerts

Clear motor alerts (limit switch triggers, E-Stop). Required after an alert before further motion is allowed.

```
-> clear_alerts motor=0
<- ok motor=0
```

---

### Pin Configuration Commands

| Command | Description | Required Params |
|---------|-------------|-----------------|
| `configure_digital_in` | Configure digital input | `pin` |
| `configure_digital_out` | Configure digital output | `pin` |
| `configure_analog_in` | Configure analog input | `pin` |
| `configure_pwm` | Configure PWM output | `pin` |
| `configure_hbridge` | Configure H-Bridge output | `pin` |
| `configure_endstop` | Configure endstop input | `pin` |

#### configure_digital_in

Configure a pin as digital input with optional change/edge reporting and error triggering.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pin` | int | required | Pin index (0-12) |
| `report_changes` | bool | false | Emit event on value change |
| `report_edges` | string | none | Edge detection: `none`, `rising`, `falling`, `both` |
| `invert` | bool | false | Invert pin value before processing |
| `error_trigger` | bool | false | Enter error state on trigger value |
| `error_value` | bool | true | Value that triggers error (after invert) |

```
-> configure_digital_in pin=6 report_changes=true
<- ok pin=6

-> configure_digital_in pin=7 report_edges=rising invert=true
<- ok pin=7
```

**Edge Events:**
```
<- event type=edge pin=7 direction=rising
<- event type=edge pin=7 direction=falling
```

#### configure_digital_out

Configure a pin as digital output with optional timeout protection.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pin` | int | required | Pin index (0-5) |
| `initial` | bool | false | Initial output value |
| `on_error` | bool | - | Value to set on error state |
| `max_raised_ms` | int | 0 | Default high timeout (0 = disabled) |

```
-> configure_digital_out pin=0 initial=false max_raised_ms=5000
<- ok pin=0
```

If the pin stays high longer than `max_raised_ms`, it is automatically lowered and a notification is sent:
```
<- pin_timeout pin=0 seq=1
```

#### configure_analog_in

Configure a pin as analog input with optional threshold monitoring.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pin` | int | required | Pin index (9-12) |
| `error_low` | int | INT16_MIN | Enter error if value below |
| `error_high` | int | INT16_MAX | Enter error if value above |
| `report_interval` | int | 0 | Report interval in ms (0 = disabled) |

```
-> configure_analog_in pin=9 error_low=100 error_high=3900
<- ok pin=9
```

#### configure_pwm

Configure a pin as PWM output. ClearCore PWM runs at a fixed hardware frequency (~1.5kHz).

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pin` | int | required | Pin index (0-5) |
| `duty` | int | 0 | Duty cycle (0-65535) |

```
-> configure_pwm pin=0 duty=32768
<- ok pin=0
```

#### configure_hbridge

Configure a pin as H-Bridge output for bidirectional DC motor control or tone generation.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pin` | int | required | Pin index (4-5) |
| `value` | int | 0 | Initial value (-32767 to +32767) |

```
-> configure_hbridge pin=4 value=0
<- ok pin=4
```

#### configure_endstop

Configure a pin as an endstop input for motor homing (legacy, prefer native limit switches).

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pin` | int | required | Pin index |
| `triggered` | int | 0 | 0 = triggered when LOW (NC), 1 = triggered when HIGH (NO) |

```
-> configure_endstop pin=6 triggered=0
<- ok pin=6
```

---

### Pin Operation Commands

| Command | Description | Required Params |
|---------|-------------|-----------------|
| `read_pin` | Read pin value | `pin` |
| `write_pin` | Write digital output | `pin`, `value` |
| `set_pwm` | Set PWM parameters | `pin` |
| `set_hbridge` | Set H-Bridge value | `pin`, `value` |
| `start_tone` | Start tone on H-Bridge | `pin`, `frequency`, `amplitude` |
| `stop_tone` | Stop tone | `pin` |

#### read_pin

Read the current value of a configured pin.

```
-> read_pin pin=6
<- ok pin=6 value=true

-> read_pin pin=9
<- ok pin=9 value=2048

-> read_pin pin=0
<- ok pin=0 duty=32768
```

#### write_pin

Write a value to a digital output pin.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pin` | int | required | Pin index |
| `value` | bool | required | Output value |
| `max_ms` | int | configured default | Timeout for this write |

```
-> write_pin pin=0 value=true
<- ok pin=0 value=true

-> write_pin pin=0 value=true max_ms=10000 seq=1
<- ok seq=1 pin=0 value=true
```

#### set_pwm

Update PWM duty cycle on a configured PWM pin.

| Parameter | Type | Description |
|-----------|------|-------------|
| `pin` | int | Pin index |
| `duty` | int | New duty cycle (0-65535) |

```
-> set_pwm pin=0 duty=49152
<- ok pin=0
```

#### set_hbridge

Set H-Bridge output value.

```
-> set_hbridge pin=4 value=16000
<- ok pin=4

-> set_hbridge pin=4 value=-16000
<- ok pin=4
```

#### start_tone / stop_tone

Generate tones on H-Bridge pins (for buzzers/speakers).

```
-> start_tone pin=4 frequency=440 amplitude=16000
<- ok pin=4

-> stop_tone pin=4
<- ok pin=4
```

---

## Events

Events are sent asynchronously when certain conditions occur.

| Event Type | Description | Parameters |
|------------|-------------|------------|
| `done` | Motor move completed | `motor`, `position`, [`epoch`], `seq` |
| `homed` | Motor homing completed | `motor`, [`epoch`], `seq` |
| `soft_limit` | Velocity move hit soft limit | `motor`, `position`, [`epoch`], `seq` |
| `estop` | E-Stop triggered | `motor`, `position`, [`epoch`], `seq` |
| `limit` | Limit switch triggered | `motor`, `direction`, `position` |
| `edge` | Digital input edge detected | `pin`, `direction` |
| `change` | Digital input changed | `pin`, `value` |
| `analog` | Analog input report | `pin`, `value` |
| `pin_timeout` | Digital output auto-lowered | `pin`, [`epoch`], `seq` |
| `enable_all_complete` | All motors enabled/homed | [`epoch`], `seq` |

### Event Examples

```
<- event type=done motor=0 epoch=1 seq=42 position=10000
<- event type=homed motor=1 seq=5
<- event type=soft_limit motor=0 seq=10 position=50000
<- event type=estop motor=0 position=1234
<- event type=limit motor=0 direction=neg position=0
<- event type=edge pin=7 direction=rising
<- event type=change pin=6 value=true
<- event type=pin_timeout pin=0 seq=5
```

---

## Error Codes

### Protocol Errors (100-199)

| Code | Name | Description |
|------|------|-------------|
| 100 | UNKNOWN_COMMAND | Command not recognized |
| 101 | INVALID_PARAM | Invalid parameter value |
| 102 | MISSING_PARAM | Required parameter missing |
| 103 | INVALID_STATE | Command not valid in current state |
| 104 | EPOCH_MISMATCH | Command epoch doesn't match current |
| 105 | STALE_SEQ | Sequence number already seen |
| 106 | INPUT_OVERFLOW | Command line exceeded 511 character limit |

### Pin Errors (200-299)

| Code | Name | Description |
|------|------|-------------|
| 200 | INVALID_PIN | Pin index out of range |
| 201 | PIN_NOT_CONFIGURED | Pin not configured |
| 202 | PIN_CAPABILITY | Pin doesn't support requested mode |
| 203 | PIN_OVERCURRENT | Pin overcurrent fault |
| 204 | PIN_TIMEOUT | (Reserved - timeout sends pin_timeout event) |
| 205 | PIN_ERROR_TRIGGER | Error trigger condition met |
| 206 | ANALOG_THRESHOLD | Analog value outside threshold |
| 207 | PIN_CONFLICT | Pin already in use |

### Motor Errors (300-399)

| Code | Name | Description |
|------|------|-------------|
| 300 | INVALID_MOTOR | Motor index out of range |
| 301 | MOTOR_NOT_CONFIGURED | Motor not configured |
| 302 | MOTOR_FAULT | Motor hardware fault |
| 303 | MOTOR_NOT_READY | Motor not enabled |
| 304 | HLFB_TIMEOUT | HLFB not asserted in time |
| 305 | SOFT_LIMIT | Move exceeds soft limits |
| 306 | EXCEEDS_LIMIT | Per-move vel/accel exceeds motor max |

### System Errors (400-499)

| Code | Name | Description |
|------|------|-------------|
| 400 | INTERNAL_ERROR | Internal firmware error |
| 401 | EMERGENCY_STOP | Emergency stop activated |

---

## I/O Connector Configuration

ClearCore has 13 configurable I/O points. Each connector supports specific modes:

| Label | Digital Input | Digital Output | PWM | 0-10V Analog | H-Bridge | Notes |
|-------|---------------|----------------|-----|--------------|----------|-------|
| IO-0 | yes | yes | yes | | | |
| IO-1 | yes | yes | yes | | | |
| IO-2 | yes | yes | yes | | | |
| IO-3 | yes | yes | yes | | | |
| IO-4 | yes | yes | yes | | yes | |
| IO-5 | yes | yes | yes | | yes | |
| DI-6 | yes | | | | | Input only |
| DI-7 | yes | | | | | Input only |
| DI-8 | yes | | | | | Input only |
| A-9 | yes | | | yes | | |
| A-10 | yes | | | yes | | |
| A-11 | yes | | | yes | | |
| A-12 | yes | | | yes | | |

### Pin Usage Notes

- **Motor Limit Switches**: When configuring limit switches via `configure_stepper`, those pins are automatically reserved and cannot be used for other purposes.
- **E-Stop Pins**: Can be shared across multiple motors.
- **Pin Conflicts**: Attempting to configure a pin that's already in use returns error code 207.

---

## Example Session

A complete session demonstrating motor setup and motion:

```
-> ping
<- ok

-> configure_sdsk motor=0 vel_max=20000 accel_max=100000 soft_limits=true soft_min=-50000 soft_max=50000
<- ok motor=0

-> configure_estop pin=8 motor=all
<- ok pin=8 motor=all

-> configure_digital_out pin=0 max_raised_ms=30000
<- ok pin=0

-> enable motor=0 seq=1
<- ok seq=1 motor=0
<- event type=homed motor=0 seq=1

-> move motor=0 steps=10000 seq=2
<- ok seq=2 motor=0
<- event type=done motor=0 seq=2 position=10000

-> write_pin pin=0 value=true seq=3
<- ok seq=3 pin=0 value=true

-> move motor=0 position=0 vel=5000 seq=4
<- ok seq=4 motor=0
<- event type=done motor=0 seq=4 position=0

-> write_pin pin=0 value=false
<- ok pin=0 value=false

-> disable motor=0
<- ok motor=0
```

---

## Addendum: Comparison with ClearCore Command Protocol (CCCP)

The Cutter Protocol is inspired by and compatible with ClearCore hardware but differs from the original CCCP example in several ways.

### Command Format

| Aspect | CCCP | Cutter |
|--------|------|--------|
| Syntax | Single-letter commands (e.g., `e0`, `m1 1000`) | Named commands with parameters (e.g., `enable motor=0`) |
| Case | Case-insensitive | Case-sensitive (lowercase) |
| Parameters | Positional | Named key=value pairs |
| Parsing | `atoi()` for numerics | Typed parameter parsing |

### Feature Comparison

| Feature | CCCP | Cutter |
|---------|------|--------|
| Motor types | ClearPath-SD only | ClearPath-SD + Generic steppers |
| Move types | Absolute OR relative (compile-time) | Both (per-command) |
| Per-move limits | No | Yes (vel, accel) |
| Soft limits | No | Yes (with velocity move protection) |
| Homing | Manual | Automatic with configurable parameters |
| Limit switches | Not integrated | ClearCore native support |
| E-Stop | Not integrated | Per-motor with shared pin support |
| Edge detection | No | Hardware-level with clear-on-read |
| Command correlation | No | Epoch + sequence numbers |
| Async events | Limited | Comprehensive event system |
| State machine | Implicit | Explicit with error recovery |
| Pin timeout | No | Configurable with auto-recovery |
| Pin conflict detection | No | Yes |

### Command Mapping

| CCCP | Cutter Equivalent |
|------|-------------------|
| `e#` (enable) | `enable motor=#` |
| `d#` (disable) | `disable motor=#` |
| `m# dist` (move) | `move motor=# steps=dist` or `move motor=# position=dist` |
| `v# vel` (velocity) | `move_velocity motor=# velocity=vel` |
| `q#p` (query position) | `status` or read motor state |
| `q#v` (query velocity) | Not directly mapped (use events) |
| `q#s` (query status) | `status` |
| `l#v limit` (vel limit) | `configure_sdsk motor=# vel_max=limit` |
| `l#a limit` (accel limit) | `configure_sdsk motor=# accel_max=limit` |
| `c#` (clear alerts) | `clear_alerts motor=#` |
| `z#` (zero position) | `set_position motor=# position=0` |
| `i#` (read input) | `read_pin pin=#` |
| `o# val` (write output) | `write_pin pin=# value=val` |
| `f type` (feedback type) | N/A (always structured) |
| `h` (help) | N/A |

### Migration Notes

1. **Command syntax**: Replace single-letter commands with named equivalents
2. **Move type**: Specify `steps=` for relative or `position=` for absolute
3. **Configuration**: Must explicitly configure pins/motors before use
4. **Events**: Use `seq` parameter to correlate async responses
5. **Errors**: Check numeric error codes for programmatic handling
