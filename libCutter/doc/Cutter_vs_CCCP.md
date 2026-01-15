# Cutter vs CCCP (ClearCore Command Protocol) Comparison

This document compares Cutter with Teknic's official CCCP example project.

## Overview

| Aspect | CCCP | Cutter |
|--------|------|--------|
| Architecture | Single-file example (~870 LOC) | Library with state machine, HAL abstraction |
| Command Format | Single letter + number (`e0`, `m1 1000`) | Named commands with key=value (`enable motor=0`) |
| Motor Types | ClearPath Step+Dir only | ClearPath SDSK + Generic Steppers |
| Homing | External (MSP configured) | Built-in endstop + hard-stop homing |
| Events | None (polling only) | Async events for state changes |
| State Machine | None | 7 states with epoch tracking |
| Error Handling | Numeric codes | Structured error responses + error state |
| Testability | Hardware only | HAL abstraction enables unit testing |

## Command Comparison

### Motor Commands

| Function | CCCP | Cutter |
|----------|------|--------|
| Enable | `e0` | `enable motor=0` |
| Disable | `d0` | `disable motor=0` |
| Position move | `m0 1000` | `move motor=0 steps=1000` or `move motor=0 position=1000` |
| Velocity move | `v0 500` | `move_velocity motor=0 velocity=500` |
| Stop | (none) | `stop motor=0` |
| Query position | `q0p` | (via events or `status`) |
| Query velocity | `q0v` | (via events) |
| Query status | `q0s` | `status` |
| Set velocity limit | `l0v 1000` | `configure_sdsk motor=0 vel_max=1000` |
| Set accel limit | `l0a 50000` | `configure_sdsk motor=0 accel_max=50000` |
| Clear alerts | `c0` | `reset` (from error state) |
| Zero position | `z0` | `set_position motor=0 position=0` |
| Home | (none - MSP only) | `home motor=0` |
| Enable all + home | (none) | `enable_all` |

### I/O Commands

| Function | CCCP | Cutter |
|----------|------|--------|
| Read digital | `i6` | `read_pin pin=6` |
| Read analog | `i9` | `read_pin pin=9` |
| Write digital | `o5 1` | `write_pin pin=5 value=1` |
| Write analog | `o0 1024` | `set_pwm pin=0 duty=1024` |
| Configure pin | (hardcoded in setup) | `configure_digital_in pin=6` |
| PWM output | (none) | `set_pwm pin=0 duty=32768 freq=1000` |
| H-Bridge | (none) | `set_hbridge pin=4 value=16000` |
| Tone generation | (none) | `start_tone pin=4 freq=440 amplitude=10000` |
| Endstop config | (none) | `configure_endstop pin=6 triggered=0` |

### System Commands

| Function | CCCP | Cutter |
|----------|------|--------|
| Help | `h` | (none - see docs) |
| Feedback mode | `f0` / `f1` | (always structured) |
| Ping/heartbeat | (none) | `ping` |
| Version | (none) | `version` |
| Emergency stop | (none) | `emergency_stop` |
| Get sequence | (none) | `get_next_seq` |

## Feature Comparison

### State Machine

**CCCP**: No state machine. Motors can be commanded at any time; errors are returned if motor not enabled.

**Cutter**: 7-state machine with epoch tracking:
```
UNCONNECTED -> CONNECTED -> CONFIGURED -> ENABLING -> READY -> WORKING
                                              |
                                              v
                                            ERROR (requires reset)
```

Epoch increments on each error, allowing stale commands to be rejected.

### Homing

**CCCP**: Relies on ClearPath MSP configuration for homing. The `e#` command waits for HLFB, which will include any MSP-configured homing sequence.

**Cutter**: Built-in homing support for both motor types:

| Motor Type | Homing Method |
|------------|---------------|
| Generic Stepper | Endstop-based: SEEKING -> BACKING_OFF -> LATCHING |
| ClearPath SDSK | Hard-stop via HLFB torque detection |

Sequential homing via `enable_all` respects `enable_priority` (lower = first).

### Endstop Configuration

**CCCP**: No endstop support.

**Cutter**: Fail-safe defaults:
- `end_stop_triggered=0` (default): Triggered when LOW (NC switch, fail-safe)
- `end_stop_triggered=1`: Triggered when HIGH (NO switch)

Broken wire reads same as triggered switch with NC default.

### Events

**CCCP**: No async events. Host must poll for status.

**Cutter**: Async events for:
- `event type=done motor=0 seq=1 position=1000` - Move complete
- `event type=homed motor=0 seq=1` - Homing complete
- `event type=all_homed seq=1 count=2` - All motors homed
- `event type=input pin=6 value=1` - Digital input change
- `event type=analog pin=9 value=2048` - Periodic analog report
- `event type=threshold pin=9 value=100` - Analog threshold crossed
- `event type=error code=304 message="HLFB timeout"` - Error occurred
- `event type=hlfb motor=0 state=1` - HLFB state change

### Error Handling

**CCCP**:
- Returns numeric error codes (0-16)
- Optional verbose mode (`f1`)
- No global error state

**Cutter**:
- Structured error responses: `error code=301 message="Motor not configured"`
- Error state requires explicit `reset` command
- Epoch tracking rejects stale commands after error recovery

### Soft Limits

**CCCP**: None.

**Cutter**: Per-motor configurable:
```
configure_stepper motor=0 soft_limits=1 soft_min=-10000 soft_max=10000
```

### Sequence Tracking

**CCCP**: None.

**Cutter**: Optional `seq=N` on commands, echoed in responses and events:
```
-> move motor=0 steps=1000 seq=42
<- ok seq=42 motor=0
<- event type=done motor=0 seq=42 position=1000
```

### Hardware Fault Monitoring

**CCCP**: Checks `MotorInFault` status bit, reports via `q#s`.

**Cutter**: Active monitoring with automatic error state entry:
- Motor `IsInHwFault()` detection
- Pin overcurrent detection (IO-0 to IO-5)
- HLFB timeout detection
- Configurable digital input error triggers
- Configurable analog threshold errors

## Protocol Syntax

### CCCP
```
e0              # enable motor 0
m1 1000         # move motor 1 to position 1000
v2 -500         # motor 2 velocity move at -500 steps/s
q0p             # query motor 0 position
l0v 5000        # set motor 0 velocity limit to 5000
i6              # read digital input 6
o5 1            # write digital output 5 high
```

### Cutter
```
ping
configure_stepper motor=0 vel_max=10000 accel_max=50000 end_stop_pin=6
configure_endstop pin=6 triggered=0
enable motor=0
home motor=0 seq=1
move motor=0 steps=1000 seq=2
move_velocity motor=0 velocity=-500
read_pin pin=6
write_pin pin=5 value=1
set_pwm pin=0 duty=32768 freq=1000
```

## When to Use Each

### Use CCCP when:
- Simple step+direction control is sufficient
- MSP-configured homing is acceptable
- Minimal code complexity is priority
- No need for async events
- Single motor type (ClearPath only)

### Use Cutter when:
- Mixed motor types (ClearPath + generic steppers)
- Built-in homing required
- Async event notification needed
- Robust error handling required
- State machine protection desired
- Unit testing required
- PWM, H-Bridge, or tone generation needed
- Fail-safe endstop defaults matter

## Migration from CCCP to Cutter

| CCCP | Cutter |
|------|--------|
| `e0` | `enable motor=0` |
| `d0` | `disable motor=0` |
| `m0 1000` | `move motor=0 position=1000` (absolute) |
| `m0 1000` | `move motor=0 steps=1000` (relative) |
| `v0 500` | `move_velocity motor=0 velocity=500` |
| `q0p` | Query via `status` or track via events |
| `l0v 5000` | Set at config: `configure_sdsk motor=0 vel_max=5000` |
| `l0a 50000` | Set at config: `configure_sdsk motor=0 accel_max=50000` |
| `z0` | `set_position motor=0 position=0` |
| `c0` | `reset` (when in error state) |
| `i6` | `read_pin pin=6` |
| `o5 1` | `write_pin pin=5 value=1` |

Key differences to note:
1. Cutter requires explicit pin/motor configuration before use
2. Cutter uses named parameters instead of positional
3. Cutter has a state machine - must be in correct state for commands
4. Cutter generates async events for move completion, input changes, etc.
