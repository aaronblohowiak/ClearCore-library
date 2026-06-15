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

**Connection banner.** When a host opens the serial port, the controller emits a one-time `debug` banner identifying itself, before any command is sent:

```
<- debug message="cutter ready" protocol=1 version=1.0.0 device_id=12648430
```

- `protocol` / `version` match the values returned by the `version` command.
- `device_id` is the board's factory serial number (the ClearCore NVM serial), stable across reboots and unique per board. Use it to tell one controller from another.

The banner is triggered by the USB virtual-port open (DTR) edge, so a client that opens the port and waits silently **will** receive it — no need to send anything first. It is emitted exactly once per connection and re-armed if the port is closed and reopened. As a fallback, a transport that never asserts DTR still gets the banner on the first byte it sends. Read and discard (or log) the banner after opening the port, before issuing commands.

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

**Event (Asynchronous, unsolicited):**
```
event type=<event_type> [param=value ...]
```

**Status (Solicited query payload):**
```
status type=<row_type> [epoch=E seq=S] [param=value ...]
```
Status rows are the body of a multi-line query response (currently only `get_status`). They are **not** asynchronous events — they are emitted only in direct reply to a command and carry that command's `epoch`/`seq`. See [get_status](#get_status).

**Debug (Diagnostic):**
```
debug message="<text>" [param=value ...]
```
Human-oriented diagnostic line carrying no command id (e.g. the [connection banner](#communication)). Clients may log or ignore it.

> **Distinguishing lines:** every line begins with one of five prefixes — `ok`, `error`, `event`, `status`, or `debug`. Dispatch on the first token. A robust client treats any unknown leading token as a `debug`/ignore line so future additions don't break it.

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
| `get_status` | Dump all configured pins and motors | `get_status` |

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

#### get_status

Dump the full configured state: one `status` row per configured pin, one per configured motor, terminated by a `status type=summary` row that carries the counts. The response uses the [status channel](#response-format), not events.

```
-> get_status seq=7
<- status type=pin epoch=1 seq=7 pin=0 mode=digital_in value=1 invert=0
<- status type=motor epoch=1 seq=7 motor=0 motor_type=clearpath enabled=1 moving=0 position=0 homed=1 homing_mode=msp vel_max=10000 accel_max=100000
<- status type=summary epoch=1 seq=7 pin_count=1 motor_count=1
```

**Client parsing contract:**

- Read `status` rows carrying the request's `epoch`/`seq` until you receive `status type=summary` with the same id. The summary row **terminates** the response — there is **no trailing `ok`**.
- With nothing configured, only the summary row is returned: `status type=summary epoch=1 seq=7 pin_count=0 motor_count=0`.
- Rows carry only the fields relevant to each pin's `mode` / motor configuration; **absent optional fields mean "default / disabled"** (e.g. no `report_edges` means edge reporting is off).
- The motor row's kind is `motor_type=` (not `type=`), so it does not collide with the line's leading `type=` key. Build your key/value map accordingly.
- Asynchronous `event` lines (e.g. a `limit` trip) may interleave between status rows; they carry a **different** `epoch`/`seq` (or none). Filter by the request id rather than assuming every line up to the summary belongs to `get_status`.

| Row (`type=`) | Key fields |
|---------------|-----------|
| `pin` | `pin`, `mode`, then mode-specific fields (`value`, `invert`, `report_edges`, `duty`, `triggered`, ...) |
| `motor` | `motor`, `motor_type`, `enabled`, `moving`, `position`, `homed`, `homing_mode`, `vel_max`, `accel_max` (+ optional `soft_limit_min/max`, `velocity_move`, `homing_state`) |
| `summary` | `pin_count`, `motor_count` |

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

When the E-Stop trips during a move, the motor is put into the alert state and
an `alert` event with `cause=estop` is emitted (see the Events section). Clear
it with `clear_alerts` before further motion.

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
| `move_until` | Velocity move until a sensor input trips | `motor`, `velocity`, `until_pin`, `until_value` |
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

If a limit switch trips during the move, the move is canceled: a `limit`
sensor event and an `alert` event (`cause=pos_limit`/`neg_limit`) are emitted,
and no `done` is sent. If the motor is already in an alert state (or you
command a move back into an active limit), the move is rejected up front with
`error code=307 MOVE_REJECTED` and the motor does not move - clear the alert
and move away from the limit first.

```
-> move motor=0 steps=10000 seq=3
<- ok seq=3 motor=0
<- event type=limit motor=0 direction=pos value=1 position=8500 seq=3
<- event type=alert motor=0 cause=pos_limit position=8500 seq=3

-> clear_alerts motor=0
<- ok motor=0
-> move motor=0 steps=10000 seq=4          # back into the limit
<- error code=307 message="Move rejected: clear alerts and move away from the limit" seq=4
-> move motor=0 steps=-1000 seq=5          # away from the limit
<- ok seq=5 motor=0
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

If a hardware limit switch trips during a velocity move, the move is canceled
and both a `limit` and an `alert` event are emitted (same as for `move`):
```
<- event type=limit motor=0 direction=pos value=1 position=20000 seq=1
<- event type=alert motor=0 cause=pos_limit position=20000 seq=1
```

#### move_until

Run a velocity move until a **non-limit sensor** input reaches a target level,
then stop and report the position. Use this to lower a suction head until a
vacuum sensor reports suction has formed, or to feed material until a part
sensor sees it. Unlike a limit switch, the sensor trip is **not a fault**: no
alert is latched and no `clear_alerts` is needed afterward.

The motor's configured **soft limits are the travel bound**. If the sensor
never trips (e.g. no item is present), the move stops at the soft limit and
emits the usual `soft_limit` event - that is the "not found" outcome. The
command therefore requires `soft_limits` to be enabled on the motor.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor` | int | required | Motor index (0-3) |
| `velocity` | int | required | Target velocity (signed, steps/sec) - sign sets direction |
| `until_pin` | int | required | Pin to watch; must be configured as a digital input |
| `until_value` | int (0/1) | required | Logical level (after the pin's `invert`) that stops the move |
| `accel` | int | motor accel_max | Acceleration to target velocity |

Prerequisites: `until_pin` configured with `configure_digital_in`, and the
motor configured with `soft_limits=1`. The trip is **level-based**: if the
sensor already reads `until_value` when the command is issued, the motor stops
immediately on the next update (it will not drive into an already-formed
vacuum).

Two terminal outcomes, both carrying the final `position`:

```
# Vacuum forms: sensor on pin 6 reaches the target level
-> move_until motor=0 velocity=-2000 until_pin=6 until_value=1 seq=1
<- ok seq=1 motor=0
<- event type=sensor_stop motor=0 pin=6 position=-1840 seq=1

# No item found: head bottoms out at the soft limit instead
-> move_until motor=0 velocity=-2000 until_pin=6 until_value=1 seq=2
<- ok seq=2 motor=0
<- event type=soft_limit motor=0 position=-3000 seq=2
```

A hardware E-Stop or limit switch during a `move_until` still cancels it with an
`alert` event, exactly as for any other move.

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

- **ClearPath-SD/SK** (`homing_mode=msp`): Move toward hard stop, detect via HLFB torque feedback.
- **Limit-switch homing** (steppers, or SDSK with `homing_mode=limit_switch`): a five-phase sequence that finds a precise, repeatable datum and then parks clear of the switch:
  1. **seeking** — fast approach (`homing_seek_velocity`) in `homing_direction` until the switch trips.
  2. **releasing** — back away until the switch's live input goes inactive. This is a *variable* distance, so it always clears the switch regardless of how far the fast seek overshot.
  3. **latching** — creep back (`homing_latch_velocity`) until the switch trips again. This precise re-trigger is the home **datum**.
  4. **backing_off** — move `homing_backoff` steps off the switch (measured from the datum).
  5. **complete** — set position **0 at the resting point**, clear of the switch, and emit `event type=homed`.

  Because zero is established after the clearance backoff, the switch contact ends up at `-homing_backoff` and **position 0 has margin from the limit** — you can command a move to 0 without risk of tripping the switch. (Set a soft-limit minimum of 0 to enforce this.) The in-progress phase is visible as `homing_state` in [`get_status`](#get_status).

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

**Recovery after a limit switch trip:** the motor stays physically on the
switch, so the limit is still active. After `clear_alerts`, the **first move
must be in the direction away from the limit**. A move commanded back into an
active limit is rejected by the hardware and returns
`error code=307 MOVE_REJECTED` (the motor does not move). Move off the switch
first, then resume normal motion.

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

Events are sent asynchronously when certain conditions occur. They are distinct from `status` rows: an `event` is unsolicited (it can arrive at any time, including between the rows of a `get_status` response), whereas `status` rows are the solicited payload of a query. Do not treat `get_status` output as events — see [get_status](#get_status).

| Event Type | Description | Parameters |
|------------|-------------|------------|
| `done` | Motor move completed | `motor`, `position`, [`epoch`], `seq` |
| `homed` | Motor homing completed | `motor`, [`epoch`], `seq` |
| `soft_limit` | Velocity move hit soft limit | `motor`, `position`, [`epoch`], `seq` |
| `sensor_stop` | `move_until` sensor reached its target level (not a fault) | `motor`, `pin`, `position`, [`epoch`], `seq` |
| `alert` | Motor put into alert state, motion canceled (latched until `clear_alerts`) | `motor`, `cause`, `position`, [`epoch`], `seq` |
| `limit` | Limit switch input changed state | `motor`, `direction`, `value`, `position`, [`epoch`], `seq` |
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
<- event type=alert motor=0 cause=estop position=1234 seq=7
<- event type=limit motor=0 direction=neg value=1 position=0
<- event type=limit motor=0 direction=neg value=0 position=200
<- event type=edge pin=7 direction=rising
<- event type=change pin=6 value=true
<- event type=pin_timeout pin=0 seq=5
```

#### `alert` vs `limit`: two distinct events

A limit switch changing state and a motor being put into the alert state are
separate conditions, and each has its own event:

- **`limit`** is a *sensor* event: the limit input changed state. It fires on
  every transition (`value=1` pressed, `value=0` released), whether the motor
  is idle, moving, or homing. It says nothing about the motor.
- **`alert`** is a *motor* event: the motor's motion was canceled and it is now
  latched in the alert state until `clear_alerts`. The `cause` identifies the
  source (`estop`, `pos_limit`, `neg_limit`).

When a limit is hit **during a move**, you get *both* - the sensor changed and
the motor was put into alert:

```
<- event type=limit motor=0 direction=pos value=1 position=8500 seq=12
<- event type=alert motor=0 cause=pos_limit position=8500 seq=12
```

When the same switch is pressed while the motor is **idle** (or during homing,
which consumes the alert itself), only the `limit` event fires.

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
| 307 | MOVE_REJECTED | Hardware rejected the move (alert present or at active limit) |

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
