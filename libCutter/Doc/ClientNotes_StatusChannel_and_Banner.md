# Client Notes: `status` channel, `get_status` reshape, and connection banner

Audience: developers writing or maintaining a client library against the Cutter
protocol. This summarizes a set of wire-format changes and what your client must
do to consume them. For the full protocol, see
[`CutterProtocol_UserGuide.md`](./CutterProtocol_UserGuide.md); the machine-readable
contract is in [`../spec/cutter_protocol.json`](../spec/cutter_protocol.json).

---

## TL;DR

1. **New line prefix: `status`.** There are now **five** top-level line types:
   `ok`, `error`, `event`, `status`, `debug`. Dispatch on the first token.
2. **`get_status` no longer uses `event` lines and no longer ends in `ok`.** It
   emits `status type=pin` / `status type=motor` rows and a terminating
   `status type=summary` row. The summary row *is* the end of the response.
3. **Motor row field renamed:** the motor's kind is now `motor_type=` (was
   `type=`), removing a key collision with the line's leading `type=`.
4. **New connection banner:** when a host opens the port, the controller emits a
   one-time `debug` line with `protocol`, `version`, and a unique `device_id`.

If your client previously parsed `get_status`, it **will break** until updated —
see [Migration](#migration).

---

## 1. The `status` channel

`status` is a solicited query-payload line. It looks like an `event` but is
semantically different and must be handled separately:

```
status type=<row_type> epoch=<E> seq=<S> [key=value ...]
```

- It is emitted **only in direct reply to a command**, never spontaneously.
- It always carries the requesting command's `epoch`/`seq`.
- Currently the only producer is `get_status`, with row types `pin`, `motor`,
  and `summary`.

Why a separate channel instead of reusing `event`: an `event` is an *unsolicited
notification* (a limit trip, an input edge, a fault) and may arrive at any time.
Conflating the two meant a generic event handler would see query payload, and a
"read until `ok`" loop could swallow an unrelated async event. Keeping `status`
distinct lets a client route the two cleanly.

Recommended client dispatch:

```
switch (firstToken) {
  case "ok":     -> resolve the pending command
  case "error":  -> reject the pending command
  case "event":  -> async notification handler
  case "status": -> feed into the in-flight query accumulator (match by epoch/seq)
  case "debug":  -> log or ignore
  default:       -> log or ignore   // forward-compatible
}
```

## 2. `get_status` response shape

**Before** (old format — for reference only):

```
event type=pin epoch=1 seq=7 pin=0 mode=digital_in value=1 invert=0
event type=motor epoch=1 seq=7 motor=0 type=clearpath enabled=1 ...
ok epoch=1 seq=7 pin_count=1 motor_count=1
```

**Now:**

```
status type=pin epoch=1 seq=7 pin=0 mode=digital_in value=1 invert=0
status type=motor epoch=1 seq=7 motor=0 motor_type=clearpath enabled=1 moving=0 position=0 homed=1 homing_mode=msp vel_max=10000 accel_max=100000
status type=summary epoch=1 seq=7 pin_count=1 motor_count=1
```

Parsing contract:

- Accumulate `status` rows whose `epoch`/`seq` match your `get_status` request.
- Stop when you receive `status type=summary` with that id — it carries
  `pin_count` / `motor_count` and **marks the end. There is no trailing `ok`.**
- **Empty configuration** returns only the summary row:
  `status type=summary epoch=1 seq=7 pin_count=0 motor_count=0`. (Uniform shape —
  no special "empty" case to detect.)
- **Absent optional fields mean default / disabled.** Rows carry only the fields
  relevant to each pin's `mode` and each motor's configuration. For example, a
  digital-input row omits `report_edges` when edge reporting is off. Treat a
  missing key as "feature off / value at default," not as an error.
- **Async events may interleave.** A spontaneous `event` (e.g. `limit`) can land
  between status rows. It carries a *different* `epoch`/`seq` (or none). Match by
  the request id; do not assume every line before the summary belongs to your
  query.

### Row field reference

| `type=` | Fields |
|---------|--------|
| `pin` | `pin`, `mode`, then mode-specific: digital_in → `value`,`invert`,[`report_edges`],[`error_trigger`]; digital_out → `value`,[`on_error`],[`max_ms`]; analog_in → `value`,[`error_low`,`error_high`],[`report_interval`]; pwm → `duty`; hbridge → `value`,[`tone_freq`,`tone_amplitude`]; endstop → `triggered`,`triggered_value` |
| `motor` | `motor`, `motor_type`, `enabled`, `moving`, `position`, `homed`, `homing_mode`, `vel_max`, `accel_max`, [`soft_limit_min`,`soft_limit_max`], [`velocity_move`], [`homing_state`] |
| `summary` | `pin_count`, `motor_count` |

## 3. Motor row: `type=` → `motor_type=`

The motor row used to emit both `type=motor` (the line type) and `type=clearpath`
(the motor kind) on the same line — a duplicate key that clobbers itself in a
naive key/value map. The motor kind is now `motor_type=`. If you split lines into
a map keyed by field name, no special handling is needed anymore; just read
`motor_type`.

## 4. Connection banner

When a host opens the serial port, before any command is sent, the controller
emits one `debug` line:

```
debug message="cutter ready" protocol=2.0 version=2.0.0 device_id=12648430
```

- `protocol` / `version` equal what the `version` command returns. Both are SemVer
  **strings** (compare as strings, not numbers), unquoted only because they have no spaces.
- `device_id` is the board's factory serial number — on ClearCore this is the
  NVM serial number (`ClearCore::NvmManager::SerialNumber()`), a `uint32` that is
  **stable across reboots and unique per board**. Use it to distinguish multiple
  controllers, tag logs, or pin a client session to a specific device.

> **Yes, we have a hardware serial.** The `device_id` is sourced from the
> ClearCore NVM serial number (the number printed on the board). A 6-byte
> Ethernet MAC and the SAM E5x 128-bit unique chip id also exist on the part, but
> the NVM serial is the cleanest stable per-board identifier and is what the
> banner reports. If you need the wider chip id or MAC instead, that would be an
> additive change to the banner.

**Trigger and timing:** the banner fires on the **port-open edge** — on USB this
is the virtual-port DTR assertion, i.e. the moment your client opens the port.
You do **not** need to send a byte first; just open the port and read. Behavior
details a client can rely on:

- Emitted **exactly once per connection**, before any command response.
- **Re-armed on disconnect:** if the port is closed and reopened, the next host
  gets a fresh banner.
- **Fallback:** a transport that never asserts DTR still receives the banner,
  triggered by the first byte the host sends.

So: open the port, read the first line, and if it is a `debug` banner, capture
`device_id`/`version` (or just log it) before issuing commands. Because it is a
`debug` line, a client that ignores `debug` lines entirely is also correct — it
simply won't learn the device id.

---

## Migration checklist

- [ ] Add a `status` case to your line dispatcher; route rows to a per-request
      accumulator keyed by `epoch`/`seq`.
- [ ] Change `get_status` handling to terminate on `status type=summary` instead
      of `ok`. Remove any "read until `ok`" logic for this command.
- [ ] Read the motor kind from `motor_type`, not `type`.
- [ ] Stop treating `pin` / `motor` as event types in your async event handler.
- [ ] (Optional) On connect, read the `debug` banner and surface its
      `device_id` / `version`.
- [ ] Ensure your dispatcher ignores unknown leading tokens (don't hard-fail) so
      future line types don't break you.
