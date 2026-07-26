 hexa_hardware

`ros2_control` SystemInterface plugin for the real hexapod: bridges the
controller manager's joint command/state interfaces and a UART-attached
open servo controller (Pimoroni Servo 2040 or any board speaking the
same protocol).

C++ / `ament_cmake` because pluginlib loads `hardware_interface`
plugins by class name from a shared library.

Sim runs through `gz_ros2_control` and lives in `hexa_simulation`; this
package only owns the real-robot path.

## Plugin name

The URDF declares `hexa_hardware/HexaHardware` (see
`hexa_description/urdf/hexapod.urdf.xacro`, the `<xacro:unless
use_sim>` branch under `<ros2_control>`). The plugin resolves its
config from `hexa_description`'s share directory by default (co-located
with `geometry.yaml` / `tuning.yaml`): `hardware.yaml` for wiring and
`servo_calibration.yaml` for the per-servo endpoint pulse widths. Pass
`<param name="config_path">/abs/path/to/hardware.yaml</param>` and/or
`<param name="calibration_path">/abs/path/to/servo_calibration.yaml</param>`
under `<hardware>` to override either (e.g. for a test rig with
different calibration).

## Pluggable transport + protocol

Two seams under one plugin class, both selected from YAML:

- **Transport** (`include/hexa_hardware/transport.hpp`) — byte pipe.
  Open / close / write / read-with-timeout, nothing more. Concrete:
  `UartTransport` (POSIX serial — the Pi header UART by default, and it
  equally covers a Servo 2040 wired over USB-CDC).
  Placeholders: `I2cTransport`, `UsbTransport` (raw HID/bulk) — both
  declared and wired through the factory, both throw on `open()` until
  someone fills in the body.
- **BoardProtocol** (`include/hexa_hardware/board_protocol.hpp`) —
  semantic operations the hardware interface needs: drive consecutive
  servo pins, set servo power (relay), read the battery in engineering
  units. Owns a `Transport&` and the wire framing. Concrete:
  `Servo2040Protocol` (Chica framing, see below). The relay pin and the
  battery telemetry units are board-owned protocol constants, not host
  config — the interface exposes intent (`set_servo_power`) and real units
  (`read_battery` → volts/amps), never raw pins or scale factors.

The factory (`hardware_factory.hpp`) picks both from
`hexa_description/config/hardware.yaml`:

    connection:
      type: uart           # uart | i2c | usb
      device: /dev/ttyAMA0
      baud: 115200
    parser:
      type: servo2040
      get_period_ticks: 10

Adding a new board is one new `BoardProtocol` subclass plus a branch
in `make_board_protocol`. Adding a new physical layer is one new
`Transport` subclass plus a branch in `make_transport`.

## Wire protocol ("Modified Chica")

Half-duplex over a single UART. Byte 0x80 mask discriminates command
bytes from data bytes:

- Command byte — MSB set. `S | 0x80` for SET, `G | 0x80` for GET.
- Data byte — MSB clear, so 7 bits per byte; a 14-bit value packs into
  two data bytes little-endian: `lo = v & 0x7F`, `hi = (v >> 7) & 0x7F`.

Frames:

- SET — `[S | 0x80][start_pin][count][val_lo, val_hi] × count`. Writes
  `count` consecutive pins starting at `start_pin`. A pin assigned as a
  digital output (e.g. the relay) interprets values 0 / 1 as low / high.
- GET request — `[G | 0x80][start_pin][count]`.
- GET reply — same shape as SET: `[G | 0x80][start_pin][count][val × count]`.
- SETALL — `[0xD5][29 payload bytes]`, a fixed 30 bytes with no start/count
  header. Drives **all 18 servos** from board index 0. The payload is the 18
  values MSB-first, 11 bits each, concatenated into one bitstream and emitted 7
  bits at a time into the low 7 bits of each byte (so payload bytes stay
  MSB-clear and resync still works); the tail byte's low 5 bits are zero padding.
  Value encoding is `pulse_us - 500`, range `0…2000` → `500…2500 µs`, clamped
  board-side. Servo-only: the relay and all telemetry keep using SET/GET.

The battery current/voltage indices reply in fixed-point centi-units
(count × 0.01 = A / V); `read_battery` applies that one fixed factor and
returns real units. The relay lives at a fixed board-owned index. See the
servo2040 driver's `protocol.md` for the unit definition and index map.

Recovery from a partial frame on the wire is trivial: discard bytes
until one with MSB set arrives.

## Joint → frame batching

`write()` sorts joints by board index once, in `on_init`, and precomputes the
frame plans — the wiring fixes them, so nothing is rebuilt per tick.

**Steady state** (energize sweep complete) sends the whole pose as **one 30-byte
SETALL frame**. That number is the point: the equivalent SET is
`[S][0][18]` + 36 data bytes = **39 bytes**, which exceeds the board's **32-byte
UART RX FIFO**. If the firmware's main loop stalls while such a frame streams in,
the FIFO overruns and the frame's **tail** is lost — i.e. the servo on the
highest pin, silently and intermittently. 30 bytes fits the FIFO whole, so the
loop can be busy for an entire frame and lose nothing.

SETALL applies only when both hold (checked once in `on_init`, logged either
way); otherwise `write()` falls back to the consecutive-run SET frames:

- The harness is the flat board map 0…17 — every servo present, no gaps, no
  offset (`is_flat_pin_map`, `leg_order.hpp`). SETALL carries no start/count
  header, so it can express nothing else.
- Every servo's `pulse_us` clamp is inside `[500, 2500]` µs. SETALL clamps a
  sub-500 pulse *up* to 500 and drives it, where a SET below 500 means "hold
  last position" in firmware — a tighter override would change meaning, so it
  disables the fast path instead.

**During the sweep ramp**, `write()` keeps emitting one SET frame per live leg
(9 bytes each, comfortably inside the FIFO). SETALL cannot be used there: the
board stages every channel in the frame, so a single SETALL energizes all 18
servos at once and defeats the stagger, whose whole point is that un-commanded
channels stay limp until their turn. Only the whole-table frame was ever at risk
of losing its tail.

A harness whose joints are **not** on consecutive pins still works — the run
splitter emits one SET per maximal consecutive run — it just pays more frames.

## Servo rail: relay, then a per-leg energize sweep

The board never drives a servo the host has not commanded. `SET RELAY 1` closes
the relay with **every servo limp**, and a servo SET sent while the rail is open
is discarded (there is no pre-relay staging), so the host owns both the pose and
the order the servos come up in.

`apply_relay()` (called from `read()`, on the controller-manager thread) drives
the relay toward `/hardware/relay_cmd` — the locomotion supervisor's arm intent,
true on a live link in any non-`fault` engine state — and forces it off while a
board over-current trip is latched. On the OFF→ON edge it arms
`hexa::EnergizeSweep` (`shared/motion_core/energize_sweep.hpp`, shared with the
Pico firmware), and `write()` then drives only the legs the sweep has brought
live so far:

- Legs come up in **pin order**, `init.sweep_leg_interval_ms` apart. With the
  shipped wiring that is `l_rear, r_rear, l_middle, r_middle, l_front, r_front`
  — rear → front, alternating sides.
- The stagger keeps the inrush as six small steps instead of one spike big
  enough to trip the board's over-current tiers. `0` disables it.
- Once the sweep completes, `write()` emits exactly what it did before it
  existed. `build_leg_order` (`leg_order.hpp`) derives the leg grouping from the
  joint names + pin table, so a rewired harness re-orders the sweep with it.
- The same edge serves cold start and over-current recovery.

## State feedback

`read()` echoes the last commanded position into the position state
interface (hobby servos don't report shaft angle) and computes velocity
as the numerical derivative. Joint state is **not** polled from the
board.

The battery bus **is** polled via a single GET, rate-limited by
`parser.get_period_ticks` so SETs aren't starved, and republished in
engineering units on `~/battery_state` (`sensor_msgs/BatteryState`) from an
internal node. It needs no host config: the units are protocol-defined and
the sensors are always present on the board.

## Lifecycle

- `on_init` — load config, build Transport + BoardProtocol via factory.
- `on_configure` — open the Transport.
- `on_activate` — `set_servo_power(false)` (known-off baseline, which also
  clears any latch), reset commands to the current echoed state so the first
  cycle doesn't snap. Activating never powers the rail; `apply_relay()` closes
  it once the supervisor asks.
- `on_deactivate` — `set_servo_power(false)`.
- `on_cleanup` — close serial, stop the aux publisher thread.

## Config

Config lives in `hexa_description/config/`, split in two:

- `hardware.yaml` — wiring: `connection`, `parser`, `init`
  (`sweep_leg_interval_ms`), `deg_at_center`, a shared
  `servo_defaults.pulse_us` clamp, and a `servos` map of per-servo
  `{pin, reversed?, pulse_us?}` (keyed by URDF joint name; `reversed`/`pulse_us`
  default when omitted). No relay/aux pins: the relay and battery sensors are
  board-owned protocol constants.
- `servo_calibration.yaml` — a pin-ordered `calibration_values` list of
  `{pin, us_at_plus_45, us_at_minus_45}`; a servo with `pin: N` reads entry
  `N-1`. Split out so a calibration routine can rewrite it without touching
  the commented wiring.

Both files document their own field semantics (calibration math, `reversed`,
the `deg_at_center` → URDF-radian conversion).

## Bench testing without hardware

There is no mock plugin in this package — sim already covers
zero-hardware testing. For a wire-level smoke test against the
servo2040 backend, pair a PTY with `socat`:

    socat -d -d pty,raw,echo=0 pty,raw,echo=0

then point `connection.device` at one end and listen on the other. The
`connection.baud` must match the firmware's UART setting; it is only
ignored if the board is reached over USB-CDC instead.
