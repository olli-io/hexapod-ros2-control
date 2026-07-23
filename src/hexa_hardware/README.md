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

The battery current/voltage indices reply in fixed-point centi-units
(count × 0.01 = A / V); `read_battery` applies that one fixed factor and
returns real units. The relay lives at a fixed board-owned index. See the
servo2040 driver's `protocol.md` for the unit definition and index map.

Recovery from a partial frame on the wire is trivial: discard bytes
until one with MSB set arrives.

## Joint → SET batching

Pin assignment is configurable so each leg's three joints occupy three
consecutive pins (0–2, 3–5, …). `write()` sorts joints by pin index,
splits into maximal consecutive runs, and emits one SET frame per run.
With the default config that's six 5-byte-payload frames per cycle.

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
- `on_activate` — `set_servo_power(true)` (servo rail on), reset commands
  to the current echoed state so the first cycle doesn't snap.
- `on_deactivate` — `set_servo_power(false)`.
- `on_cleanup` — close serial, stop the aux publisher thread.

## Config

Config lives in `hexa_description/config/`, split in two:

- `hardware.yaml` — wiring: `connection`, `parser`, `deg_at_center`, a shared
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
