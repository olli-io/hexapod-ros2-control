# hexa_hardware

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
config from this package's own share directory by default; pass
`<param name="config_path">/abs/path/to/servo2040.yaml</param>` under
`<hardware>` to override (e.g. for a test rig with different
calibration).

## Wire protocol ("Chica")

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

Non-joint pins (battery voltage, currents, touch) **are** polled via
GET, rate-limited by `serial.get_period_ticks` so SETs aren't starved.
Voltage / current are republished on `~/battery_state`
(`sensor_msgs/BatteryState`) from an internal node.

## Lifecycle

- `on_configure` — open serial.
- `on_activate` — drive relay pin high (servo rail on), reset commands
  to the current echoed state so the first cycle doesn't snap.
- `on_deactivate` — drive relay pin low.
- `on_cleanup` — close serial, stop the aux publisher thread.

## Config

`config/servo2040.yaml` carries:

- `serial.{device,baud,get_period_ticks}`
- `relay.pin` — board pin wired to the servo power relay
- `aux.{name}.{pin,scale}` — GET-only sensor channels
- `deg_at_center.{coxa,femur,tibia}` — joint angle at servo center pulse, shared across all six legs, in the intuitive per-joint convention from `hexa_description/config/geometry.yaml` (`coxa.deg`, `femur.above_horizontal_deg`, `tibia.interior_deg`). Defaults mirror that file's `joints:` block (0 / 35 / 68).
- `joints.{urdf_joint_name}.{pin, joint_position, us_at_plus_45, us_at_minus_45, min_us, max_us}` — `joint_position` is `coxa | femur | tibia` and selects which `deg_at_center` entry applies.

Per-servo calibration is two measured endpoints in the *servo's* frame
(shaft at ±π/4 from mechanical center). Center pulse and slope (with
sign) fall out automatically; a reversed mount is expressed by
swapping the two values. The shared `deg_at_center` table captures the
assembly offset — the joint angle each segment actually sits at when
its servo is centered — letting one set of three numbers describe an
ideally-mounted build. The loader translates those intuitive degrees
to URDF radians per joint position (coxa: `rad`, femur: `−rad`, tibia:
`π − rad`), matching the conversion in `hexa_description/urdf/hexapod.urdf.xacro`.
URDF-side joint angle limits in `hexa_description`'s `geometry.yaml`
remain the source of truth for travel; `min_us` / `max_us` are the
electrical clamp the driver enforces.

## Bench testing without hardware

There is no mock plugin in this package — sim already covers
zero-hardware testing. For a wire-level smoke test against the
servo2040 backend, pair a PTY with `socat`:

    socat -d -d pty,raw,echo=0 pty,raw,echo=0

then point `serial.device` at one end and listen on the other. The
Servo2040 itself does not care about baud (USB-CDC); a real UART build
should set it to match the firmware.
