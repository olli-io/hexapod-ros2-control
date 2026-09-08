# hexa_hardware

`ros2_control` SystemInterface plugin for the real hexapod. It bridges the
controller manager's joint interfaces and a UART-attached servo board
(Pimoroni Servo 2040, "Modified Chica" protocol). C++ / `ament_cmake`
because pluginlib loads the plugin by class name.

Sim runs through `gz_ros2_control` in `hexa_simulation`. This package owns the
real-robot path only.

## Plugin

The URDF declares `hexa_hardware/HexaHardware`
(`hexa_description/urdf/hexapod.urdf.xacro`, `<xacro:unless use_sim>`).
Config is read from `hexa_description`'s share directory: `hardware.yaml`
(wiring) and `servo_calibration.yaml` (per-servo endpoint pulses). Override
with `<param name="config_path">` / `<param name="calibration_path">`.

## Seams

Two YAML-selected seams, built by `hardware_factory.hpp` from
`hardware.yaml`'s `connection:` and `parser:` blocks:

- **Transport** (`include/hexa_hardware/transport.hpp`) — byte pipe. Concrete:
  `UartTransport`. Placeholders: `I2cTransport`, `UsbTransport` (throw on
  `open()`).
- **BoardProtocol** (`include/hexa_hardware/board_protocol.hpp`) — drive
  servo pins, set servo power (relay), read battery in real units. Concrete:
  `Servo2040Protocol`. Relay index and telemetry units are protocol constants,
  not host config.

New board: one `BoardProtocol` subclass + a branch in `make_board_protocol`.
New physical layer: one `Transport` subclass + a branch in `make_transport`.

## Wire protocol

Half-duplex UART. Command bytes have MSB set, data bytes MSB clear (7 bits;
14-bit values as two bytes little-endian). Frames:

- **SET** — `[S|0x80][start_pin][count][lo,hi] × count`.
- **GET** — request `[G|0x80][start_pin][count]`; reply shaped like SET.
- **SETALL** — `[0xD5][29 bytes]`, all 18 servos from pin 0, 11 bits each
  packed 7 bits per byte. Value = `pulse_us - 500`.

Resync: discard bytes until one with MSB set. Index map and units: the
servo2040 driver's `protocol.md`.

## Write path

Frame plans are precomputed in `on_init` (`leg_order.hpp`). Steady state sends
one 30-byte SETALL, which fits the board's 32-byte RX FIFO. SETALL requires a
flat pin map 0…17 and every `pulse_us` clamp inside `[500, 2500]`; otherwise
`write()` sends one SET per consecutive pin run. During the energize sweep it
always sends one SET per live leg.

## Servo rail

`apply_relay()` (from `read()`) drives the relay toward `/hardware/relay_cmd`
and forces it off while an over-current trip is latched. The OFF→ON edge arms
`hexa::EnergizeSweep` (`shared/motion_core/energize_sweep.hpp`, shared with the
Pico firmware): legs come up in pin order, `init.sweep_leg_interval_ms` apart.

## Buzzer

Publishes tune names on `/buzzer/play` (`std_msgs/String`, transient_local):
`up` from `on_activate`, `fault` on a trip edge, `undervolt` on undervoltage
rung 1. Best-effort; `hexa_buzzer` owns the hardware.

## State feedback

`read()` echoes the last command as position and differentiates it for
velocity. The battery is polled by GET every `parser.aux_period_ms` on the aux
thread and published on `~/battery_state` (`sensor_msgs/BatteryState`).

Undervoltage policy lives in the locomotion supervisor
(`shared/motion_core/supervisor.hpp`), published on `/hardware/undervoltage`
(`std_msgs/UInt8`, 0–3). This node acts on rung 1 (buzzer) and rung 3 (sticky
relay-off latch, reset only by restarting the process).

## Threading

The control cycle never waits on the board.

- **controller-manager thread** — `read()` / `write()` at 200 Hz. Writes only.
- **aux thread** — `hexa_hardware_aux` node executor + `poll_aux()`. Owns
  every blocking GET.

`Transport::write()` is serialized; reads take no lock. The two paths use
separate scratch buffers. `poll_aux()` never drives the relay: a trip sets
atomics and `apply_relay()` emits `SET RELAY 0` on the next control tick.

## Lifecycle

- `on_init` — load config, build Transport + BoardProtocol.
- `on_configure` — open the Transport.
- `on_activate` — servo power off, sync commands to echoed state, request `up`.
- `on_deactivate` — servo power off.
- `on_cleanup` — close serial, stop the aux thread.

## Config

In `hexa_description/config/`:

- `hardware.yaml` — `connection`, `parser`, `init`, `buzzer`, `battery`
  (read by the supervisor and `gen_config.py`, not here), `deg_at_center`,
  `servo_defaults.pulse_us`, and a `servos` map of `{pin, reversed?, pulse_us?}`
  keyed by joint name.
- `servo_calibration.yaml` — pin-ordered `calibration_values` list of
  `{pin, us_at_plus_45, us_at_minus_45}`.

Both files document their own field semantics.

## Bench test

No mock plugin; sim covers zero-hardware testing. For a wire-level smoke test:

    socat -d -d pty,raw,echo=0 pty,raw,echo=0

Point `connection.device` at one end and listen on the other.
