# hexa_buttons

Two momentary switches on the Raspberry Pi's GPIO header — the robot's only
input that does not need a gamepad, a phone, or an SSH session. They put
diagnostic screens on the face's OLED and request a Bluetooth pairing scan.

- **info button** (GPIO5) — press: pack percentage and voltage, plus the address
  a phone points at the web teleop UI.
- **bluetooth button** (GPIO6) — press: which controller is connected. Hold 3 s:
  start a pairing scan; the face wears its `scanning` spinners while it runs.

A *producer into* the display, not part of it. `hexa_display` stays a pure sink
of robot state, and everything here reaches it over topics that node already
subscribes to — `/display/text` for the screens, `/bluetooth/scanning` for the
spinners. Nothing here imports `hexa_display`, and with no face fitted the
topics simply go unread.

## Wiring

Each switch goes between its BCM line and ground; the SoC's internal pull-up
holds the line high, so no external resistor is needed (`active_low` +
`bias_pull_up` in `config/buttons.yaml`).

- **GPIO5** — physical pin 29, ground on 30. Info button.
- **GPIO6** — physical pin 31, ground on 34. Bluetooth button.

Both are clear of everything else the robot claims on the header: SPI0 on 8–11
(the SH1122 face), the header UART on 14/15 (the Servo2040 link), PWM0 on 18
(the buzzer), and 24/25 (the panel's DC/RST). They also avoid GPIO2/3 so an I2C
peripheral can still be added later.

The lines are read with **gpiozero** on an explicitly pinned **lgpio** pin
factory — edge-driven, so nothing is polled between presses, and debounce
(`bounce_time`) and the 3 s hold clock (`hold_time`) are gpiozero's rather than
ours. The container already has `/dev/gpiochip0` and the `gpio` group for the
face, so no compose change is needed; `hexa_display` holds different lines on
the same chip and the kernel arbitrates per line.

The factory is pinned rather than left to gpiozero's default search
(lgpio -> rpigpio -> pigpio -> native), because the tail of that list is
dangerous here: `rpigpio` does not work on a Pi 5 at all, `pigpio` needs a
daemon the container does not run, and `native` maps BCM283x registers
directly — on a Pi 5's RP1 it does not fail, it reads garbage. Naming the
factory turns a wrong environment into a startup error instead of a button that
silently never responds.

If the lines cannot be claimed — no chip, wrong permissions, lines already
taken, or gpiozero simply not installed — the node logs an error and stays alive
but inert. Buttons are a convenience fitting and must never hold up bringup.

## Screens

The panel goes back to the face when the same button is pressed again, when the
other button swaps screens and that one times out, or after
`screen_timeout_s` (6 s).

    Battery -> 50 %  ( 7.4 V )
    Control -> 192.168.172.42:8080

    Connected to:
    8 Bit Do Pro 2
    Hold 3 seconds to pair

No pack reading yet renders as `-- %  ( --.- V )` rather than a fabricated 0 %,
which would read as a dead battery. No address renders as `no network`.

The separator is ASCII on purpose. The display's bundled Pixel Operator font
covers ASCII + Latin-1 only, so a U+2192 `→` renders blank; to use one, add the
codepoint to `codepoints()` in `shared/display_core/tools/gen_font.py` and
regenerate `hexa_text_font.c`. `label_arrow` in the config picks the separator.

## Bluetooth seam

The scanning utility is **not yet written**. The contract it plugs into:

- **`/bluetooth/scanning`** (`std_msgs/Bool`, transient_local) — published
  *here*. This node owns the pairing **session**: true when the operator holds
  the Bluetooth button, false when the scan is cancelled (either button), times
  out (`scan_timeout_s`, 30 s), or succeeds — where "succeeds" means a
  non-empty `/bluetooth/status` arrived while the scan was running, which drops
  the spinners immediately instead of making the operator wait out the timeout.
  `hexa_display` reads it for the spinners.
- **`/bluetooth/status`** (`std_msgs/String`, transient_local) — published by the
  **utility**. The connected controller's name, empty for none. It is the
  *executor*: it watches `/bluetooth/scanning`, runs the scan/pair for as long
  as that is true, and reports the result here.

Until the utility exists, `/bluetooth/status` has no publisher, the bluetooth
screen reads "No connected controllers", and a scan simply runs out its timeout.

Note that pairing has to happen outside the container: BlueZ is reached over the
host's system D-Bus, which the robot container does not mount. The established
pattern for that in this repo is the buzzer spool — a file on the bind-mounted
log volume watched by a host systemd `.path` unit (see `systemd/buzzer.sh` and
`docs/robot-environment.md`).

## Layout

Same split as `hexa_teleop`: pure logic in its own modules, carrying the tests;
the device- and ROS-touching code thin and separate.

- `hexa_buttons/screen_logic.py` — the screen state machine and the wiring →
  gpiozero-argument translation. Pure: no rclpy, no gpiozero, no clocks. It is
  handed one `(event, t)` per real button event plus a periodic `TICK`.
- `hexa_buttons/info_text.py` — the screen strings and the voltage →
  percentage map.
- `hexa_buttons/local_ip.py` — the address to advertise (`SIOCGIFADDR`,
  preferring `wlan0` then `eth0`). The container runs `network_mode: host`, so
  these are the Pi's own interfaces. The ranking half is pure and tested; the
  ioctl half is a dozen lines.
- `hexa_buttons/gpio_buttons.py` — the only module that imports gpiozero, and
  it does so **lazily inside the function**. That keeps the pure modules
  importable in the sim container, which has no gpiozero, and makes a missing
  library land on the inert path rather than crashing at import.
- `hexa_buttons/button_node.py` — the only rclpy component.

### Threading

gpiozero delivers edges on its own pin thread and holds on a second one; rclpy
runs the tick timer and both subscriptions on the executor thread. The GPIO
callbacks do exactly one thing — timestamp the event and put it on a
`queue.SimpleQueue` — and the tick drains that queue *before* advancing the
timeout clocks. That buys three things a lock around the state machine would
not:

- The state machine is touched by one thread, so it stays a plain object with no
  concurrency contract — and the render path, which makes ioctls to find the IP,
  never holds a mutex a button callback is waiting on.
- **Ordering is total.** A press at t=5.99 is always applied before the timeout
  check at t=6.05. Under a lock the two can interleave the other way: the screen
  times out, then the press reopens it — a visible, nondeterministic flash.
- A callback firing during shutdown cannot reach a destroyed publisher, because
  it never touches one.

The cost is up to one tick (50 ms at the default `tick_rate_hz: 20.0`) between
the release and the screen appearing. Timestamps are taken in the callback, not
at drain time, so a stalled executor still applies events at the instant they
happened.

All timing is `time.monotonic()`, never the node clock: that is `RCL_ROS_TIME`,
i.e. system time, and a Pi has no RTC — an NTP step at boot would otherwise
expire a live screen or hang one for the size of the step.

## Configuration

All knobs in `config/buttons.yaml`: line numbers and wiring polarity, the
housekeeping tick rate, debounce/hold/timeout clocks, the battery topic and the
voltage span, the address interface preference, and the label separator.
`enabled: false` makes `robot.launch.py` skip the node.

`active_low` and `bias_pull_up` collapse onto gpiozero's `pull_up` /
`active_state` pair, which are mutually constrained. The combination
`active_low: false` with `bias_pull_up: true` is rejected at startup — a
pull-up holds the line high, so an active-high button would read as permanently
pressed.

`control_port` is overridden at launch from `server.port` in `hexa_webteleop`'s
`webteleop.yaml` — the port the screen advertises has to be the port the server
actually binds. The value in `buttons.yaml` is only the fallback for a
hand-launched node.

The `battery_empty_v` / `battery_full_v` span drives **the number on the screen
only**. It is not a safety threshold: the undervoltage ladder that beeps, folds
and cuts the servo rail is `battery:` in `hexa_description/config/hardware.yaml`,
and the expressions a weak pack puts on the face are `battery_warning_v` /
`battery_critical_v` in `hexa_display/config/display.yaml`. The reading itself
comes through the Servo2040's voltage divider, whose scale is uncalibrated on
this build — measure against a meter before trusting the percentage.

## Tests

`./hexa sim python3 -m pytest src/hexa_buttons/test -q` — 44 cases, all
headless, and deliberately runnable in a container **without** gpiozero (which
is also the check that the lazy import holds):

- `test_screen_logic` — screen toggling and swapping, timeouts, the pairing
  hold and its swallowed release, scan cancellation and success, the
  already-held-at-startup guard, the hold/release thread race in both orders,
  and the wiring translation.
- `test_info_text` — the percentage map, its clamping, its degenerate case and
  its half-away-from-zero rounding, plus every screen string including the
  no-reading and no-network paths.
- `test_local_ip` — interface preference order and both fallbacks.

Not unit-tested on purpose (same policy as `hexa_teleop`'s
`test_joy_publisher.py`): `open_buttons`, the ioctl enumeration, and the rclpy
wiring. Those hinge on kernel and driver behaviour and are verified on the
robot.
