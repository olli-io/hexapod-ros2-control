# hexa_buttons

Two momentary switches on the Raspberry Pi's GPIO header — the robot's only
input that does not need a gamepad, a phone, or an SSH session. They put
diagnostic screens on the face's OLED, request a Bluetooth pairing scan, and
switch the Pi between joining Wi-Fi and hosting its own.

- **info button** (GPIO5) — press: pack percentage and voltage, plus the address
  a phone points at the web teleop UI. Hold 3 s: switch network mode, between
  joining Wi-Fi and hosting the `hexapod` hotspot that serves that same UI.
- **bluetooth button** (GPIO6) — press: which controller is connected. Hold 3 s:
  start a pairing scan; the face wears its `scanning` spinners while it runs.

A *producer into* the display, not part of it. `hexa_display` stays a pure sink
of robot state, and everything here reaches it over topics that node already
subscribes to — `/display/text` for the screens, `/bluetooth/scanning` and
`/display/busy` for the spinners. Nothing here imports `hexa_display`, and with
no face fitted the topics simply go unread.

The two spinner topics are separate because they mean different things — a
pairing scan, and a network switch — and land on one expression because the
face's answer to both is "wait, I am working". `hexa_display` ORs them.

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

Pinning it takes more than `LGPIOFactory(chip=N)`, because gpiozero 2.0 — the
`python3-gpiozero` Ubuntu 24.04 ships, and what the robot image installs —
discards that argument on the next line:

    def __init__(self, chip=None):
        super().__init__()
        chip = 4 if (self._get_revision() & 0xff0) >> 4 == 0x17 else 0
        self._handle = lgpio.gpiochip_open(chip)

`0x17` is a BCM2712, so on a Pi 5 it always opens `/dev/gpiochip4` whatever it
was asked for. On Raspberry Pi OS that is harmless — udev leaves a
`gpiochip4 -> gpiochip0` compatibility symlink from the 6.1 kernel, where the
RP1 really was chip 4. In the container there is no symlink, only the one device
node compose maps in, so the open fails with `can not open gpiochip` and the
node runs on inert. `_lgpio_factory` therefore builds the factory the long way
round — everything `LGPIOFactory.__init__` does, with the chip number it was
actually given — and asserts the chip it ended up on, so a future gpiozero
rearranging that constructor is a startup error rather than a silent relapse.

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

In hotspot mode the battery screen grows a third line, so the network a phone
has to join is always one press away rather than only visible in the seconds
after a switch:

    Battery -> 50 %  ( 7.4 V )
    Control -> 192.168.4.1:8080
    WiFi -> hexapod / hexahexa

and finishing a switch puts up the result — the credentials, or what went wrong:

    Hotspot -> hexapod
    Password -> hexahexa
    Control -> 192.168.4.1:8080

    Network switch failed
    Could not start the hotspot
    Hold 3 seconds to retry

No pack reading yet renders as `-- %  ( --.- V )` rather than a fabricated 0 %,
which would read as a dead battery. No address renders as `no network`.

**Lines are budgeted at 30 characters** (`LINE_BUDGET` in `info_text.py`). The
panel is four lines of a proportional 16 px font wrapping at 252 px, and
overflow does not truncate — it *wraps*, and the extra line pushes the last one
off the panel, so a line over budget makes an unrelated line vanish. 30 holds
for ordinary mixed text; 28 is the floor for the widest glyphs. Both numbers are
measured against the real font in `hexa_display`'s `test_text_screen.cpp`, and
guarded on this side by a test over every string this package can emit.

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
host's system D-Bus, which the robot container does not mount. The pattern to
copy is the network seam below — a request file on the bind-mounted log volume
watched by a host systemd `.path` unit, answered in a second file. The network
switch is the worked example, and the only one left: the buzzer used the same
shape until it got a writable bind of the PWM tree and became `hexa_buzzer`, an
ordinary node.

## Network seam

Holding the info button flips the Pi between joining Wi-Fi and hosting the
`hexapod` hotspot (password `hexahexa`, robot at `192.168.4.1`) that serves the
web teleop. The face wears its spinners while the switch runs.

The switch cannot happen in this process. The container is unprivileged with
host networking, no D-Bus socket and no `NET_ADMIN`, so it cannot reach
NetworkManager at all — the same wall the buzzer hits with the PWM sysfs, and
the same way around it. Two files on the bind-mounted log volume:

- **`log/network`** (`network_spool` in this package) — container to host. One
  line: an action and a token. Written in place, truncate + one write + close,
  and deliberately **not** renamed into position: systemd's `PathModified`
  watches that inode, and swapping a new one underneath would break the watch.
- **`log/network.state`** — host to container. `key=value` lines naming the
  mode, the credentials, and how the last request went. The host writes this one
  tmp + rename, so the tick's poll can never read half a line.

The host runs `systemd/network-mode.sh` and is the **authority on the current
mode** — the AP profile being active is the whole definition of it — so the
button sends `toggle` rather than naming a target, and this node only ever
renders what it was told. The SSID and password live in that script and come
back in the state file; they are deliberately not duplicated in `buttons.yaml`.

The **token** separates "the host answered me" from "this file still holds the
answer to a switch from ten minutes ago". It carries a per-process nonce, so a
node that restarts mid-switch cannot mistake the reply to its previous life's
request for an ack. `mode` is trusted whatever the token says — it is a standing
report, and a node that just started needs it to render a correct screen.

Three ways a switch ends, in order of how quickly the operator finds out:

- The host answers `result=ok` or `result=error`, and the panel shows why.
- Nothing acknowledges within `network_ack_timeout_s` (5 s) — nothing is
  watching the spool, so the units were never installed. Says so at once rather
  than spinning for the full timeout.
- The host acknowledged and then went quiet: `network_timeout_s` (60 s) ends it
  on the result screen, not the face, because a silent return to the eyes is
  exactly the case where something needs saying.

The whole feature is **inert** until `./hexa robot install-network` has been run
on the Pi. See `docs/robot-environment.md` §14.

## Layout

Same split as `hexa_teleop`: pure logic in its own modules, carrying the tests;
the device- and ROS-touching code thin and separate.

- `hexa_buttons/screen_logic.py` — the screen state machine and the wiring →
  gpiozero-argument translation. Pure: no rclpy, no gpiozero, no clocks. It is
  handed one `(event, t)` per real button event plus a periodic `TICK`.
- `hexa_buttons/info_text.py` — the screen strings, the voltage → percentage
  map, and the panel's line budget.
- `hexa_buttons/network_state.py` — the container/host spool wire format:
  request lines, the `key=value` state file, and the tolerant parser for it.
  Pure, so both sides of a bind mount have one definition to agree on.
- `hexa_buttons/local_ip.py` — the address to advertise (`SIOCGIFADDR`,
  preferring `wlan0` then `eth0`). The container runs `network_mode: host`, so
  these are the Pi's own interfaces. The ranking half is pure and tested; the
  ioctl half is a dozen lines.
- `hexa_buttons/gpio_buttons.py` — the only module that imports gpiozero, and
  it does so **lazily inside the function**. That keeps the pure modules
  importable in the sim container, which has no gpiozero, and makes a missing
  library land on the inert path rather than crashing at import.
- `hexa_buttons/network_spool.py` — the two file touches that carry a switch
  across the container boundary. Impure and decision-free, so it is not
  re-exported and not unit-tested, same policy as `gpio_buttons`.
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
voltage span, the address interface preference, the network spool paths and
their timeouts, and the label separator. `enabled: false` makes
`robot.launch.py` skip the node; `network_toggle_enabled: false` keeps the
screens but drops the info button's hold.

`hold_s` is passed to **both** buttons and to the screens that advertise it, so
the panel can never promise a hold the buttons do not honour.

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

`./hexa sim python3 -m pytest src/hexa_buttons/test -q` — 128 cases, all
headless, and deliberately runnable in a container **without** gpiozero (which
is also the check that the lazy import holds):

- `test_screen_logic` — screen toggling and swapping, timeouts, both holds and
  their swallowed releases, scan cancellation and success, the
  already-held-at-startup guard (which is what stops a jammed button taking the
  robot off the network), the hold/release thread race in both orders, that
  nothing can cancel a switch in flight, and the wiring translation.
- `test_info_text` — the percentage map, its clamping, its degenerate case and
  its half-away-from-zero rounding, every screen string including the
  no-reading, no-network, hotspot and failure paths, and a guard asserting every
  string this package can emit is inside the line budget and pure ASCII.
- `test_network_state` — the spool wire format: request round-trips, token
  uniqueness across requests and across restarts, and a parser that reads junk,
  partial writes and unknown keys as "no news" rather than raising.
- `test_local_ip` — interface preference order and both fallbacks.

Not unit-tested on purpose (same policy as `hexa_teleop`'s
`test_joy_publisher.py`): `open_buttons`, the ioctl enumeration, `network_spool`,
and the rclpy wiring. Those hinge on kernel, filesystem and driver behaviour and
are verified on the robot.

The panel's line budget is pinned from the other side too — `hexa_display`'s
`test_text_screen.cpp` measures it against the real font, so the number this
package designs against cannot drift from what the panel can actually show.
