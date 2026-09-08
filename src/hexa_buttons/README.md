# hexa_buttons

Two momentary switches on the Pi's GPIO header. Real robot only.

- **info button** (GPIO5) — press: pack percentage, voltage, web teleop
  address. Hold 3 s: toggle Wi-Fi station / `hexapod` hotspot.
- **bluetooth button** (GPIO6) — press: connected controller. Hold 3 s: start a
  pairing scan.

A producer into `hexa_display` over `/display/text`, `/bluetooth/scanning` and
`/display/busy`. Imports nothing from it.

## Wiring

Each switch between its BCM line and ground, SoC pull-up (`active_low` +
`bias_pull_up`). GPIO5 is pin 29, GPIO6 is pin 31.

Read with gpiozero on an explicitly pinned lgpio factory. gpiozero 2.0's
`LGPIOFactory.__init__` ignores its `chip` argument on a Pi 5 and opens
`gpiochip4`, which does not exist in the container, so `_lgpio_factory` in
`gpio_buttons.py` rebuilds the factory by hand and asserts the chip. If the
lines cannot be claimed, the node logs once and runs inert.

## Screens

Dismissed by the same button, swapped by the other, or after
`screen_timeout_s`. Strings live in `info_text.py`. Lines are budgeted at 30
ASCII characters (`LINE_BUDGET`); overflow wraps and pushes the last line off
the panel. `label_arrow` picks the separator; the font is ASCII + Latin-1 only.

Station screen shows `ip:port` (or `mdns_name` first when set). Hotspot screen
shows `control.hexa` plus SSID / password, both reported by the host. No pack
reading renders as `-- %`, no address as `no network`.

## Bluetooth seam

The scanning utility is **not yet written**.

- **`/bluetooth/scanning`** (`std_msgs/Bool`, transient_local) — published here.
  True while the operator's scan runs; false on cancel (either button),
  `scan_timeout_s`, or a non-empty `/bluetooth/status`.
- **`/bluetooth/status`** (`std_msgs/String`, transient_local) — to be published
  by the utility. Controller name, empty for none.

Pairing must run on the host (BlueZ over system D-Bus). Copy the network seam.

## Network seam

The container cannot run `nmcli`. Two files on the bind-mounted log volume:

- **`log/network`** (`network_spool`) — container → host. One line, action +
  token, written in place (never renamed; systemd watches the inode).
- **`log/network.state`** — host → container. `key=value` lines: mode,
  credentials, result. Written tmp + rename.

Host runs `systemd/network-mode.sh` and owns the mode, SSID and password, so
the node sends `toggle` and renders what comes back. The token is a per-process
nonce so a stale reply is never read as an ack. Ends: host `result=`, no ack
within `network_ack_timeout_s` (units not installed), or `network_timeout_s`.
Inert until `./hexa robot install-network`. See `docs/robot-environment.md` §14.

## Layout

- `screen_logic.py` — screen state machine and wiring → gpiozero args. Pure.
  Fed `(event, t)` plus a periodic `TICK`.
- `info_text.py` — screen strings, voltage → percentage, line budget.
- `network_state.py` — spool wire format and tolerant state-file parser. Pure.
- `local_ip.py` — address to advertise (`SIOCGIFADDR`, `wlan0` then `eth0`).
- `gpio_buttons.py` — the only gpiozero import, lazy inside the function.
- `network_spool.py` — the two file touches. Impure, untested.
- `button_node.py` — the only rclpy module.

### Threading

GPIO callbacks only timestamp an event onto a `queue.SimpleQueue`. The tick
drains it before advancing timeout clocks, so the state machine is
single-threaded and ordering is total. All timing is `time.monotonic()`, never
the node clock.

## Configuration

`config/buttons.yaml`. `enabled: false` skips the node in `robot.launch.py`;
`network_toggle_enabled: false` drops the info hold. `hold_s` feeds both buttons
and the screen text. `control_port` is overridden at launch from
`hexa_webteleop`'s `server.port`. `mdns_name` is a hand edit, valid only after
`./hexa robot install-mdns`. `battery_empty_v` / `battery_full_v` drive the
screen number only; the safety ladder is `battery:` in `hardware.yaml`.

## Tests

    ./hexa sim python3 -m pytest src/hexa_buttons/test -q

Runs without gpiozero.

- `test_screen_logic` — toggling, timeouts, holds, scan lifecycle,
  held-at-startup guard, thread race orders, wiring translation.
- `test_info_text` — percentage map, every screen string, line budget and
  ASCII guard.
- `test_network_state` — request round-trips, token uniqueness, tolerant
  parser.
- `test_local_ip` — interface preference and fallbacks.

Not unit-tested: `open_buttons`, the ioctl, `network_spool`, rclpy wiring.
