# hexa_webteleop

Web-app teleop — an HTTP + WebSocket server hosting a phone/tablet control
UI that publishes the **same** ROS topics as the gamepad teleop
(`hexa_teleop`): `/cmd_vel`, `/body/pose`, `/cmd_gait`, `/animation/mode`,
`/gait/initialize`, plus `/teleop/owner`. Subscribes `/gait/state` for
switch gating.

## Architecture

- **`web_mapping.py`** — pure Python (no rclpy). Loads webapp config and
  delegates to `hexa_teleop.joy_mapping.map_joy` for the full state
  machine (modes, init two-press, record, yaw easing, height, cycling).
  Unit-testable.
- **`webteleop_node.py`** — ROS glue. `aiohttp` server in a daemon thread
  + 50 Hz rclpy timer that maps input and publishes; shared state behind a
  `threading.Lock`. Single-connection policy: a second device gets `busy`
  and is closed, retrying until the slot frees.
- **`web/`** — static webapp (`index.html` + `main.js`, `logs.html` +
  `logs.js`, `styles.css`). No TypeScript, no build step, no npm.

## Webapp UI

- **Navbar** (symbols only, top bar in portrait / left strip in landscape)
  — wifi connection indicator, controller icon (green while a controller
  owns `/cmd_vel`; tap for a switch toggle), log icon.
- **Control area** — two touch joysticks flanking a 3×3 button grid; top 3
  buttons select mode (Gait / Posture / Anim), bottom 6 are
  mode-dependent (node pushes labels on mode change). While a controller
  owns control the grid becomes an inline "Take control" prompt and the
  sticks are disabled.

Default stick mapping (config): left = forward/strafe (gait) or x/y
translation (posture); right = turn (gait) or roll/pitch (posture).
Default bottom-6 buttons: init, record, then per mode — gait prev/next
(gait) / yaw left/right (posture) / animation prev/next (animation) — and
height up/down.

## Safety

The phone link is unreliable; three independent guards stop motion when it
goes quiet:

- **WebSocket heartbeat** — server pings every `server.ws_heartbeat_s` and
  force-closes on a missed pong, turning a half-open TCP connection into a
  real disconnect (which zeroes inputs and releases control).
- **Input watchdog** — the 50 Hz timer feeds neutral input whenever no
  message arrived within `safety.input_timeout_s`, so `/cmd_vel` falls to
  zero instead of latching. Backstop for any stall. Pure and unit-tested.
- **Client visibility stop** — the webapp re-centres both sticks on
  `visibilitychange → hidden` (tab switch / screen lock).

On disconnect the node also zeroes shared state so a new device can't
inherit the departed one's inputs.

## Coexistence with gamepad teleop

Both nodes run simultaneously; only one publishes. A single latched
`/teleop/owner` (`gamepad` default, or `web`) arbitrates; only the web
node writes it. Webapp connects as a passive observer, then **Take
control** → `request_control` → owner `web`, gamepad goes dormant.
Releasing (toggle, disconnect, or `POST /control/release`) publishes
`gamepad` and resumes it. Logic lives in
`hexa_teleop.teleop_arbitration` (pure, shared, unit-tested).

## HTTP endpoints

Alongside `/ws`:

- **`GET /logs`** — returns `{"lines": [...]}` from the configured
  `logs.command` (default: recent `~/.ros/log` files), for the log page.
- **`POST /control/release`** — hands control back to the gamepad
  (out-of-band equivalent of the webapp's `release_control`).

## Config

Server port/heartbeat, safety watchdog timeout, stick deadband, per-mode
button→function bindings, and posture scalar limits all live in
[`config/webteleop.yaml`](config/webteleop.yaml) (documented inline).
Velocity caps and the animation list come from
`hexa_description/config/tuning.yaml` (SSoT — `gait_node` and
`posture_node` blocks), not from here.

## Running

`./hexa sim up` brings up sim + webteleop + teleop; open
`http://<container-ip>:8080`. In production `bringup.launch.py` includes
webteleop alongside the gamepad teleop. Tests:
`./hexa sim python3 -m pytest src/hexa_webteleop/test -q`.
