# hexa_webteleop

Web-app teleop for the hexapod — a lightweight HTTP + WebSocket server
that hosts a phone/tablet control UI and publishes the same ROS topics
as the gamepad teleop (`hexa_teleop`).

## What it does

- Hosts a static webapp (pure HTML + JavaScript, no build step) on port
  8080. The webapp renders two touch joysticks and 9 buttons.
- Relays stick/button events over WebSocket to the ROS side, where a
  pure mapping function (`web_mapping.py`) translates them into
  `/cmd_vel`, `/body/pose`, `/cmd_gait`, `/animation/mode`, and
  `/gait/initialize` — the exact same topics the gamepad teleop
  publishes.
- Coexists with the gamepad teleop via `/teleop/owner` arbitration. The
  gamepad owns by default; the webapp must explicitly claim control via
  the take-control prompt or the navbar controller toggle. See
  **Coexistence** below.

## Architecture

- **`web_mapping.py`** — pure Python (no rclpy). Loads the webapp
  config and delegates to `hexa_teleop.joy_mapping.map_joy` for the
  full state machine (mode switching, init two-press, record, yaw
  easing, height integration, gait/animation cycling). Unit-testable
  standalone.
- **`webteleop_node.py`** — ROS glue. Runs an `aiohttp` HTTP + WS
  server in a daemon thread and a 50 Hz rclpy timer that maps input and
  publishes. Thread-safe shared state via a `threading.Lock`.
  Single-connection policy: only one webapp may hold the `/ws` socket at
  a time. A second device is sent a `busy` message and its socket is
  closed; its client keeps retrying and connects once the slot frees.
- **`web/`** — static webapp: `index.html` + `main.js` (navbar,
  joysticks, buttons, WS, control handover), `logs.html` + `logs.js`
  (log viewer), shared `styles.css`. No TypeScript, no build step, no
  npm dependencies.

## Webapp layout

- **Navbar** — symbols only. Responsive: a horizontal bar across the top
  in portrait, a vertical strip down the left in landscape. It holds a
  wifi on/off connection indicator (green connected, red disconnected),
  a controller icon (green while a controller owns `/cmd_vel`; tap for a
  status popover with a switch-to/from-controller toggle), and a log
  icon (opens the log page).
- **Control area** — two touch joysticks flanking a 3x3 button grid.
  While a controller is active the grid is replaced in place by an
  inline "Take control" prompt and the joysticks are disabled.

## HTTP endpoints

Alongside the `/ws` WebSocket, the server exposes two plain HTTP
endpoints:

- **`GET /logs`** — runs the configured `logs.command` shell command and
  returns `{"lines": [...]}` with its last `logs.lines` lines (default
  200). The command is environment-specific and set in
  `config/webteleop.yaml`; the default concatenates the most recent ROS
  log files under `~/.ros/log`. Used by the log page.
- **`POST /control/release`** — hands control back to the gamepad
  (same effect as a webapp disconnect); returns `{"owner": ...}`. The
  webapp itself releases via the `release_control` WS message; this
  endpoint remains for out-of-band use.

## Safety

The link to a phone is unreliable — WiFi drops, the screen locks, the
tab is backgrounded — and a naive relay would keep republishing the
last stick value at 50 Hz, walking the robot away with nobody holding
it. Three independent guards stop motion when the link goes quiet:

- **WebSocket heartbeat** — the server pings each client every
  `server.ws_heartbeat_s` and force-closes a socket that misses its
  pong. This turns a half-open TCP connection (no clean FIN) into a real
  disconnect, which runs the cleanup path (zero inputs, release control).
- **Input watchdog** — the 50 Hz timer feeds neutral input (centred
  sticks, released buttons) to the mapping whenever no stick/button
  message has arrived within `safety.input_timeout_s`, so `/cmd_vel`
  falls to zero instead of latching. This is the backstop that covers
  any way the stream can stall, including a browser suspended before it
  can send anything. The decision (`input_is_stale`, `neutral_inputs`)
  is pure Python and unit-tested.
- **Client-side visibility stop** — the webapp re-centres both sticks on
  `visibilitychange → hidden`, so a tab switch or screen lock commands
  zero immediately rather than waiting out the watchdog timeout.

On disconnect the node also zeroes the shared stick/button state so a
freshly connecting device cannot inherit the departed one's inputs.

## Topics

Published (same as `teleop_joy.py`):
- **`/cmd_vel`** (`geometry_msgs/Twist`) — body velocity for the gait chain
- **`/body/pose`** (`hexa_interfaces/msg/BodyPose`) — body-pose offset for the posture chain
- **`/cmd_gait`** (`std_msgs/String`, TRANSIENT_LOCAL) — gait selection by name
- **`/animation/mode`** (`std_msgs/String`, TRANSIENT_LOCAL) — animation selection
- **`/gait/initialize`** (`std_msgs/Empty`) — one-shot fold/initialize trigger
- **`/teleop/owner`** (`std_msgs/String`, TRANSIENT_LOCAL) — current owner: `gamepad` or `web`

Subscribed:
- **`/gait/state`** (`std_msgs/String`) — gait-engine state for switch gating

## Coexistence with the gamepad teleop

Both teleop nodes run simultaneously; only one publishes at a time. A
single latched `/teleop/owner` topic carries the current owner:

- **`gamepad`** (default) — the gamepad node publishes.
- **`web`** — the web node publishes; the gamepad node goes dormant.

Protocol (only the web node writes `/teleop/owner`):
- Cold start, no webapp connected → gamepad owns by default.
- Webapp connects → receives current owner. If `gamepad`, the navbar
  controller icon goes green and the button grid is replaced by an
  inline "Take control" prompt; the webapp is a passive observer.
- User taps **Take control** (or the navbar controller toggle) → webapp
  sends `request_control` → web node publishes `web` → gamepad node
  stops publishing.
- User taps the navbar controller toggle while in control → webapp sends
  `release_control` → web node publishes `gamepad` → gamepad resumes.
- Webapp disconnects → web node publishes `gamepad` → gamepad resumes.

The arbitration logic lives in `hexa_teleop.teleop_arbitration` (pure
Python, shared by both nodes, unit-tested).

## Webapp controls

- **Two touch joysticks** — left and right, each an x/y pair.
- **9 buttons** — top 3 are fixed mode-select (Gait / Posture / Anim),
  bottom 6 are mode-dependent (the node sends labels to the webapp on
  mode change so the UI relabels dynamically). A button press triggers a
  short haptic tick via the Vibration API where the browser supports it
  (Android Chrome; a no-op on iOS Safari).

Default stick mapping (configurable in `config/webteleop.yaml`):
- **Gait / Animation**: left stick = forward/strafe, right stick X = turn.
- **Posture**: left stick = body x/y translation, right stick = roll/pitch tilt.

Default button mapping (bottom 6, per mode):
- **Gait**: init, record, gait prev/next, height up/down
- **Posture**: init, record, yaw left/right, height up/down
- **Animation**: init, record, animation prev/next, height up/down

## Config

All tunable values live in `config/webteleop.yaml`:
- Server port and WebSocket heartbeat interval (`server.ws_heartbeat_s`)
- Safety input-watchdog timeout (`safety.input_timeout_s`)
- Stick deadband
- Per-mode button→function bindings
- Posture-mode scalar limits (x/y max, roll/pitch/yaw max, height range)

Velocity caps are loaded from `hexa_gait/config/gait.yaml` (single
source of truth); the animation list from
`hexa_posture/config/posture.yaml`.

## Running

In sim (dev container):
```
pod sim
ros2 launch hexa_webteleop webteleop.launch.py
```
Then open `http://<container-ip>:8080` in a browser.

In production, `prod.launch.py` includes webteleop alongside the
gamepad teleop automatically.

## Testing

```
./hexa --dev python3 -m pytest src/hexa_webteleop/test -q
```

Tests cover config loading, button-label resolution, stick→cmd_vel
mapping, mode switching, gait cycling, and deadband — all pure-Python,
no ROS context needed.
