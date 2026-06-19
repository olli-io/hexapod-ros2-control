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
  a prompt. See **Coexistence** below.

## Architecture

- **`web_mapping.py`** — pure Python (no rclpy). Loads the webapp
  config and delegates to `hexa_teleop.joy_mapping.map_joy` for the
  full state machine (mode switching, init two-press, record, yaw
  easing, height integration, gait/animation cycling). Unit-testable
  standalone.
- **`webteleop_node.py`** — ROS glue. Runs an `aiohttp` HTTP + WS
  server in a daemon thread and a 50 Hz rclpy timer that maps input and
  publishes. Thread-safe shared state via a `threading.Lock`.
- **`web/`** — static webapp: `index.html`, `styles.css`, `main.js`.
  No TypeScript, no build step, no npm dependencies.

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
- Webapp connects → receives current owner. If `gamepad`, the webapp
  shows a prompt: *"The hexapod is currently connected to a controller.
  Control from here?"*
- User taps **Yes** → webapp sends `request_control` → web node
  publishes `web` → gamepad node stops publishing.
- Webapp disconnects → web node publishes `gamepad` → gamepad resumes.
- User taps **No** → webapp stays a passive observer.

The arbitration logic lives in `hexa_teleop.teleop_arbitration` (pure
Python, shared by both nodes, unit-tested).

## Webapp controls

- **Two touch joysticks** — left and right, each an x/y pair.
- **9 buttons** — top 3 are fixed mode-select (Gait / Posture / Anim),
  bottom 6 are mode-dependent (the node sends labels to the webapp on
  mode change so the UI relabels dynamically).

Default stick mapping (configurable in `config/webteleop.yaml`):
- **Gait / Animation**: left stick = forward/strafe, right stick X = turn.
- **Posture**: left stick = body x/y translation, right stick = roll/pitch tilt.

Default button mapping (bottom 6, per mode):
- **Gait**: init, record, gait prev/next, height up/down
- **Posture**: init, record, yaw left/right, height up/down
- **Animation**: init, record, animation prev/next, height up/down

## Config

All tunable values live in `config/webteleop.yaml`:
- Server port
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
