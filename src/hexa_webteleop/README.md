# hexa_webteleop

Web-app teleop — an HTTP + WebSocket server hosting a phone/tablet control
UI that publishes the **same** ROS topics as the gamepad teleop
(`hexa_teleop`): `/cmd_vel`, `/body/pose`, `/cmd_gait`, `/animation/mode`,
`/gait/initialize`, plus `/teleop/owner`. Subscribes `/gait/state` for
switch gating, and the latched `/cmd_gait` + `/animation/mode` themselves —
the only truth for the current selection (locomotion publishes no name
feedback), heard from **both** teleops and from its own publishes via
loopback. `/cmd_gait` does double duty: it drives the UI's status strip
*and* resyncs the node's stick velocity caps and gait cycler when the
gamepad switches gaits (`web_mapping.resync_gait`), so the two teleops
agree on more than the display. `/animation/mode` is display-only —
syncing the animation cycler would be dead code, since the shared
`map_joy` resets it to 0 on every ANIMATION-mode entry.

## Architecture

- **`web_mapping.py`** — pure Python (no rclpy). Loads webapp config and
  delegates to `hexa_teleop.joy_mapping.map_joy` for the full state
  machine (modes, init two-press, record, yaw easing, height, cycling).
  Unit-testable.
- **`webteleop_node.py`** — ROS glue. `aiohttp` server in a daemon thread
  + 60 Hz rclpy timer that maps input and publishes; shared state behind a
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
- **Status strip** — a compact readout above the button grid showing the
  current gait, animation mode (em dash until an animation is first
  latched) and pack voltage/current. Gait and animation are fed by the node
  from the latched command topics, so the strip follows gamepad-initiated
  switches too, and it stays visible while the take-control prompt replaces
  the grid — a controller owner switching gaits is exactly when the passive
  observer wants to see it.

Default stick mapping (config): left = forward/strafe (gait) or x/y
translation (posture); right = turn, plus forward on its Y axis so either
pad alone is a complete drive control (gait), or roll/pitch (posture). The
two forward sources sum, and the resulting velocity triple is fitted to the
reachable envelope by the shared `hexa_teleop` mapping — see that package's
README for what that does to the feel.
Default bottom-6 buttons: init, then per mode — quadruped init + gait
prev/next (gait) / record + yaw left/right (posture) / record + animation
prev/next (animation) — and height up/down. The quadruped init is the
four-legged half of the init button: from the belly it stands the robot up
with the middle pair left folded, and off the belly it folds like init does.
It takes gait mode's second slot because `record` only does anything in
posture mode, and it is bound in the gait section alone, which is what
confines it to gait mode. It selects `default_quadruped_gait`, which stays
out of `gait_cycle` — that init is the only way in, so prev/next can never
ask a standing robot for a leg set it cannot reach without folding. Once on
four legs, prev/next walks `quadruped_gait_cycle` instead, the four-corner
rotation; the six-leg selection keeps its own slot and is waiting after the
next fold. Posture mode still poses the body on four feet; only `record`
goes inert there, for the reason `hexa_teleop`'s README gives.

## Pack telemetry

The node subscribes `sensor_msgs/BatteryState` (`telemetry.battery_topic`,
default `/hexa_hardware_aux/battery_state` — the same topic the face and the
front-panel button read) with sensor QoS, and the webapp **polls** it over the
WebSocket every `telemetry.poll_period_s`, one reply per ask. Polled rather
than broadcast because the robot samples the pack at 10 Hz while the strip
needs it about once a second, and because a client that stops asking — a
backgrounded tab — stops the traffic by itself. The node pushes the period in
its `init` message, so it stays a config value rather than a constant in the
webapp.

Either value reads as a dash when it is unknown: nothing has published (the
sim has no hardware node), the last reading is older than
`telemetry.stale_after_s` so a dead hardware node cannot leave a frozen number
on screen, or the board reported a non-finite value — that last one is not
only defensive, since `NaN` is not JSON and would throw in the client's
`JSON.parse`. The rule is pure and unit-tested (`battery_payload`).

## Safety

The phone link is unreliable; three independent guards stop motion when it
goes quiet:

- **WebSocket heartbeat** — server pings every `server.ws_heartbeat_s` and
  force-closes on a missed pong, turning a half-open TCP connection into a
  real disconnect (which zeroes inputs and releases control).
- **Input watchdog** — the 60 Hz timer feeds neutral input whenever no
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
- **everything else** — redirected to `/`. Nothing 404s, because on the
  robot's hotspot (`systemd/network-mode.sh`) the AP answers every hostname
  with the robot, so an unrecognised path is a person who wants the
  controller. Leaving the OS connectivity probes (`/generate_204`,
  `/hotspot-detect.html`, …) among them is what makes a joining phone declare
  a captive portal and open the controller by itself. Rules in
  `captive_portal.py` (pure, unit-tested); the redirect goes to the root
  rather than serving the page in place because the webapp's asset paths are
  relative.

The server also binds `server.portal_port` (80) alongside its own, so
`http://control.hexa/` needs no port typed and the probes — which are plain
port 80 — arrive at all. Best-effort: a privileged port needs a root container
(the robot's is, the sim's is not), and a refusal is logged and ignored.

## Config

Server port/heartbeat, safety watchdog timeout, pack-telemetry topic and
poll period, stick deadband, and per-mode button→function bindings live in
[`config/webteleop.yaml`](config/webteleop.yaml) (documented inline).
Velocity caps, the animation list, and the posture-mode scalar limits come
from `hexa_description/config/tuning.yaml` (SSoT — `gait_node`,
`posture_node` and `teleop_node` blocks), not from here. The scalar limits
being shared is what makes the webapp pose the body over the same range a
gamepad does.

## Running

`./hexa sim up` brings up sim + webteleop + teleop; open
`http://<container-ip>:8080`. In production `bringup.launch.py` includes
webteleop alongside the gamepad teleop. Tests:
`./hexa sim python3 -m pytest src/hexa_webteleop/test -q`.
