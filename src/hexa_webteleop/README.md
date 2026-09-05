# hexa_webteleop

Web-app teleop — an HTTP + WebSocket server hosting a phone/tablet control
UI that publishes the **same** ROS topics as the gamepad teleop
(`hexa_teleop`): `/cmd_vel`, `/body/pose`, `/cmd_gait`, `/cmd_preset`,
`/animation/mode`, `/gait/initialize`, plus `/teleop/owner`. Subscribes
`/gait/state` for switch gating, and the latched `/cmd_gait` + `/cmd_preset` +
`/animation/mode` themselves — the only truth for the current *selection*
(locomotion publishes no gait-name feedback), heard from **both** teleops and
from its own publishes via loopback. It also subscribes the engine's two report
topics: `/gait/preset`, the preset it has applied and the Mode view's only source
of truth, and `/gait/leg_set`, which drives the Mode tab icon alone — three of the
four presets stand on six legs, so the leg set cannot tell them apart.
`/cmd_gait` does double duty: it drives the UI's status strip *and* resyncs the
node's stick velocity caps and gait cycler when the gamepad switches gaits
(`web_mapping.resync_gait`), so the two teleops agree on more than the display. `/animation/mode` is display-only —
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
- **`web/`** — static webapp (`index.html` + `main.js` + `styles.css`), one
  page holding all four views. No TypeScript, no build step, no npm.

## Webapp UI

- **Tab bar** (symbols only, bottom bar in portrait / left strip in landscape)
  — four tabs, each swapping the view above it: **Control** (a joystick; green
  while a controller owns `/cmd_vel`, since that is the view whose grid becomes
  the take-control prompt), **Mode** (a six-legged body whose middle pair dims on
  four legs, so the bar carries the current leg set without the view being open;
  it shows the leg set rather than the preset, so NORMAL, FAST and OFFROAD share
  one icon), **Network** (a wifi symbol keeping its connection colour) and
  **Log**. The tab now showing is drawn in `#FABD2F`, which outranks those status
  tints — which view you are looking at is the one thing the bar must not get
  wrong, and each view says the rest in words. The bar never leaves the screen,
  so no view carries a back arrow and Control is the way back from all of them.
- **Control area** — two touch joysticks flanking a 3×3 button grid; top 3
  buttons select mode (Gait / Posture / Anim), bottom 6 are
  mode-dependent (node pushes labels on mode change). While a controller
  owns control the grid becomes an inline "Take control" prompt and the
  sticks are disabled.
- **Status strip** — a compact readout above the button grid showing the
  current **preset**, gait, animation mode (em dash until an animation is first
  latched) and pack voltage/current. The preset is here because the Mode tab icon
  cannot carry it: that icon shows the leg set, and NORMAL, FAST and OFFROAD all
  stand on six legs. It reads `/gait/preset`, so it is a dash until the engine
  has spoken — the Mode view lights no row then either. Gait and animation are
  fed by the node from the latched command topics, so the strip follows
  gamepad-initiated switches too, and it stays visible while the take-control prompt replaces
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
confines it to gait mode. It asks for the four-corner **leg set**, which the
engine resolves to the QUAD preset — so prev/next can never ask a standing robot
for a leg set it is not on. Once on four legs, prev/next walks the QUAD rotation
instead; the six-leg selection keeps its own
slot and is waiting when the robot comes back. The Mode view is the other way
in, and unlike this button it does not need the belly. Posture mode still poses the body on four feet; only `record`
goes inert there, for the reason `hexa_teleop`'s README gives.

## Mode view

A tab opening a view that lists the operator **presets** — `NORMAL` (six
legs, everyday stance), `FAST` (low, long-striding), `OFFROAD` (tall, short
steps, high clearance) and `QUAD` (four corners, middle pair parked) — and
switches between them. Tapping one publishes its id on `/cmd_preset` and that
preset's remembered gait on `/cmd_gait`, in that order, since the engine measures
a gait against the preset in force. The engine then runs a **preset change** in
place, without folding to the belly: a reseat onto the new footprint, plus the
middle pair's own move on either side of it where the two presets differ in leg
set. See `hexa_teleop`'s README for what a preset is and `docs/leg-phases.md`
for what the robot actually does.

A view that **replaces the control area** in `index.html` — not an overlay over
it, and not a page of its own. Full-screen because the list is the whole task
while it is open and a preset row is a thing you tap on a phone, in this page
because pending and refused are live states and only the WebSocket carries them;
navigating away would drop the socket, and the server hands its one client slot
to whoever reconnects. The tab bar stays put, so the Mode tab opens the view and
the Control tab leaves it, and both sticks are re-centred on the way in — they
leave the screen, and a knob held at that moment never sees its own touchend.

- **The active row comes from `/gait/preset` and nothing else** — never the tap,
  never the latched `/cmd_preset`. That command topic keeps a refused id forever,
  so a view reading it would show a mode the robot never took, with nothing to
  correct it. Nor can it come from `/gait/leg_set`: `NORMAL`, `FAST` and
  `OFFROAD` all stand on six legs. Before the first `/gait/preset` arrives no row
  is lit, which is honest rather than a guess.
- The client sets no optimistic state. During the ~2 s change the current row
  stays filled and the target row goes dashed — "on six, going to four" — and
  every row is inert until it lands.
- **On the belly the rows are inert and a `STAND` button takes their place.**
  Off the belly a mode is a preset change; on it, the leg set is chosen by the
  stand itself — every init edge asks for a leg set, and the engine resolves that
  to a preset — so a four-corner mode tapped here would be overwritten the moment
  the robot got up.
  The button presses the grid's own `init` slot (found by label, not by a
  hardcoded index) rather than reaching for `/gait/initialize` directly, so
  standing up stays one path through `map_joy`, two-press revert and all. It is
  exempt from arbitration exactly as a mode switch is (below), which also makes
  it the only stand a webapp can reach while a controller drives — the grid it
  presses is a take-control prompt in that state.
- A switch is **refused** where the engine would not take one: the node gates on
  `/gait/state` before publishing (a preset change is legal only from a stand,
  unlike a plain gait switch), and it holds a deadline for the cases it cannot
  predict — the state moved between the tap and the tick, or the operator's body
  pose never came back to neutral. `/gait/preset` publishes on change, so silence
  past the deadline is the answer. Either way a reason appears under the
  list and the active row does not move.

## Network view

Link state and the controller handover, in one view, because they are one
question: which input the robot is listening to, and whether this device can
reach it at all. It replaces the two tab-bar popovers — a dialog over the
joysticks sat in front of a control the operator still had a thumb on, and the
handover is a state worth reading rather than a menu item to confirm.

- **Link** — connected/disconnected, the host, and a disconnect/reconnect
  toggle. A manual disconnect stays down; every other close retries.
- **Control** — who owns `/cmd_vel`, and the toggle that moves it:
  **Take control** while a controller drives, **Give control to controller**
  while the webapp does. The same handover the control area's inline prompt
  offers, which stays where it is: that one is the affordance in context, this
  one is the one that works from any tab and in either direction. With
  arbitration disabled the toggle is hidden and the view says why.

## Log view

Recent output from `GET /logs`, fetched when the tab opens and on its refresh
button. A view rather than the standalone page it used to be: leaving `index.html`
dropped the WebSocket, and the server hands its one client slot to whoever
reconnects, so reading the log cost the reader control of the robot. Not polled —
it is a thing you go and read, and the socket beside it is carrying control input.

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

Both nodes run simultaneously; only one publishes **drive** commands. A single
latched `/teleop/owner` (`gamepad` default, or `web`) arbitrates; only the web
node writes it.

A **Mode view switch is exempt**, deliberately: it touches neither `/cmd_vel`
nor `/body/pose` — the two continuous streams arbitration exists to stop from
fighting — and is two idempotent writes to latched selection topics both teleops
already read without asking anyone. So the Mode view works while a controller
drives, which is the point of it. **An init — a stand or a fold — is exempt for
the same reason**: one `Empty` on `/gait/initialize` plus the gait naming the leg
set it wants, discrete rather than streamed. Without that the Mode view's `STAND`
would be dead in the one state it exists for, since the button grid it presses is
replaced by the take-control prompt whenever a controller owns. The mapping's
bookkeeping stays honest either way — the gait publish loops back through
`/cmd_gait` into `resync`, in both teleops. The gamepad node follows the result by
subscribing `/cmd_gait` itself (see its README), so its D-pad never ends up
rotating a list the robot is not standing on. Webapp connects as a passive observer, then **Take
control** → `request_control` → owner `web`, gamepad goes dormant.
Releasing (toggle, disconnect, or `POST /control/release`) publishes
`gamepad` and resumes it. Logic lives in
`hexa_teleop.teleop_arbitration` (pure, shared, unit-tested).

## HTTP endpoints

Alongside `/ws`:

- **`GET /logs`** — returns `{"lines": [...]}` from the configured
  `logs.command` (default: recent `~/.ros/log` files), for the Log view.
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

The **operator** half of each preset (the Mode view's list, each one's label and
gait rotation), server port/heartbeat, safety watchdog timeout, pack-telemetry
topic and poll period, stick deadband, and per-mode button→function bindings live
in [`config/webteleop.yaml`](config/webteleop.yaml) (documented inline). The
**physical** half — the leg set, the standing pose and the stride/swing bundle —
is `hexa_description/config/tuning.yaml`'s `gait_node.presets` list under the
same ids, and is never restated here; adding a preset is an edit to those two
files. Velocity caps (per preset now, since a preset owns the stride, the swing
time and the stance they are derived from), the animation list, and the
posture-mode scalar limits come from that same `tuning.yaml` (SSoT — `gait_node`,
`posture_node` and `teleop_node` blocks), not from here. The scalar limits
being shared is what makes the webapp pose the body over the same range a
gamepad does.

## Running

`./hexa sim up` brings up sim + webteleop + teleop; open
`http://<container-ip>:8080`. In production `bringup.launch.py` includes
webteleop alongside the gamepad teleop. Tests:
`./hexa sim python3 -m pytest src/hexa_webteleop/test -q`.
