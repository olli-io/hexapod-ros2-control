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
syncing the animation cycler would be dead code, since the shared state
machine resets it to 0 on every ANIMATION-mode entry.

## Architecture

- **`web_mapping.py`** — pure Python (no rclpy). Loads webapp config and
  delegates to `hexa_teleop.joy_mapping.map_functions` for the full state
  machine (modes, init two-press, record, yaw easing, height, cycling) —
  the same machine the gamepad runs, entered by function name rather than
  through a key layout the webapp does not have. Unit-testable.
- **`webteleop_node.py`** — ROS glue. `aiohttp` server in a daemon thread
  + 60 Hz rclpy timer that maps input and publishes; shared state behind a
  `threading.Lock`. Single-connection policy: a second device gets `busy`
  and is closed, retrying until the slot frees.
- **`web/`** — the webapp: React 19 + TypeScript sources in `web/src/`, its four
  views file-based routes under `web/src/app/`, built by Vite into a single
  inlined `web/dist/index.html`. `web/dist/` is **committed** and is what
  `setup.py` installs and the node serves; see [Frontend](#frontend) for why, and
  for the rules the bundle has to obey.

## Webapp UI

- **Tab bar** (symbols only, bottom bar in portrait / left strip in landscape)
  — four tabs, each swapping the view above it: **Control** (a joystick; green
  while a controller owns `/cmd_vel`, since that is the view whose mode column
  becomes the take-control prompt), **Mode** (a six-legged body whose middle pair dims on
  four legs, so the bar carries the current leg set without the view being open;
  it shows the leg set rather than the preset, so NORMAL, FAST and OFFROAD share
  one icon), **Network** (a wifi symbol keeping its connection colour) and
  **Log**. The tab now showing is drawn in `#FABD2F`, which outranks those status
  tints — which view you are looking at is the one thing the bar must not get
  wrong, and each view says the rest in words. The bar never leaves the screen,
  so no view carries a back arrow and Control is the way back from all of them.
- **Control area** — two touch joysticks flanking **three mode buttons** (Gait /
  Posture / Anim) — a column between the circles in landscape, a row in
  portrait, where the circles are stacked and a square in the middle would spend
  height they want — and that is all the middle of the screen holds: every other
  function is at a joystick corner, and preset and gait selection are the Mode
  view's. Each button names the **function** it asks for and carries its own
  caption (`web/src/components/ModeStack.tsx`), so there is no binding elsewhere
  for the text to drift from. While a controller owns control the mode buttons
  become an inline "Take control" prompt and the sticks are disabled.
- **Corner buttons** — the functions a thumb must reach without leaving its
  stick sit at the corners of the joystick circles: body up / body down on the
  right circle's left corners, yaw left and yaw right on the outer **top**
  corner of the left and right circle, wiggle left and wiggle right on the outer
  **bottom** corner of those same two, **stand** on the left circle's
  bottom-right and **save pose** (the `record` function) on the left circle's
  top-right. They are positioned against each circle's own bounding square
  (`.joystick-pad`), so they follow the sticks in both orientations. Which
  corners a mode offers is one table, `CORNERS` in `web/src/app/index.tsx`, read
  both by the JSX and by the pass that releases what a mode change takes off the
  screen — a corner cannot be on screen in a mode the release pass thinks it is
  gone from. Stand is offered in every mode; height, yaw and wiggle in **gait and
  posture**; record in **posture** alone. **Animation mode keeps stand, and
  spends the right circle's two bottom corners on animation prev / next** — the
  animation pair and the height-down / wiggle pair are never on screen together,
  since that is the one mode offering neither: the animation is driving the
  body, so a pose trimmed, wiggled or saved underneath it is a pose fighting the
  animation.
  Yaw's and wiggle's absence there also matches the gamepad, which leaves
  `l1`/`r1`/`l2`/`r2` unbound in that mode; height is a deliberate divergence,
  still live on the gamepad and still acted on by `map_web`, just not offered
  here. All of them are gone with the sticks while a controller owns control.
- **Stand** is the plain stand button, not a leg-set one: from the belly it
  stands the robot on the last six-leg preset it applied, and off the belly it
  folds. It is **red while the robot is folded** — the one press that has to
  happen before anything else on screen does anything, and the only piece of
  robot state a button here reports about itself. It is the **only** stand on
  the Control view: the four-legged one is gone, and the quadruped leg set is
  reached by picking the QUAD preset in the Mode view, from a stand.
- **Status strip** — a compact readout above the mode column showing the
  current **preset**, gait, animation mode (em dash until an animation is first
  latched) and pack voltage/current. Each item is keyed by a lucide icon rather
  than a word: the strip is held to the mode column's width, and MODE / GAIT /
  ANIM / PACK spent a third of it repeating what a reader learns once. The preset is here because the Mode tab icon
  cannot carry it: that icon shows the leg set, and NORMAL, FAST and OFFROAD all
  stand on six legs. It reads `/gait/preset`, so it is a dash until the engine
  has spoken — the Mode view lights no tile then either. Gait and animation are
  fed by the node from the latched command topics, so the strip follows
  gamepad-initiated switches too, and it stays visible while the take-control
  prompt replaces the mode column — a controller owner switching gaits is
  exactly when the passive observer wants to see it.

Default stick mapping (config): left = forward/strafe (gait) or x/y
translation (posture); right = turn, plus forward on its Y axis so either
pad alone is a complete drive control (gait), or roll/pitch (posture). The
two forward sources sum, and the resulting velocity triple is fitted to the
reachable envelope by the shared `hexa_teleop` mapping — see that package's
README for what that does to the feel.
No function but the mode select is in the middle of the screen. Height, yaw,
wiggle, stand, record and animation prev/next all sit at the joystick corners
above — height, yaw and wiggle because they are held rather than tapped, the rest
because they are what the operator reaches for mid-drive, and the middle of the
screen is the one place a thumb on a stick cannot get to. Gait selection is the
Mode view's, a button per gait rather than the prev/next pair that used to sit in
a 3×3 grid here, and nothing filled the gaps that left in gait and posture mode —
`record` is read only in posture mode, so a button for it in gait mode would be a
button that does nothing. The column is what is left once the empty
cells are admitted to be empty, and it makes each mode button a third of the
panel to hit.
The four-corner **leg set** has no button on this view. `map_web` still resolves
the `quadruped_mode` function — the mapping is shared with the gamepad, whose
select button is the producer — but nothing in the web UI asks for it, so a web
operator on the belly stands on six legs and picks QUAD from the stand, one step
further than the gamepad's route. Posture mode still poses the body on four
feet; only `record` goes inert there, for the reason `hexa_teleop`'s README
gives.

## Mode view

A tab opening a view that offers the operator **presets** — `NORMAL` (six
legs, everyday stance), `FAST` (low, long-striding), `OFFROAD` (tall, short
steps, high clearance) and `QUAD` (four corners, middle pair parked) — switches
between them, and offers the **gaits** the one in force walks. The two live in
one view because they are one choice made in two steps: a preset is a leg set and
a stance, and its rotation is the gaits that walk it. Tapping one publishes its id on `/cmd_preset` and that
preset's remembered gait on `/cmd_gait`, in that order, since the engine measures
a gait against the preset in force. The engine then runs a **preset change** in
place, without folding to the belly: a reseat onto the new footprint, plus the
middle pair's own move on either side of it where the two presets differ in leg
set. See `hexa_teleop`'s README for what a preset is and `docs/leg-phases.md`
for what the robot actually does.

Both are **grids of tiles**, laid out to the screen rather than stacked: portrait
puts them two abreast, landscape — which has the width and lacks the height —
lays each grid out in a single row. A tile is what a thumb hits; a column of
full-width rows wasted the width on a phone and did not fit at all on a landscape
one.

A **route** that replaces the control area — not an overlay over it, and not a
page of its own. Full-screen because the choice is the whole task while it is
open and a preset tile is a thing you tap on a phone; a client-side route because
pending and refused are live states and only the WebSocket carries them, and a
navigation that fetched a document would drop the socket — the server hands its
one client slot to whoever reconnects. The tab bar stays put, so the Mode tab
opens the view and the Control tab leaves it, and both sticks are re-centred on
the way in: the Control route unmounts, and a knob held at that moment never sees
its own touchend, so each canvas commands zero as it goes.

- **The active tile comes from `/gait/preset` and nothing else** — never the tap,
  never the latched `/cmd_preset`. That command topic keeps a refused id forever,
  so a view reading it would show a mode the robot never took, with nothing to
  correct it. Nor can it come from `/gait/leg_set`: `NORMAL`, `FAST` and
  `OFFROAD` all stand on six legs. Before the first `/gait/preset` arrives no tile
  is lit, which is honest rather than a guess.
- The client sets no optimistic state. During the ~2 s change the current tile
  stays filled and the target tile goes dashed — "on six, going to four" — and
  every tile is inert until it lands.
- **On the belly a `STAND` button replaces the preset grid outright.** The grid
  is not rendered there at all, rather than shown inert beside a third thing to
  press: off the belly a mode is a preset change; on it, the leg set is chosen by
  the stand itself — every init edge asks for a leg set, and the engine resolves
  that to a preset — so a four-corner mode tapped here would be overwritten the
  moment the robot got up.
  The button asks for the same `init` function the grid's stand slot does,
  rather than reaching for `/gait/initialize` directly, so standing up stays one
  path through the shared state machine, two-press revert and all. It is
  exempt from arbitration exactly as a mode switch is (below), which also makes
  it the only stand a webapp can reach while a controller drives — the grid it
  presses is a take-control prompt in that state.
- **A second grid under the presets, a tile per gait in the preset in force.**
  Same size and shape as a preset tile, so the view reads as one choice made
  twice. The column count does not follow the rotation's length — a rotation is
  two to five long, and a count that changed with it would move every tile on a
  preset switch — so a portrait rotation of five ends on a half-empty row. One
  press to any of them, where the grid's old prev/next pair took up to four and
  never said what the choices were. The names come from that preset's
  `gait_cycle`, shipped with the grid in the `init` message and indexed by
  `/gait/preset`, so the grid can only ever offer gaits that walk the legs the
  robot is standing on — the guarantee the cycler's arithmetic used to give for
  free. Tapping one publishes it on `/cmd_gait`, and the lit tile is what that
  topic last carried, never the tap. Live on the belly too: no gait runs there,
  but `/cmd_gait` is latched, so the lit tile is what the next stand will come up
  walking — worth picking before standing rather than after. The grid is inert
  while a preset change is in flight, since that switch has already published the
  new preset's entry gait and one from the old rotation landing behind it would
  sit latched as a gait the engine refuses. The node re-checks both rules before
  publishing; the client's job is only to not offer what it knows is wrong.
- A switch is **refused** where the engine would not take one: the node gates on
  `/gait/state` before publishing (a preset change is legal only from a stand,
  unlike a plain gait switch), and it holds a deadline for the cases it cannot
  predict — the state moved between the tap and the tick, or the operator's body
  pose never came back to neutral. `/gait/preset` publishes on change, so silence
  past the deadline is the answer. Either way a reason appears under the
  grids and the active tile does not move.

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
- The two panels stack in portrait and stand **side by side in landscape**,
  which is where the view is short and wide: two panels each ending in a button
  spend height there is none of, across width there is plenty of.

## Log view

Recent output from `GET /logs`, fetched when the route mounts — i.e. each time
the tab opens — and on its refresh button. A route rather than the standalone
page it used to be: leaving `index.html` dropped the WebSocket, and the server
hands its one client slot to whoever reconnects, so reading the log cost the
reader control of the robot. Not polled — it is a thing you go and read, and the
socket beside it is carrying control input. Reachable with the link down, which
is why it takes no session state at all.

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
drives, which is the point of it. Its **gait grid is exempt for the same reason**,
and it is one write rather than two. **An init — a stand or a fold — is exempt for
the same reason**: one `Empty` on `/gait/initialize` plus the gait naming the leg
set it wants, discrete rather than streamed. Without that the Mode view's `STAND`
would be dead in the one state it exists for, since the Control view's own
stand goes with the take-control prompt whenever a controller owns. The mapping's
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
topic and poll period, stick deadband, and the per-mode **stick** tables live
in [`config/webteleop.yaml`](config/webteleop.yaml) (documented inline). There
are no button bindings there: the webapp has no keys, so a button sends the
function name, and the mode column's layout lives with its captions in
`web/src/components/ModeStack.tsx` (the joystick corners' in `CORNERS`). The sticks stay here because which function each
drives depends on the mode, and the node is what knows the mode. The
**physical** half — the leg set, the standing pose and the stride/swing bundle —
is `hexa_description/config/tuning.yaml`'s `gait_node.presets` list under the
same ids, and is never restated here; adding a preset is an edit to those two
files. Velocity caps (per preset now, since a preset owns the stride, the swing
time and the stance they are derived from), the animation list, and the
posture-mode scalar limits come from that same `tuning.yaml` (SSoT — `gait_node`,
`posture_node` and `teleop_node` blocks), not from here. The scalar limits
being shared is what makes the webapp pose the body over the same range a
gamepad does.

## Frontend

The webapp is React 19 + TypeScript on TanStack Router, bundled by Vite. Unlike
the rest of the repo it builds on the **host**, not in a container — no image
carries node, and none needs to.

The build runs the **React Compiler** (`babel-plugin-react-compiler`, targeting
19) and minifies with **terser** (`drop_console`, two passes — ~2.5 kB gzipped
better than the esbuild default, on a page a phone pulls over the robot's own
hotspot). The compiler is conservative by design and skips what it cannot prove
safe: `Joystick`, `HoldButton`, the Control route and `useTeleopSocket` all write
a ref during render so hand-attached listeners can read live values, and all four
come out exactly as written. Nothing here depends on that memoization for
correctness — it is a compile step, not a design.

- `web/src/app/` — the **routes**, one file per view: `index.tsx` (Control, the
  home route), `preset.tsx`, `network.tsx`, `log.tsx`, and `__root.tsx`, the
  shell that holds the tab bar and the busy overlay around an `<Outlet/>`. File
  names *are* the paths — `@tanstack/router-plugin` generates
  `src/routeTree.gen.ts` from this directory, which is **committed** because
  `npm run build` type-checks before Vite runs and a fresh checkout has to
  type-check.
- `web/src/` — everything the routes are made of, sorted by what a file *is*:
  `hooks/useTeleopSocket.ts` holds every piece of server state in one reducer
  (one case per `/ws` message type); `types/protocol.ts` types the wire contract
  both ways; `utils/views.ts` is the tab order and each tab's path, so the bar
  and the routes agree by construction, and `utils/labels.ts` is the display
  strings. `session.tsx` — that socket held **above the router**, since the
  server has a single client slot and a link owned by a route would drop the
  robot every time somebody looked at the log — stays at the top beside
  `main.tsx`, with the routes it wraps. `components/Joystick.tsx` is
  deliberately imperative — canvas drawing and hand-attached listeners, no React state — because a
  touchmove fires per frame and its handlers need `preventDefault`, which
  React's passive synthetic events cannot do; `components/HoldButton.tsx` is
  there for the same reason — every button on the Control view reports the press
  *and* the release, so the mode buttons and the joystick corners share one set
  of hand-attached touch listeners.
- **The router runs on a hash history.** The server serves one page and answers
  every other path with a 302 to `/` — deliberately, since that redirect is what
  makes a joining phone declare a captive portal — so a reload on `/network`
  would come back as the Control view. Behind a `#` a route is never a path the
  server has to know about, and the two rules stop having an opinion about each
  other.
- **One route is mounted at a time**, which is what a router buys over the four
  hidden `<div>`s this replaced: each view's state is created and disposed with
  it. The Log fetches on mount instead of on a nonce the shell bumps; the Control
  route owns the keepalive, the set of held functions and the joystick handles,
  and releases every one on the way out — a button under a thumb when the view
  leaves the screen never sees its own touchend, and each canvas commands zero in
  its own cleanup. A mode change is the same hazard in miniature: every corner
  the new mode does not offer is gone from the screen, so anything held there is
  released for the same reason — in animation mode that is all of them but
  stand; the right circle's two bottom corners are what the animation pair takes
  over. The three mode
  buttons are offered in every mode and so are never the stale one.
- `web/dist/` — the built bundle, **committed to git**. It has to be: the ARM64
  robot image builds `robot.Dockerfile` from a bare checkout (`hexa deploy
  build`, and the release workflow), and its builder stage has no node. A
  rebuild that is never committed ships a robot with no UI, which is why the
  pytest suite asserts the bundle is a built one.
- **The bundle is a single file.** `captive_portal.static_filename` refuses any
  path with a `/` in it, and `_handle_get` answers a refusal with a 302 to `/`
  rather than a 404 — so a file under an `assets/` subdirectory fails
  *silently*, handing the browser the HTML page in place of the script it asked
  for. `vite-plugin-singlefile` inlines the script and the stylesheet into
  `index.html`, so there is nothing beside the page to get wrong; code splitting
  is off (`autoCodeSplitting: false`, `inlineDynamicImports`) for the same
  reason — a split route would be a second file to fetch. `vite.config.ts` still
  pins flat, unhashed names for anything too large to inline. Hashing would buy
  nothing anyway: every response carries `Cache-Control: no-store`. Fixed names
  have a second payoff — colcon's `--symlink-install` links stay valid, so a
  rebuild refreshes a running sim with no colcon run.

```
cd src/hexa_webteleop/web
npm ci          # first time, or after a dependency change
npm run build   # type-checks, then writes web/dist — commit the result
npm run dev     # host dev server on :5173, /ws and /logs proxied to :8080
```

`npm run dev` wants a robot to talk to: bring one up with `./hexa sim up` first.
Node 20.19+ or 22.12+ (see `.nvmrc`).

## Running

`./hexa sim up` brings up sim + webteleop + teleop; open
`http://<container-ip>:8080`. In production `bringup.launch.py` includes
webteleop alongside the gamepad teleop. Tests:
`./hexa sim python3 -m pytest src/hexa_webteleop/test -q`. The webapp itself is
not covered by that suite beyond the bundle checks — verify UI changes against
the sim.
