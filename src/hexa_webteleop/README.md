# hexa_webteleop

An HTTP + WebSocket server hosting a phone/tablet control UI. It publishes the
same ROS topics as the gamepad teleop (`hexa_teleop`), so the two are
interchangeable and only one drives at a time.

- **Publishes** — `/cmd_vel`, `/body/pose`, `/cmd_gait`, `/cmd_preset`,
  `/cmd_gesture`, `/animation/mode`, `/gait/initialize`, `/teleop/owner`.
- **Subscribes** — `/gait/state` (switch gating), `/gait/preset`,
  `/gait/leg_set` and `/gait/gesture` (the engine's reports), the latched
  `/cmd_gait`, `/cmd_preset` and `/animation/mode` (the current *selection*,
  heard from both teleops and from its own publishes), and
  `sensor_msgs/BatteryState`.

`/cmd_gait` does double duty: it drives the status strip *and* resyncs the
node's velocity caps and gait cycler when the gamepad switches gaits
(`web_mapping.resync_gait`).

## Layout

- **`web_mapping.py`** — pure Python, no rclpy. Loads the webapp config and
  delegates to `hexa_teleop.joy_mapping.map_functions` for the full state
  machine, entered by function name rather than by a key layout. Unit-tested.
- **`webteleop_node.py`** — ROS glue. `aiohttp` server in a daemon thread plus a
  60 Hz rclpy timer that maps input and publishes; shared state behind a lock.
  One connection at a time: a second device gets `busy` and is closed, and
  retries until the slot frees.
- **`web/`** — React 19 + TypeScript, built by Vite into a single inlined
  `web/dist/index.html`, which is committed. See [Frontend](#frontend).

## The views

A tab bar (symbols only; bottom in portrait, left strip in landscape) swaps the
view above it. Network is a grey cog with a wifi glyph beside it, green with the link up and
red with it down. The current tab is drawn in `#FABD2F`, which outranks any status
tint. The bar never leaves the screen, so no view carries a back arrow.

### Control

Two touch joysticks flanking three mode buttons (Gait / Posture / Anim) — a
column in landscape, a row in portrait. Nothing else sits in the middle of the
screen, because a thumb on a stick cannot reach it.

Every other function is at a joystick corner, positioned against each circle's
own bounding square so it follows the sticks in both orientations. `CORNERS` in
`web/src/app/index.tsx` is the one table of which mode offers which corner, read
both by the JSX and by the pass that releases a held button when a mode change
takes it off the screen.

- Right circle, left corners — body up / body down.
- Outer top corner of each circle — yaw left / yaw right.
- Outer bottom corner of each circle — wiggle left / wiggle right.
- Left circle, bottom-right — **stand**; top-right — **save pose** (`record`).
- Animation mode instead spends the right circle's two bottom corners on
  animation prev / next.

Stand is offered in every mode, height/yaw/wiggle in gait and posture, record in
posture alone. Stand is a plain stand, not a leg-set one: from the belly it
stands on the last six-leg preset applied, off the belly it folds. It is **red
while folded**.

A **status strip** above the mode column reads the current preset, gait,
animation and pack voltage/current, each keyed by an icon. It stays visible while
a controller owns control.

Default stick mapping (config): left = forward/strafe (gait) or x/y translation
(posture); right = turn, plus forward on its Y axis, or roll/pitch (posture).
The two forward sources sum, and `hexa_teleop`'s shared mapping fits the result
to the reachable envelope.

While a controller owns `/cmd_vel` the sticks and corners are disabled and the
mode buttons become an inline "Take control" prompt.

### Mode

Everything picked by name is picked here: **presets** — `NORMAL` (everyday),
`FAST` (low, long-striding), `OFFROAD` (tall, high clearance) and `QUAD` (four
corners, middle pair parked) — plus the **gaits** and **animations**, under a row
of the three modes. Three grids of tiles, two abreast in portrait and one row
each in landscape.

It is a client-side route, not an overlay: pending and refused are live states
that only the WebSocket carries, so a navigation that fetched a document would
drop the socket. Both sticks re-centre on the way in.

- Tapping a preset publishes its id on `/cmd_preset` and its **entry gait** on
  `/cmd_gait`, in that order. The entry gait is the one already in force where
  the new preset walks it, and the new preset's `default_gait` where it does not.
- **The lit preset tile comes from `/gait/preset` and nothing else** — never the
  tap, never the latched `/cmd_preset`, which keeps a refused id forever. Before
  the first report no tile is lit. During the ~2 s change the current tile stays
  filled, the target goes dashed, and every tile is inert.
- **On the belly a `STAND` button replaces the preset grid outright**, since the
  stand itself chooses the leg set. It asks for the same `init` function the
  Control view's stand does.
- The gait grid shows **every declared gait**; the ones the preset in force does
  not walk are dimmed in place, so the row never changes shape. Names come from
  the presets' own `gait_cycle`s, shipped in the `init` message. Live on the
  belly too — `/cmd_gait` is latched, so the lit tile is what the next stand
  comes up walking. Inert while a preset change is in flight.
- The animation grid is the `animation_mode_animations` list, dimmed outside
  animation mode rather than hidden.
- **Animation mode lives on one preset**, `presets.animation` in
  [`config/webteleop.yaml`](config/webteleop.yaml) — `normal`. `ANIM` is dimmed
  on every other preset and on four legs, off one predicate
  (`animationAvailable`) shared with the Control view's column. While the mode is
  in force every other preset tile is inert, so the way out is to leave the mode.
- A switch is **refused** where the engine would not take one (a preset change is
  legal only from a stand). The node gates on `/gait/state` and holds a deadline
  for what it cannot predict; a reason appears under the grids and the lit tile
  does not move.

Every button here is a tap, not a hold, and there is no keepalive.

### Gesture

One tile per gesture in `hexa_description/config/gestures.yaml`, whose ids the
node ships in the `init` message. A tap sends `select_gesture`; the node
publishes the id once on `/cmd_gesture` (volatile — a gesture is an event). The
lit tile is the engine's report on `/gait/gesture`, never the tap, and the tab
icon takes the accent while one plays.

The engine plays a gesture only from a stand on the default preset
(`tuning.yaml`'s `gait_node.default_preset`, shipped as `preset_gesture`), so
the view is gated the same way, off the engine's reports:

- Off that preset a **modal replaces the view** with the fix: a *Switch to
  NORMAL* button, which is the Mode view's own `select_preset` request and
  lands under the same rules — live from a stand only, pending until
  `/gait/preset` reports it, spinner meanwhile. On the belly or mid-walk the
  button is dimmed and the second button leads to the Mode view, where STAND is.
- On the preset but not standing, the tiles are dimmed with *Stand to activate*
  on the heading. While a gesture plays every tile is inert and the running one
  keeps its fill.
- The node pre-gates the request (`gesture_refusal`, pure and unit-tested) so a
  refusal is a sentence under the tiles rather than a line in the engine's log;
  the engine still has the last word.

Like a preset or gait switch, a gesture is exempt from `/cmd_vel` ownership: one
event that touches neither drive stream, and the engine holds the sticks off
for the duration itself.

### Network

Link state and the controller handover, because they are one question: which
input the robot listens to, and whether this device can reach it. Plus the way
to the log.

- **Link** — connected/disconnected, the host, and a disconnect/reconnect
  toggle. A manual disconnect stays down; every other close retries.
- **Control** — who owns `/cmd_vel`, and the toggle that moves it. Hidden with
  arbitration disabled, and the view says so.
- A **Fullscreen** button, never automatic — the app's only other gesture is a
  joystick drag. Absent on iPhone Safari, which needs none once installed.
- **Logs** — an *Open logs* button to the Log view below. A panel rather than a
  fifth tab: the log is read when the link or the robot misbehaves, which is
  what this view is for, and a phone's bar has no room for another symbol.
- The three panels stack in portrait and stand side by side in landscape.

### Log

Recent output from `GET /logs`, fetched on mount and on the refresh button. Not
polled. Takes no session state, so it works with the link down. Opened from the
Network view; the Network tab stays lit while it is up, and is the way back.

A row of level chips (`DEBUG`, `INFO`, `WARN`, `ERROR`, with `FATAL` folded into
`ERROR`) hides the levels turned off. A line with no level tag — a traceback, a
wrapped line — follows the tagged line above it. The rule is pure
(`web/src/utils/logs.ts`) and resets on every visit.

## Pack telemetry

The node subscribes `telemetry.battery_topic` (default
`/hexa_hardware_aux/battery_state`) with sensor QoS, and the webapp **polls** it
over the WebSocket every `telemetry.poll_period_s`, one reply per ask — the robot
samples at 10 Hz, the strip needs ~1 Hz, and a backgrounded tab stops the traffic
by itself. The period ships in the `init` message.

A value reads as a dash when nothing has published, the reading is older than
`telemetry.stale_after_s`, or the board reported a non-finite value (`NaN` is not
JSON and would throw in `JSON.parse`). The rule is pure and unit-tested
(`battery_payload`).

## Safety

The phone link is unreliable; three independent guards stop motion when it goes
quiet.

- **WebSocket heartbeat** — the server pings every `server.ws_heartbeat_s` and
  force-closes on a missed pong, turning a half-open TCP connection into a real
  disconnect.
- **Input watchdog** — the 60 Hz timer feeds neutral input whenever nothing
  arrived within `safety.input_timeout_s`, so `/cmd_vel` falls to zero instead of
  latching. Pure and unit-tested.
- **Client visibility stop** — the webapp re-centres both sticks on
  `visibilitychange → hidden`.

On disconnect the node zeroes shared state, so a new device cannot inherit the
departed one's inputs.

## Coexistence with gamepad teleop

Both nodes run at once; only one publishes **drive** commands. A single latched
`/teleop/owner` (`gamepad` default, or `web`) arbitrates, and only the web node
writes it. **Take control** → `request_control` → owner `web`, and the gamepad
goes dormant; releasing (toggle, disconnect, or `POST /control/release`) resumes
it. The logic is `hexa_teleop.teleop_arbitration` — pure, shared, unit-tested.

Preset switches, gait switches, gestures and inits are **exempt**: they touch
neither `/cmd_vel` nor `/body/pose`, and are one-shot or idempotent writes to
selection topics both teleops already read. So the Mode view works while a controller drives, which is
the point of it — and its `STAND` is the only stand a webapp can reach then.

## HTTP endpoints

Alongside `/ws`:

- **`GET /logs`** — `{"lines": [...]}` from `logs.command`.
- **`POST /control/release`** — hands control back to the gamepad.
- **everything else** — 302 to `/`. Nothing 404s: on the robot's hotspot the AP
  answers every hostname, so an unrecognised path is a person who wants the
  controller, and leaving the OS connectivity probes (`/generate_204`,
  `/hotspot-detect.html`, …) among them is what makes a joining phone declare a
  captive portal. Rules in `captive_portal.py` (pure, unit-tested).

The server also binds `server.portal_port` (80) alongside its own, so
`http://control.hexa/` needs no port typed and the probes arrive at all.
Best-effort — a privileged port needs a root container, and a refusal is logged
and ignored.

## Config

[`config/webteleop.yaml`](config/webteleop.yaml) holds the **operator** half of
each preset (the Mode view's list, its label and gait rotation), the server
port/heartbeat, the safety timeout, the pack topic and poll period, the stick
deadband, and the per-mode **stick** tables. There are no button bindings: a
button sends a function name, and the mode column's layout lives with its
captions in `web/src/components/ModeStack.tsx`.

The **physical** half of a preset — leg set, standing pose, stride/swing bundle —
is `hexa_description/config/tuning.yaml`'s `gait_node.presets` under the same
ids, and is never restated here. Velocity caps, the animation list and the
posture scalar limits come from that same file. Adding a preset is an edit to
those two files.

## Frontend

React 19 + TypeScript on TanStack Router, bundled by Vite. Unlike the rest of the
repo it builds on the **host** — no image carries node.

```
cd src/hexa_webteleop/web
pnpm install     # first time, or after a dependency change
pnpm build       # type-checks, then writes web/dist — commit the result
pnpm dev         # dev server on :5173, /ws and /logs proxied to :8080

./tools/render-icons.sh   # only after editing icon.svg; then build and commit
```

`pnpm dev` wants a robot to talk to — `./hexa sim up` first. Node 20.19+ or
22.12+ (see `.nvmrc`).

### Source layout

- `web/src/app/` — the routes, one file per view: `index.tsx` (Control, home),
  `preset.tsx`, `gesture.tsx`, `network.tsx`, `log.tsx`, and `__root.tsx`, the
  shell holding the tab bar around an `<Outlet/>`. File names *are* the paths;
  `src/routeTree.gen.ts` is generated from this directory and **committed**,
  because `pnpm build` type-checks first and a fresh checkout has to type-check.
- `web/src/hooks/useTeleopSocket.ts` — every piece of server state in one
  reducer, one case per `/ws` message type.
- `web/src/types/protocol.ts` — the wire contract, both ways.
- `web/src/utils/` — `views.ts` is the tab order and each route's path, so the
  bar and the routes agree by construction; `labels.ts` is the display strings;
  `logs.ts` is the log level filter.
- `web/src/providers/` — the two contexts `main.tsx` wraps the router in.
  `TeleopProvider` holds the socket **above the router**, since the server has
  one client slot and a link owned by a route would drop the robot whenever
  somebody read the log. `ModalProvider` puts a host on `<body>`: a modal written
  inside a view is a child of the view's box, and `#preset-view` is a grid in
  landscape, which sizes a `position: fixed` child to its grid area.
- `web/src/components/Joystick.tsx` and `HoldButton.tsx` are deliberately
  imperative — canvas drawing and hand-attached listeners, no React state —
  because touchmove fires per frame and needs `preventDefault`, which React's
  passive synthetic events cannot do.
- `web/public/` — the four files that cannot be inlined: the manifest and three
  icons. Vite copies them into `dist/` verbatim, flat at the root. `web/icon.svg`
  is the artwork and does not ship.

### Bundle rules

- **The bundle is a single file.** `captive_portal.static_filename` refuses any
  path containing `/`, and a refusal answers 302 to `/` rather than 404 — so a
  file under `assets/` fails *silently*, handing the browser the HTML page in
  place of the script. `vite-plugin-singlefile` inlines the script and stylesheet
  into `index.html`; code splitting is off for the same reason. Names are flat
  and unhashed, which keeps colcon's `--symlink-install` links valid; hashing
  would buy nothing, since every response carries `Cache-Control: no-store`.
- **`web/dist/` is committed to git.** The ARM64 robot image builds from a bare
  checkout and its builder stage has no node, so an uncommitted rebuild ships a
  robot with no UI. The pytest suite asserts the bundle is a built one.
- **The router runs on a hash history**, because the server serves one page and
  302s every other path. A reload on `/network` would otherwise come back as
  Control.
- **One route is mounted at a time**, so each view's state is created and
  disposed with it. The Control route owns the keepalive, the held functions and
  the joystick handles, and releases every one on the way out — a button under a
  thumb never sees its own touchend when the view leaves the screen.
- The build runs the **React Compiler** and minifies with **terser**
  (`drop_console`, two passes). Nothing depends on that memoization for
  correctness.

### pnpm

Pinned in `package.json`'s `packageManager` and locked by `pnpm-lock.yaml`. There
is no `package-lock.json`; `npm install` here would write one and a second,
divergent tree. Settings live in `pnpm-workspace.yaml` — pnpm 11 moved them:

- `allowBuilds: esbuild` — pnpm refuses to install while a dependency's install
  script is undecided, and esbuild's links its platform binary.
- `minimumReleaseAge: 20160` (two weeks) — the supply-chain guard. A newer
  version is not resolved at all, so a compromised release has two weeks to be
  caught.

The guard has a cost when bumping a dependency: a range whose *only* members are
younger than the cutoff fails outright (`ERR_PNPM_NO_MATURE_MATCHING_VERSION`),
so `^`-ranges name the newest version that had matured when they were last
touched. `lucide-react` and `terser` are behind latest for that reason. Raise a
range only to something the policy accepts, or wait — `minimumReleaseAgeExclude`
disables the guard for exactly the package it exists to cover.

## The app shell

The webapp installs to a phone's home screen — not for offline use, but for the
screen: a joystick UI loses a URL bar and a toolbar, a stray swipe can
pull-to-refresh mid-walk, and an ordinary page does not stop the phone dimming.

- **The manifest and the Apple meta tags**, in `index.html`. `start_url` and
  `scope` are both `/`, the only document the server serves.
- **`viewport-fit=cover` plus `env(safe-area-inset-*)`** in `styles.css`, so the
  tab bar does not sit under the notch or the home indicator. The insets are
  `0px` in an ordinary tab.
- **`hooks/useKeepAwake.ts`**, called by the Control route and released with it.
  Prefers `navigator.wakeLock`, falls back to a muted looping video inlined as a
  data URI. Every failure path is a silent no-op.

**There is no service worker, and one would not work.** Service workers register
only in a secure context, and every address the robot answers on is plain HTTP.
Let's Encrypt does not rescue this: `.hexa` is not a delegated TLD, DNS-01 wants
internet the hotspot does not have, and the Pi has no RTC, so a stale clock fails
validation with no way to click past it. An expired certificate means a UI that
will not load at all.

Two existing rules point the same way. `Cache-Control: no-store` keeps a phone
from running last week's UI against today's protocol, and a caching worker is the
opposite of that. Unknown paths 302 to `/` so the OS probes open the controller,
and a worker with a navigation fallback would answer them.

So: **iOS gets the whole thing** (Add to Home Screen needs no worker and no
HTTPS), **Android gets fullscreen and no-sleep but a shortcut rather than an
installed app**, since Chrome's install criteria require HTTPS plus a worker. A
trusted local CA and an HTTPS listener would close that half later. One wart
meanwhile: launching the installed iOS app while the robot is unreachable shows
Safari's error page inside the standalone shell, with no address bar to retry
from — close it and reopen.

## Running

`./hexa sim up` brings up sim + webteleop + teleop; open
`http://<container-ip>:8080`. In production `bringup.launch.py` includes
webteleop alongside the gamepad teleop.

Tests: `./hexa sim python3 -m pytest src/hexa_webteleop/test -q`. The webapp
itself is covered only by the bundle checks — verify UI changes against the sim.
