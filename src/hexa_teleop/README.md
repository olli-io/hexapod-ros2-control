# hexa_teleop

Human-input nodes that publish to `/cmd_vel` (body velocity, consumed
by `hexa_control`) and `/body/pose` (raw user body pose offset,
consumed by `hexa_posture`).

- `joy_publisher.py` — reads `/dev/input/jsN` directly and publishes
  `sensor_msgs/Joy` on `/joy`. Drop-in for upstream `joy_node` with
  reliable hot-plug recovery in the dev container.
- `teleop_joy.py` — consumes `/joy` and publishes `/cmd_vel`,
  `/body/pose`, `/cmd_gait`, `/animation/mode`, and
  `/gait/initialize`. Mode-switched stick semantics; axis / button
  mapping fully configurable via YAML.
- `joy_mapping.py` — pure mapping library (`Joy` snapshot →
  high-level commands). No `rclpy`; unit-testable standalone.

Intentionally thin: this package exists so the rest of the stack has a
single, swappable producer of high-level commands. An autonomy node would
publish the same topics and replace teleop transparently.

## Run it

Use any X-input controller (Xbox, 8BitDo, etc.). If the controller
has a mode switch, put it in **X-input mode** (on 8BitDo: hold
`Start + X` for a few seconds at power-on), then plug in via USB-C.
Plug timing relative to the dev
container does not matter — the `/dev/input` bind mount in
`docker-compose.yaml` propagates new device nodes into the running
container, and `joy_publisher.py` polls for them.

- Launch the sim (separate terminal): `ros2 launch hexa_bringup sim.launch.py`
- Launch teleop: `ros2 launch hexa_teleop teleop.launch.py`

## Joy publisher / hot-plug

`joy_publisher.py` reads `/dev/input/jsN` directly and publishes
`sensor_msgs/Joy` on `/joy`. It replaces upstream `joy_node` so the
controller can be unplugged and plugged back in at any point — the
node closes the dead fd, polls `/dev/input/` once per `scan_period_s`
(default 1 s), and re-opens as soon as the device reappears. No ROS
process restart. The controller may also be absent at launch — the
node logs and waits for it. While the device is gone the node still
publishes an empty `Joy` at `autorepeat_rate`, which `joy_mapping`
resolves to the safe all-zero state.

Device selection is auto-discovery: the lowest-numbered
`/dev/input/jsN` present is opened. Linux renumbers `jsN` on replug
(a controller that was `js0` can come back as `js1`), so a pinned
number would be brittle in exactly the use case this node was written
for. Set `device_path:=/dev/input/jsN` to override when multiple
controllers are attached.

Upstream `joy_node` nominally supports hot-plug via SDL2 + udev, but
udev events don't propagate reliably into the dev container — that's
the gap this node fills. The Joy layout matches what `joy_node` would
have produced for the same device, so `teleop_joy` and downstream
consumers are unchanged.

## Keybind-driven configuration

`config/teleop_joy.yaml` is split into four sections:

- **base** — controller hardware identity. `buttons` and `axes` map
  named physical keys (`a`, `l1`, `dpad_x`, `left_stick_x`, …) to the
  Linux joystick API indices the driver reports. `axis_signs` flip a
  driver that reports stick-right / dpad-right as `-1` back to the
  REP-103 convention. `bindings` here assigns the mode-agnostic
  functions (mode toggles, init, record). Edit this block — and only
  this block — to support a different controller.
- **gait**, **posture**, **animation** — per-mode `bindings`. Each
  section enumerates the shoulder buttons, D-pad directions
  (`dpad_up`, `dpad_down`, `dpad_left`, `dpad_right` are exposed as
  virtual button names derived from the bound D-pad axes), and stick
  axes, and assigns each a function name (or `""` for unbound).
  `posture` also carries the mode's scalar limits (translation /
  tilt / yaw maxima, low-pass time constants, height range).

The loader validates every binding at startup: unknown function
names, unknown keys, button-class functions bound to stick axes (or
vice versa), and cross-section conflicts (the same function bound to
different keys in different sections) all raise. Identical duplicates
across sections — e.g. `dpad_left: gait_prev` in both `gait.bindings`
and `posture.bindings` — are allowed.

Function namespace (assignable in YAML):

- **base** — `gait_mode`, `posture_mode`, `animation_mode`, `init`, `record`.
- **button-class** — `yaw_left`, `yaw_right`, `wiggle_left`, `wiggle_right`, `height_up`, `height_down`, `gait_prev`, `gait_next`, `animation_prev`, `animation_next`. Bindable to any button or D-pad direction. `wiggle_left` / `wiggle_right` are polymorphic: bound to a trigger axis (`l2` / `r2`), they're thresholded against `base.trigger_threshold`; bound to a face button, they read the button directly.
- **axis-class** — `drive_x`, `drive_y`, `drive_yaw` (gait `/cmd_vel`), `pose_x`, `pose_y` (posture translation), `tilt_roll`, `tilt_pitch` (posture body tilt). Bindable only to stick axes.

## Default behavior

The shipped YAML:

- **Right stick** — `pose_x` / `pose_y` in posture, `drive_x` / `drive_y` in gait and animation. Stick forward → body `+x`; stick left → body `+y`.
- **Left stick (posture)** — `tilt_roll` / `tilt_pitch`. Stick forward → pitch forward (front dips); stick left → roll left (left side dips).
- **Left stick X (gait, animation)** — `drive_yaw`.
- **L1 / R1 (posture)** — `yaw_left` / `yaw_right` body yaw, eased through `posture.yaw_tau_s` (default ~0.1 s), saturates at `posture.yaw_max_deg` while held. Both buttons cancel to zero. Inactive in other modes.
- **L2 / R2 (posture)** — `wiggle_left` / `wiggle_right`. Same yaw target as L1 / R1 (shared) plus a body translation that holds a point `posture.wiggle_pivot_forward_m` ahead of body centre stationary. Triggers thresholded at `base.trigger_threshold`. Inactive in other modes.
- **D-pad up / down (posture)** — `height_up` / `height_down` integrate a persistent body-height offset (`pose.z`) while held, clamped to `posture.height.[min,max]_m`. Height **persists** across release and a mode toggle into gait, so the robot keeps walking at the lifted/lowered chassis height.
- **D-pad up / down (animation)** — `animation_next` / `animation_prev` cycle through the animation list loaded at startup from `hexa_posture/config/posture.yaml` (`animation_mode_animations`; `vertical_body_roll → horizontal_body_roll → body_roll_3d` by default, wrapping). Each rising edge publishes the new selection on `/animation/mode`. Entering ANIMATION mode snaps to index 0 and publishes that name so the body is visibly animated immediately.
- **D-pad left / right (gait, posture)** — `gait_prev` / `gait_next` cycle the active gait through `gait_cycle` (`wave → ripple → tetrapod → surf → tripod` by default). The switch is published to `/cmd_gait` when the gait engine reports `stand`, `gait`, `pausing`, `paused`, or `reseating`. A switch while standing applies immediately; a switch while walking makes the engine pause, reseat with a short dwell, commit the new gait, and resume walking if the stick is still held — further presses mid-sequence keep updating the pending gait. During `engaging` / `resuming` the gait is locked: the press advances the local cursor but the switch is dropped, not queued.
- **Select (posture)** — `record`. Snapshots the current effective body pose on rising edge into a persistent baseline. The robot holds that pose when the joystick is released; subsequent stick input adds on top and clamps per-axis, so re-pushing a stick that's already at its limit has no further effect. The baseline bleeds through into gait mode.
- **A** — `gait_mode`. **Y** — `posture_mode`. **B** — `animation_mode` (toggles GAIT ↔ ANIMATION).
- **Start** — `init`. Single-shot trigger for the gait engine's fold / initialize cycle, with a two-press safety: if any posture state is non-default, the first press arms a **smooth revert** back to default (time constant `posture.revert_tau_s`, default 0.25 s) and **does not** publish `/gait/initialize`; the next press once the revert has settled fires init as usual. Pressing Select mid-revert cancels it. Held presses don't repeat — release and press again.

The node starts in **gait** mode. Override the config at launch with
`joy_config_file:=/path/to/file.yaml`.

Gait-mode stick scaling — the linear and angular velocity caps applied
to a full-stick deflection — is **not** owned here. It is loaded at
startup from `hexa_gait/config/gait.yaml` via
`hexa_gait.load_velocity_caps`:

- Linear is **isotropic and per-gait**: full-stick forward and full-stick
  sideways saturate at `stride_length × (1 − β) / (min_swing_time × β)`
  for the *active* gait's β. Tripod has the highest cap, wave the
  lowest. The stick re-scales the moment a D-pad cycle accepts a new
  gait so 100% deflection always commands the gait's true top speed.
- Angular cap is the explicit `angular_z_max` knob in `gait.yaml`.

To change those caps, edit `gait.yaml`; no teleop config edit needed.

Likewise the ANIMATION-mode cycler list is loaded from
`hexa_posture/config/posture.yaml` via
`hexa_posture.load_animation_mode_animations`, so adding an entry to
`animation_mode_animations` exposes it on the joystick without any
teleop-side edit.
