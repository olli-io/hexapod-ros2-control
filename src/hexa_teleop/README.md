# hexa_teleop

Gamepad teleop. Publishes the high-level command topics the rest of the
stack consumes: `/cmd_vel`, `/body/pose`, `/cmd_gait`, `/animation/mode`,
`/gait/initialize`. Deliberately thin — the single, swappable producer of
high-level commands; an autonomy node would publish the same topics and
replace it transparently.

- **`joy_publisher.py`** — reads `/dev/input/jsN` directly, publishes
  `sensor_msgs/Joy` on `/joy`. Drop-in for upstream `joy_node` with
  reliable hot-plug in the sim container (see below).
- **`teleop_joy.py`** — `/joy` → command topics. Mode-switched stick
  semantics; mapping fully configurable via YAML.
- **`joy_mapping.py`** — pure mapping library (`Joy` → commands), no
  `rclpy`, unit-testable.

Use any X-input controller (fe. 8BitDo etc. in X-input mode). The
`/dev/input` bind mount propagates hot-plugged devices into the running
container.

## Hot-plug (`joy_publisher.py`)

Replaces `joy_node` because udev events don't propagate reliably into the
container. Unplug/replug at any time: the node closes the dead fd, polls
`/dev/input/` every `scan_period_s` (1 s), and re-opens — no ROS restart.
While the device is absent it publishes empty `Joy` at `autorepeat_rate`
(mapped to the safe all-zero state). Auto-discovers the lowest-numbered
`jsN` (Linux renumbers on replug); override with
`device_path:=/dev/input/jsN` when multiple controllers are attached.

## Configuration

Controller identity and per-mode bindings live in
[`config/teleop_joy.yaml`](config/teleop_joy.yaml) (documented inline;
override via `joy_config_file:=...`). The node starts in **gait** mode. The
loader validates every binding at startup — unknown names/keys,
button-vs-axis class mismatches, and cross-section conflicts all raise.

Posture-mode scalar limits are not a teleop knob: they are shared with the
web teleop and the firmware, so `hexa_description/config/tuning.yaml` owns
them (`teleop_node` block), alongside the `posture_node` envelope each must
stay inside. Velocity caps and the animation list come from the same file.

## Quadruped mode (**select**, from the belly)

**select** is the second half of the init button. From the folded state
**start** stands the robot up on six legs and **select** stands it up on four:
the middle pair stays folded where it powered up and the four corner legs creep
one at a time, with the body carrying itself into the next support triangle
before each foot leaves. Off the belly both buttons still mean the same thing —
fold. What has changed is that folding is no longer the *only* way between the
two leg sets: see **leg-set change** below.

- Both stands climb the same ladder (folded → initialized → standing); the
  quadruped one just leaves out the middle pair's rungs. About two seconds
  either way, during which a gait switch is refused.
- **select** is bound only in the `gait` section, so it does nothing in posture
  or animation mode, where it keeps its base `record` binding. Gait is the mode
  the teleop boots into, so the cold start is covered.
- The D-pad gait cycler walks the QUAD preset's rotation while quadruped, not
  the six-leg one. Every preset's rotation is disjoint from every other by
  load-time validation, so a press can never ask for a leg set the engine would
  refuse — and the six-leg slot the operator was on is kept in its own index,
  waiting for them to come back to it.
- Animation mode is unavailable while quadruped — every animation is written for
  six legs. Gait and posture stay available.
- Posture mode poses the body on four feet exactly as it does on six; what it
  will not do there is **record**. `select` is inert in posture mode while
  quadruped, because a recorded pose bleeds through into gait mode, where it
  would spend the same x-y envelope the support shift needs to carry the body
  into the next support triangle — and that margin is millimetres. Body height
  is unaffected either way: it rides its own integrator, not the record.

The mode rides `/cmd_gait` as one of the four-corner gait names, which live in
their own preset's rotation and never in the six-leg one. From the belly the
engine reads the leg set off the strategy applied when `/gait/initialize`
arrives, so the gait publish leads the init publish by design.

### Leg-set change (from a stand, not a button)

A `/cmd_gait` naming a gait of the *other* leg set, sent while the engine is at
`stand`, changes the leg set in place — no fold. The corners are reseated onto
the other footprint with all six feet planted, and the middle pair is folded up
or unfolded down on the other side of that, in whichever order keeps the reseat
on six legs. It is refused from every other state, so a walking robot ignores it
rather than stopping for it. `docs/leg-phases.md` has the ordering rationale.

Nothing on the pad asks for one: **start** and **select** keep the meanings
above exactly. The request comes from the web teleop's Mode view. This node
still has to *follow* it, which is why it subscribes `/cmd_gait` — see below.

### Following `/cmd_gait`

Both teleops write `/cmd_gait`, so this node also reads it, own publishes
included via loopback. One callback then owns all the bookkeeping a gait switch
implies, whoever initiated it: the stick velocity caps, the cycler slot, the
active **preset**, the leg-set flag the cycler picks its rotation off, and the
posture revert a leg-set change needs (the engine holds the middle pair until
the body pose is back at neutral, and the revert is what puts it there). Without
this a switch made from the web app left the D-pad rotating a list the robot was
not standing on. The shared logic is `presets.py` — pure, rclpy-free, and
imported by `hexa_webteleop` too.

## Presets

A **preset** is a named bundle the operator picks as one thing: an id, a label,
a gait rotation, and the gait it enters on. `NORMAL` (six legs) and `QUAD` (four
corners) ship. They are declared under `presets:` in
[`config/teleop_joy.yaml`](config/teleop_joy.yaml); a third is a config edit,
because the active preset's rotation is *projected* onto the one `gait_cycle`
`map_joy` reads, so the mapping's state machine never learns how many exist.

A preset's **leg set is derived** from `hexa_common.gait_catalog`, never
declared — that catalog already owns which legs a gait walks. Load-time
validation keeps the rotations pairwise disjoint and one leg set per rotation,
which is what lets a bare gait name on `/cmd_gait` say which preset is in force
without a second field on the wire.

The two footfall orders, both duty factor 3/4 with one foot airborne at a time:

- **`quad_walk`** — lateral sequence, `l_rear → l_front → r_rear → r_front`.
  Each hind is followed by the fore on its own side, so the body works up one
  side and then the other.
- **`quad_canter`** — perimeter sequence, `r_front → l_front → l_rear → r_rear`.
  Round the chassis rather than up one side, so the two fores lift back to back
  and the handovers carry the body across it.

## Drive sticks (gait / animation mode)

Both sticks can drive; neither is a strict subset of the other.

- **right stick** — full 2D translation: `drive_x` (forward/back) on Y,
  `drive_y` (strafe) on X.
- **left stick** — arcade drive: `drive_yaw` (turn) on X, `drive_x_aux` on
  Y. The two forward sources are summed and clipped, so either stick alone
  reaches full speed, holding both does not double it, and opposing them
  cancels.

The summed triple is then fitted to the reachable velocity envelope
(`fit_drive_to_envelope`). Deflection maps linearly onto `0 → boundary`
along whatever direction the sticks point, so:

- **no dead travel** — full deflection lands exactly on the boundary in
  every direction, including the corners of the stick gate. A full
  translation diagonal comes out at the linear cap's *magnitude*, not
  1.41× it; full forward plus full yaw comes out at roughly half of each.
- **the commanded direction is exact** — one scale factor for all three
  axes, so the robot goes where the sticks pointed, only slower. This is
  what publishing a pre-fitted `/cmd_vel` buys: the engine's own clamp
  (`hexa_common.scale_to_envelope`) finds nothing left to cut, and its
  `yaw_bias` split — which does rotate the command — now only shapes
  autonomous `/cmd_vel` sources.
- **the trade** — mild cross-axis coupling below saturation: adding yaw
  trims forward speed a little at any deflection, not just at the
  extremes. For a walker that reads as physical.

The single-axis case is untouched by all of this: each axis alone still
commands exactly its cap. The deadband is scaled-radial — what clears it
is stretched back over the full range, so output leaves centre
continuously rather than stepping to a tenth of the cap, and full
deflection still means the cap.

## Values owned elsewhere (SSoT in `hexa_description/config/tuning.yaml`)

- **Velocity caps** — via `hexa_common.load_velocity_caps`. Both are
  per-gait and both are re-scaled the moment a gait switch is accepted.
  Linear is isotropic (`stride_length × (1−β) / (min_swing_time × β)`);
  angular is that same cap divided by the outermost standing foot's planar
  radius, the lever arm a yaw rate acts through. There is no turn-rate knob
  — to change it, change `stride_length`, `min_swing_time`, the gait's duty
  factor, or the stance width (`default_standing_pose.<group>.tip_reach`).
  Because the
  angular cap comes from the stance, the loader reads `geometry.yaml`
  alongside `tuning.yaml`. Edit those, not teleop config.
- **Animation cycler list** — via
  `hexa_posture.load_animation_mode_animations`
  (`animation_mode_animations`). Adding an entry exposes it on the
  joystick with no teleop edit.
