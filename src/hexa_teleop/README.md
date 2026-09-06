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
  `rclpy`, unit-testable. Two halves: `resolve_functions` reads a `Joy`
  snapshot as the *functions* a section's bindings say are held, and
  `map_functions` is the state machine over those. `map_joy` is the pair
  composed, and is what a device reporting indices calls. A device whose
  operator names the function outright — the webapp — builds the seam's
  `FunctionInput` itself and skips the index half entirely.

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
two leg sets: see **preset change** below.

- Both stands climb the same ladder (folded → initialized → standing); the
  quadruped one just leaves out the middle pair's rungs. About two seconds
  either way, during which a gait switch is refused.
- **select** is bound only in the `gait` section, so it does nothing in posture
  or animation mode, where it keeps its base `record` binding. Gait is the mode
  the teleop boots into, so the cold start is covered.
- The D-pad gait cycler walks the QUAD preset's rotation while quadruped, not
  the six-leg one. Every gait in a preset's rotation walks that preset's legs by
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

The mode rides `/cmd_preset` as the `quad` id. From the belly the init edge
carries the leg set itself — **start** asks for the last six-leg preset applied,
**select** for the four-corner one — so the stand needs no gait publish to tell
it which ladder to climb.

### Preset change (from a stand, not a button)

A `/cmd_preset`, sent while the engine is at `stand`, changes the preset in place
— no fold. The walking feet are reseated onto the new preset's footprint with all
six planted. Where the two presets differ in **leg set** the middle pair is also
folded up or unfolded down on the other side of that reseat, in whichever order
keeps it on six legs; where they do not, the reseat is the whole change. It is
refused from every other state, so a walking robot ignores it rather than
stopping for it. `docs/leg-phases.md` has the ordering rationale.

Nothing on the pad asks for one: **start** and **select** keep the meanings above
exactly, and they choose a leg set rather than a preset. The request comes from
the web teleop's Mode view. This node still has to *follow* it, which is why it
subscribes both `/cmd_preset` and `/gait/preset` — see below.

### Following the wire

Both teleops write `/cmd_gait` and `/cmd_preset`, so this node also reads them,
own publishes included via loopback. Three callbacks split the bookkeeping by
what each event actually decides:

- **`/cmd_gait`** — the stick velocity caps and the cycler slot. Not the leg
  set: the preset owns that now.
- **`/cmd_preset`**, the *request* — starts the posture revert. It has to be the
  request and not the report, because the engine will not begin the change until
  the body pose is back at neutral and this decay is the only thing that puts it
  there.
- **`/gait/preset`**, the engine's *applied* preset — the caps again (every one
  is derived from the preset's stride, swing time and stance), the two rotations
  the cycler picks from, and the leg-set flag `map_joy` reads.

Without this a switch made from the web app left the D-pad rotating a list the
robot was not standing on. The shared logic is `presets.py` — pure, rclpy-free,
and imported by `hexa_webteleop` too.

## Presets

A **preset** is a named bundle the operator picks as one thing. It has two
halves, in two files, because they are owned by different layers:

- the **physical** half — the legs it stands on, where those feet sit, and the
  `stride_length`, `stride_length_radial`, `min_swing_time`, `max_swing_time` and
  `step_height` the walk lays down on it — is in `hexa_description`'s
  `tuning.yaml` under `gait_node.presets`, which the engine loads directly;
- the **operator** half — an id, a label, a gait rotation and the gait it enters
  on — is under `presets:` in
  [`config/teleop_joy.yaml`](config/teleop_joy.yaml), keyed by the same ids.

`NORMAL` (six legs, everyday stance), `FAST` (low, long-striding), `OFFROAD`
(tall, short steps, high clearance) and `QUAD` (four corners) ship. A fifth is an
edit to those two files and nothing else, because the active preset's rotation is
*projected* onto the one `gait_cycle` `map_joy` reads, so the mapping's state
machine never learns how many exist.

A preset's **leg set is read from `tuning.yaml`**, never declared here: whether
the middle pair stands is a physical fact about the robot, not a property of the
gaits a preset happens to offer. Load-time validation then requires every gait in
a rotation to walk that preset's legs, so a prev/next press can never ask a
standing robot for a leg set the engine would refuse.

Rotations across presets may **overlap** — `NORMAL`, `FAST` and `OFFROAD` all
stand on six legs and all offer six-leg gaits. That is why the preset needs a
channel of its own: a bare gait name cannot say which of the three is in force,
and neither can `/gait/leg_set`. `/cmd_preset` carries the request and
`/gait/preset` the engine's answer.

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

### Body yaw and height while walking

Two posture offsets are **held functions live in every mode**, not just in
posture mode: `height_up` / `height_down` on their integrator, and
`yaw_left` / `yaw_right` on the eased `yaw_current`. Both are added to the
pose the gait/animation branch publishes, so the body can be held turned
or raised while the robot walks, and both ease or integrate the same way
whichever mode asked for them.

- **the binding table still decides where they are reachable** — `l1`/`r1`
  are bound in the `gait` and `posture` sections and left empty in
  `animation`, so a gamepad has yaw in the first two and none in the
  third. The webapp offers the same two, by the same reasoning.
- **wiggle is live in gait and posture** — `l2`/`r2` translate the body
  about a pivot ahead of centre, and both the wiggle and the yaw it pushes
  ride along on the walk exactly as height and yaw do; the gait branch adds
  the translation to the recorded x-y baseline, which is the only x-y
  offset it carries, since the sticks are driving there. **Animation mode
  is the exception**: the animation owns the body, so the mapping ignores
  the function there and `l2`/`r2` are unbound in that section besides. An
  offset already held still bleeds through and eases off.
- **record is unchanged** — it folds `yaw_current` into `recorded_yaw` in
  posture mode alone, and an init edge over a held yaw still arms the
  revert rather than standing.

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
