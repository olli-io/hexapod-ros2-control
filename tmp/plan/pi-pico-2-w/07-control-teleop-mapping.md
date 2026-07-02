# Part 07 — Control shaping & teleop mapping (drive it by hand)

**Goal:** close the loop from the Bluetooth gamepad to walking — port the input
mapping and the velocity-shaping stage so a human drives the robot.

**Depends on:** 02 (BT input), 04 (config), 06 (engine + caps). **Blocks:** 09.

## Scope

- `src/joy_mapping.{hpp,cpp}` — fork of `hexa_teleop/joy_mapping.py` (801 lines, pure):
  - `JoyState`: mode, rising-edge trackers, eased yaw/wiggle (`1−exp(−dt/tau)`), persistent `height_current`, recorded baseline, gait/animation indices.
  - `map_joy(axes,buttons,cfg,state,dt) → JoyOutput{linear_x/y, angular_z, pose_x/y/z, pose_roll/pitch/yaw, mode_changed, init_request, gait_select?, animation_name?}`.
  - Helpers: `apply_deadband`, `axis_value_for` (sign+deadband), `button_pressed_for` (physical / virtual-dpad ±0.5 / analog-trigger `value<threshold`), mode FSM (gait/posture/animation, posture wins ties), two-press init/revert with exponential decay, height integration + clamp, gait/animation cyclers, POSTURE-mode yaw+wiggle (`wx=wiggle·px·(1−cos yaw)`, `wy=−wiggle·px·sin yaw`), record button folding live offsets into baseline.
  - **Drop** `teleop_arbitration.py` / `/teleop/owner` — always publish.
- `src/control.{hpp,cpp}` — fork `body_velocity_limiter.py` + `control_node.py` logic:
  - `BodyVelocityLimiter`: constant-max-accel slew — linear pair as 2D vector (`dist=hypot`, `max_step=accel·dt`, snap if within), scalar angular slew, snap-to-zero tolerances, per-gait `accel=cap/ramp_time`.
  - Per tick: `scale_to_envelope(vx,vy,wz, leg_mounts, linear_max(gait), angular_max, yaw_bias(gait))` (from part 06 `limits`) → `limiter.step` → feed `engine.update`. Reset limiter when engine leaves `{engaging,gait}`; recompute `accel_linear` on gait change.
- `main.cpp` tick becomes: `bt_teleop.read` → `map_joy` → control → engine → compose/IK/pulse. Route `gait_select`/`init_request`/`animation_name` into engine calls (`set_strategy`, `start_initialize`), gated by engine state (`{stand,gait,pausing,paused,reseating}` for gait switches, as `teleop_joy.py` does).

## Done when / verification

- Live drive on a stand, then on the ground: left/right stick produce forward/strafe/yaw with correct sign and scaling; releasing sticks ramps smoothly to a stop (slew, not instant); the robot pauses/settles at zero.
- `scale_to_envelope` proven: combined max linear + max yaw stays within the per-leg foot-speed envelope (no leg saturates/jitters).
- Gait cycle button switches tripod→…→ripple only in allowed states; the init button triggers a clean stand.
- Host: compare `map_joy` output against the Python `map_joy` for a recorded axes/buttons trace (same `JoyOutput`).
