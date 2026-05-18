# hexa_gait

Gait engine. Given a desired body velocity and gait parameters, emits a
stream of per-leg foot targets in the **nominal** body frame — i.e. the
body frame as if the user hadn't applied any pose offset. Body pose is
composed in downstream by `hexa_kinematics`, not here. This package is
intentionally unaware of `/body/pose_target`; gait strategies stay pure
functions of `(phase, params) → foot_target`.

Designed around a strategy pattern so additional gaits drop in cleanly:
- `gaits/tripod.py` — alternating 3+3 (fast, default).
- `gaits/wave.py`   — one leg at a time (slow, max stability).
- `gaits/ripple.py` — overlapping wave (medium).

v1 ships **tripod only**; `wave` and `ripple` are next drop-ins on the
same `Strategy` protocol. Multi-gait selection will land in
`hexa_control` (e.g. wave below a speed threshold, tripod above);
`hexa_gait` itself stays agnostic about gait choice.

Inputs:
- `/gait/params` (`hexa_interfaces/GaitParams`) — gait selection and the
  target body velocity (`linear_x`, `linear_y`, `angular_z`).

Outputs:
- `/legs/targets` (`hexa_interfaces/LegState[6]`) — foot pose + phase.

The engine is stateful (it owns the phase clock) but the gait strategies
themselves are pure functions of `(phase, params) → foot_target`.

All walk-cycle knobs (`stride_length`, `min_cycle_time`,
`max_cycle_time`, `duty_factor`, `step_height`, ...) live in
`config/gait.yaml` — they are not on the wire.

See [`docs/leg-phases.md`](../../docs/leg-phases.md) for the shared
vocabulary (stance, swing, AEP, PEP, duty factor, support polygon) used
below and throughout the codebase.

## Velocity → cycle_time

`cycle_time` is **not** a configured constant. The engine derives it
each GAIT tick from the commanded velocity and a fixed
`stride_length`:

- For each leg, compute `v_leg = v_body + ω_z × r_leg`.
- `max_leg_v = max( |v_leg| for all 6 legs )`.
- `cycle_time = clip( stride_length / (max_leg_v × duty_factor),
  min_cycle_time, max_cycle_time )`.
- Per-leg `stride_vector = v_leg × cycle_time × duty_factor`, with the
  magnitude further clamped to `stride_length` so saturated commands
  never push past the joint-limit-safe footprint.

Faster commands therefore *cycle faster at constant stride* instead of
taking *bigger steps at constant cycle*. The implied per-leg velocity
ceiling is `stride_length / (min_cycle_time × duty_factor)`; beyond
that the gait saturates (`cycle_time` pinned at `min_cycle_time`, stride
clamped to `stride_length`). Below the velocity that implies
`max_cycle_time`, the cycle stops dragging out — stride shrinks linearly
instead.

## Velocity caps for upstream nodes

`gait.yaml` is the **single source of truth** for the velocity caps that
upstream nodes (`hexa_teleop` for stick scaling, `hexa_control` for
`/cmd_vel` clamping) apply at their respective boundaries. Both load
the caps through `hexa_gait.load_velocity_caps(gait_yaml_path)` at
startup; there are no duplicate knobs in the teleop or control YAML.

- `linear_max` is **derived** isotropically as
  `stride_length / (min_cycle_time × duty_factor)` — exactly the per-leg
  velocity ceiling above. Anything above this would be silently clipped
  by the engine, so we make the ceiling explicit at the input boundary.
- `angular_z_max` is an **explicit** knob in `gait.yaml`. Kept explicit
  (not geometry-derived) because angular feel is harder to predict from
  leg radii — the gait's geometric ceiling
  (`linear_max / r_outer`) is typically well above what feels
  comfortable, so this knob trades reach for tunable feel.

See `hexa_gait/hexa_gait/limits.py` for the helper API.

## Stopping: idle and standing reset

When GaitParams arrives with zero velocity (sent by `hexa_control` when
`cmd_vel` goes idle), the engine does not simply freeze the phase clock
— that would leave any leg in mid-swing dangling in the air. Instead it
runs a four-state reset sequence that brings the robot from an
arbitrary mid-cycle pose to a clean standing pose:

1. **FORCE_TOUCHDOWN** — every leg airborne at stop time swings to its
   nominal stance position via the standard swing arc, rising through
   `swing_clearance` before descending, over `recenter_swing_time`.
   The swing arc runs with both endpoint velocities pinned to zero so
   the Bezier decelerates fully at touchdown — landing at the
   steady-state stance velocity would slam the foot into the floor
   and rock the chassis. All airborne legs move in parallel. The legs
   that were already on the ground at stop time hold their stop-time
   positions exactly — they do not budge. The forced lift matters
   when a leg stopped just above the ground: a straight-line move
   there would skim the floor instead of clearing it. Skipped if no
   leg was airborne when `cmd_vel` went idle.
2. **SETTLE** — hold every foot still for `touchdown_settle_time`
   seconds. Lets residual chassis sway from the touchdown impact damp
   out before the next sweep adds more motion. Skipped when
   `touchdown_settle_time` is zero or FORCE_TOUCHDOWN was skipped
   (no impact to settle from).
3. **RECENTER** — sweep the originally-grounded legs to nominal one at
   a time, in canonical leg order, using the normal swing-arc
   trajectory (lift → translate → place). By this point every foot is
   on the ground, so the support polygon is always 5/1 stance/swing
   and the body stays stable.
4. **STAND** — hold the nominal stance with phase frozen. The engine
   stays here until a non-zero velocity arrives.

The key stability invariant: a grounded foot is never repositioned
while any other foot is airborne. FORCE_TOUCHDOWN holds the stance legs
perfectly still while the swing legs settle, and RECENTER only starts
once all six feet are down.

### Architectural note

The reset sequence is *stateful per leg* (each leg remembers where it
started and where it's going), which does not fit the
`(phase, params) → foot_target` pure-function contract of the gait
strategies. It is therefore implemented as a separate **transition
controller** alongside the strategies; the engine routes between the
active strategy and the transition controller based on commanded
velocity. RECENTER reuses the strategy's swing-arc helper so trajectory
code is not duplicated.

### Resume

If a non-zero velocity arrives mid-sequence, the engine completes the
reset first (FORCE_TOUCHDOWN → SETTLE → RECENTER → STAND) and only then starts
the new gait from the nominal stance. The reset is short (≤ one
wave-style cycle ≈ 6 leg moves), so the latency cost is acceptable, and
aborting mid-sequence would risk legs left in unsafe poses.
