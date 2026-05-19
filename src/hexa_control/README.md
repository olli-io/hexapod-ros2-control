# hexa_control

Velocity shaping. Sits between teleop / autonomy and the gait engine.

Three side jobs:

- Subscribe to `/cmd_vel` (`geometry_msgs/Twist`).
- Clamp linear x / y and angular z against the velocity caps loaded at
  startup from `hexa_gait/config/gait.yaml` via
  `hexa_gait.load_velocity_caps`. `control.yaml` owns `default_gait`
  and the body-velocity acceleration caps (`max_linear_accel`,
  `max_angular_accel`).
- Subscribe to `/cmd_gait` (`std_msgs/String`, `transient_local`
  durability) — the active gait selection from teleop / autonomy.
  Validates the name against `hexa_gait.gaits.STRATEGIES`; unknown
  names are warned and dropped.
- Subscribe to `/gait/state` (`std_msgs/String`) — used to reset the
  body-velocity limiter on every transition out of the active walking
  set (`{engaging, gait}`).
- Republish as `GaitParams` on `/gait/params` at 50 Hz with the active
  gait name in `GaitParams.gait_name` and the rate-limited, shaped
  velocity in `linear_*` / `angular_z`.

Walk-cycle knobs (`stride_length`, `min_swing_time`, `max_cycle_time`,
per-gait `duty_factor`, `step_height`) and the velocity caps live in
`hexa_gait/config/gait.yaml` — the single source of truth.
`cycle_time` is derived inside the gait engine each tick from the
commanded velocity — it is not configured here.

## Body-velocity rate limit

`BodyVelocityLimiter` (in `body_velocity_limiter.py`) sits between
`scale_to_envelope` and the `GaitParams` publish. Each 50 Hz tick it
slews the stored `(v_x, v_y, ω_z)` toward the new envelope output by
at most `max_*_accel·dt`:

- **Linear** — `(Δv_x, Δv_y)` is bounded as a *vector* by
  `max_linear_accel · dt`. Magnitude is clamped, direction preserved
  — diagonal stick inputs ramp in the same wall time as axis-aligned
  ones (the hex is radially symmetric in the body plane).
- **Angular** — `|Δω_z|` is bounded by `max_angular_accel · dt`.

This catches every step the upstream chain can produce: joystick
release (where `scale_to_envelope` stops suppressing one axis and the
others jump), gait-cap change on `/cmd_gait` (`linear_max` shifts and
`scale_to_envelope` re-cuts harder), Nav2 stop commands, and any
other `/cmd_vel` source.

Placement is deliberate. Putting the limiter *before*
`scale_to_envelope` would let the envelope re-cut a smoothed signal
and the jump returns at the publish boundary — so the limiter must
see the envelope's output.

**The limiter is not a hard safety limit.** During ramp-down it
intentionally holds the previous velocity for `Δv / max_accel`
seconds. In that window the published `(v_xy, ω_z)` exceeds what the
static `scale_to_envelope` would have permitted. This is acceptable
because `linear_max` is a swing-time constraint (see
`hexa_gait/hexa_gait/limits.py`), not a joint-velocity ceiling —
exceeding it transiently stretches `cycle_time` via the engine's
`_derive_cycle_time`, not joint limits.

**Reset on state transitions.** The limiter subscribes to
`/gait/state` and resets to `(0, 0, 0)` on every edge that leaves the
walking set (`{engaging, gait}`). This guarantees that a fresh
`STAND → ENGAGING` cycle begins with the limiter at zero, so it and
the engagement controller's geometric smoothstep (in
`hexa_gait/hexa_gait/engagement.py`) compose cleanly from a known
state. A `/cmd_gait` switch is **not** a reset trigger — letting the
limiter slew through the new envelope is the design.

Out of scope:

- **Deadband.** `hexa_teleop` zeroes stick noise at its boundary.
- **Speed-based gait selection.** Selection is operator-driven via the
  D-pad cycler; an autonomy node could publish `/cmd_gait` directly
  with the same contract.

Does **not** own body pose — that lives in `hexa_posture`, which runs
in parallel and publishes `/body/pose_target` directly to the IK node.
`hexa_control` may still read `/cmd_vel` and gait state, but the body
pose signal does not flow through here.
