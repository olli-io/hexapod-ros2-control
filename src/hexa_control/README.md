# hexa_control

Velocity shaping between teleop / autonomy and the gait engine.

- Subscribes to `/cmd_vel`, clamps against the caps in
  `hexa_gait/config/gait.yaml`, and republishes as `GaitParams` on
  `/gait/params` at 50 Hz.
- Subscribes to `/cmd_gait` (`transient_local`) for the active gait
  name; unknown names are warned and dropped.
- Subscribes to `/gait/state` and resets the body-velocity filter to
  zero on every edge that leaves the walking set
  (`{engaging, gait}`), so a fresh `STAND → ENGAGING` starts clean.

## Body-velocity command filter

`BodyVelocityLimiter` sits between `scale_to_envelope` and the
`GaitParams` publish. It runs a first-order low-pass on
`(v_x, v_y, ω_z)` with separate time constants `tau_linear` and
`tau_angular`: each tick,
`v ← v + (1 − exp(−dt/τ)) · (target − v)`. Linear axes share `τ` so
the response is naturally isotropic in the body plane.

The filter exists to absorb the per-tick step that
`scale_to_envelope` produces on yaw release (suppressed `v_x` snaps
to full demand), on `/cmd_gait` cap changes, and on Nav2 stops.
Placement after `scale_to_envelope` is required — putting it before
would let the envelope re-cut the smoothed signal and the jump would
reappear at the publish boundary.

Time constants (not an accel cap) so the filter has no derivative
discontinuity at saturation, self-scales with command size, and
composes on the same axis as the engagement controller's smoothstep.
The filter is not a hard safety limit: during ramp-down the published
velocity may briefly exceed the static envelope window. That is
acceptable because `linear_max` is a swing-time constraint, not a
joint-velocity ceiling — exceeding it transiently stretches
`cycle_time` via the engine's `_derive_cycle_time`.

Out of scope: deadband (handled in `hexa_teleop`); speed-based gait
selection (operator-driven via D-pad or an autonomy node publishing
`/cmd_gait`). Body pose lives in `hexa_posture`.
