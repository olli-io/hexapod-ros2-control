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

## Body-velocity command limiter

`BodyVelocityLimiter` sits between `scale_to_envelope` and the
`GaitParams` publish. It runs a vectorial rate-cap slew on
`(v_x, v_y, ω_z)`: each tick the planar linear vector advances toward
its target by at most `accel_linear · dt`, and `ω_z` advances toward
its target by at most `accel_angular · dt`. When the remaining
distance falls within one max-step the limiter snaps to the target,
giving finite-time convergence (including to exact zero).

Treating `(v_x, v_y)` as a single vector keeps the slew isotropic in
the body plane and means a diagonal direction reversal traverses the
magnitude-zero point at the same single bounded rate as an
axis-aligned reversal — no per-axis coupling artefacts.

The limiter exists to absorb the per-tick step that
`scale_to_envelope` produces on yaw release (suppressed `v_x` snaps
to full demand), on `/cmd_gait` cap changes, and on Nav2 stops, **and**
to bound the worst-case body-frame acceleration on every transition
including operator stick flips. Placement after `scale_to_envelope` is
required — putting it before would let the envelope re-cut the
smoothed signal and the jump would reappear at the publish boundary.

A constant-acceleration slew (not a first-order time constant) so the
worst-case derivative is symmetric and finite-time, and the gait
engine's `cmd_zero_tol` triggers cleanly on release without any
special-case snap. The acceleration cap itself is not the tuning
knob: `control.yaml` exposes `vmax_ramp_time_linear` /
`vmax_ramp_time_angular` (seconds to ramp from rest to the active
gait's velocity ceiling) and the node derives `accel_linear =
linear_max(gait) / vmax_ramp_time_linear` per gait. Without this the
gait-independent acceleration cap reverses tripod over ~2.7 s but
ripple over ~0.5 s, which reads as "instant" on slower gaits and is the
worst case for stance-foot slip on the mid-duty crawl gait. On every
`/cmd_gait` switch the limiter's `accel_linear` is updated so the
ramp time stays constant across gaits. The limiter is not a hard
safety limit: during ramp-down the published velocity may briefly
exceed the static envelope window. That is acceptable because
`linear_max` is a swing-time constraint, not a joint-velocity ceiling
— exceeding it transiently stretches `cycle_time` via the engine's
`_derive_cycle_time`.

`snap_tol_linear` / `snap_tol_angular` are sub-tolerance dribble
erasers (≤ engine `cmd_zero_tol`), not the release-tail mechanism.

Out of scope: deadband (handled in `hexa_teleop`); speed-based gait
selection (operator-driven via D-pad or an autonomy node publishing
`/cmd_gait`). Body pose lives in `hexa_posture`.
