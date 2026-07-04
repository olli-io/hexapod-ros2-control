# hexa_control

Velocity shaping between teleop / autonomy and the gait engine.

- Subscribes to `/cmd_vel`, clamps against the caps in
  `hexa_description/config/tuning.yaml`, and republishes as `GaitParams` on
  `/gait/params` at 200 Hz.
- Subscribes to `/cmd_gait` (`transient_local`) for the active gait
  name; unknown names are warned and dropped.
- Subscribes to `/gait/state` and resets the body-velocity filter to
  zero on every edge that leaves the walking set
  (`{engaging, gait}`), so a fresh `STAND → ENGAGING` starts clean.

## Body-velocity command limiter

`BodyVelocityLimiter` sits between `scale_to_envelope` and the `GaitParams`
publish, running a constant-acceleration slew on `(v_x, v_y, ω_z)`. Each tick the
planar linear vector advances toward its target by at most `accel_linear · dt` and
`ω_z` by at most `accel_angular · dt`; within one max-step it snaps to the target,
giving finite-time convergence (including to exact zero).

**Why a vector, not per-axis** — treating `(v_x, v_y)` as one vector keeps the
slew isotropic, so a diagonal reversal crosses zero at the same bounded rate as an
axis-aligned one, with no per-axis coupling artefacts.

**Why constant-acceleration** — the worst-case derivative stays symmetric and
finite-time, and the engine's `cmd_zero_tol` triggers cleanly on release with no
special-case snap.

**Why after `scale_to_envelope`** — it absorbs the per-tick step the envelope
produces on yaw release (suppressed `v_x` snaps to full demand), `/cmd_gait` cap
changes, and Nav2 stops, and bounds body-frame acceleration on every transition
including stick flips. Placed before, the envelope would re-cut the smoothed signal
and the jump would reappear at the publish boundary.

**Per-gait ramp** — the acceleration cap is derived, not tuned. `tuning.yaml`
(`control_node` block) exposes `vmax_ramp_time_linear` / `vmax_ramp_time_angular`
(seconds from rest to the gait's velocity ceiling); the node derives
`accel_linear = linear_max(gait) / vmax_ramp_time_linear` and re-derives it on
every `/cmd_gait` switch so ramp time stays constant across gaits. A
gait-independent cap would instead reverse tripod over ~2.7 s but ripple over
~0.5 s — "instant" on slow gaits, and worst-case stance-foot slip on mid-duty crawl.

**Not a hard safety limit** — during ramp-down the published velocity may briefly
exceed the static envelope. That is fine: `linear_max` is a swing-time constraint,
not a joint-velocity ceiling, so exceeding it transiently just stretches
`cycle_time` via the engine's `_derive_cycle_time`.

`snap_tol_linear` / `snap_tol_angular` are sub-tolerance dribble erasers
(≤ engine `cmd_zero_tol`), not the release-tail mechanism.

Out of scope: deadband (handled in `hexa_teleop`); speed-based gait
selection (operator-driven via D-pad or an autonomy node publishing
`/cmd_gait`). Body pose lives in `hexa_posture`.
