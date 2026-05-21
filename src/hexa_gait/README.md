# hexa_gait

Gait engine. Given a desired body velocity and gait parameters, emits a
stream of per-leg foot targets in the **nominal** body frame — i.e. the
body frame as if the user hadn't applied any pose offset. Body pose is
composed in downstream by `hexa_kinematics`, not here. This package is
intentionally unaware of `/body/pose_target`; gait strategies stay pure
functions of `(phase, params) → foot_target`.

Designed around a strategy pattern so additional gaits drop in cleanly:
- `gaits/tripod.py` — alternating 3+3 (β = 0.5, fast, default).
- `gaits/ripple.py` — metachronal pair, two legs in swing (β = 2/3,
  medium).
- `gaits/wave.py`   — one leg in swing at a time (β = 5/6, max
  stability).

Ripple and wave share the metachronal phase-offset table; they differ
only in duty factor. The strategy registry in `gaits/__init__.py`
exposes them by name (`STRATEGIES["tripod" | "ripple" | "wave"]`).
Switching at runtime is strict: `Engine.set_strategy(name)` only swaps
in `STAND` and returns `False` otherwise. The teleop's D-pad cycler
publishes the chosen name on `/cmd_gait`; `hexa_control` multiplexes
it onto `GaitParams.gait_name`, and `gait_node` calls `set_strategy`
accordingly.

Inputs:
- `/gait/params` (`hexa_interfaces/GaitParams`) — gait selection and the
  target body velocity (`linear_x`, `linear_y`, `angular_z`).

Outputs:
- `/legs/targets` (`hexa_interfaces/LegState[6]`) — foot pose + phase.

The engine is stateful (it owns the phase clock) but the gait strategies
themselves are pure functions of `(phase, params) → foot_target`.

All walk-cycle knobs (`stride_length`, `min_swing_time`,
`max_cycle_time`, per-gait `duty_factor`, `step_height`, ...) live in
`config/gait.yaml` — they are not on the wire. `duty_factor` is
per-gait under the `gaits:` block; the engine reads it from the active
strategy each tick. The engine derives `min_cycle_time = min_swing_time /
(1 − β)` per gait so the swing-phase foot velocity ceiling is shared.

See [`docs/leg-phases.md`](../../docs/leg-phases.md) for the shared
vocabulary (stance, swing, AEP, PEP, duty factor, support polygon) used
below and throughout the codebase.

## Velocity → cycle_time

`cycle_time` is **not** a configured constant. The engine derives it
each GAIT tick from the commanded velocity and a fixed
`stride_length`:

- For each leg, compute `v_leg = v_body + ω_z × r_leg`.
- `max_leg_v = max( |v_leg| for all 6 legs )`.
- `β = active strategy's duty_factor`; `min_cycle = min_swing_time / (1 − β)`.
- `cycle_time = clip( stride_length / (max_leg_v × β), min_cycle, max_cycle_time )`.
- Per-leg `stride_vector = v_leg × cycle_time × β`, with the magnitude
  further clamped to `stride_length` so saturated commands never push
  past the joint-limit-safe footprint.

Faster commands therefore *cycle faster at constant stride* instead of
taking *bigger steps at constant cycle*. The implied per-leg velocity
ceiling per gait is `stride_length × (1 − β) / (min_swing_time × β)`;
tripod sits at the high end of this curve. Below the velocity that
implies `max_cycle_time`, the cycle stops dragging out — stride shrinks
linearly instead.

## Velocity caps for upstream nodes

`gait.yaml` is the **single source of truth** for the velocity caps that
upstream nodes (`hexa_teleop` for stick scaling, `hexa_control` for
`/cmd_vel` clamping) apply at their respective boundaries. Both load
the caps through `hexa_gait.load_velocity_caps(gait_yaml_path)` at
startup; there are no duplicate knobs in the teleop or control YAML.

- `linear_max` is **derived per-gait** as
  `stride_length × (1 − β) / (min_swing_time × β)` — exactly each gait's
  per-leg velocity ceiling. Tripod sits at the high end (β=0.5),
  ripple in the middle (β=2/3), wave at the low end (β=5/6). The
  cap is applied at the `/cmd_vel` boundary using the *active* gait's
  value: `hexa_control` looks it up on every tick, and `hexa_teleop`
  rebuilds its stick scaling whenever the user's D-pad cycler accepts
  a new gait. Anchoring on the active gait keeps the engagement
  controller's stance integration bounded — over-cap commands would
  push initial-stance feet past PEP and trip joint limits.
- `angular_z_max` is an **explicit** knob in `gait.yaml`. Kept explicit
  (not geometry-derived) because angular feel is harder to predict from
  leg radii — the gait's geometric ceiling
  (`linear_max / r_outer`) is typically well above what feels
  comfortable, so this knob trades reach for tunable feel.
- `yaw_bias` controls how `scale_to_envelope` splits the cut between
  translation and yaw when a combined command overruns the per-leg
  envelope. The reduction is allocated in ratio
  `yaw_bias : (1 − yaw_bias)`, so values above 0.5 push the cut onto
  translation and preserve more of the commanded yaw. `0.5` is the
  unbiased baseline (uniform scaling, direction preserved); `0.75` is
  the current setting (at full v_x + full angular_z_max the result is
  25% v, 75% ω instead of 50% / 50%). Pure yaw priority is the
  `yaw_bias → 1` limit. The trade-off is direction fidelity for yaw
  responsiveness at the extremes.

See `hexa_gait/hexa_gait/limits.py` for the helper API.

Command-velocity smoothing is **not** done here. `hexa_control` runs
a vectorial rate-cap `BodyVelocityLimiter` between `scale_to_envelope`
and the `/gait/params` publish, so the velocity arriving here is
already acceleration-bounded. The engagement smoothstep (`engagement.py`) still owns the
geometric `STAND → GAIT` foot handoff: it runs one full master cycle
during which each leg performs its single first swing from NOMINAL,
landing at the strategy's expected position by `master = 1.0`. The two
shapers compose harmlessly.

## Cold start: FOLDED → INITIALIZE

At power-on the hexapod sits on its belly with the legs folded above
the body (see `initial_pose:` in `hexa_description/config/geometry.yaml`).
On the real robot some servos cannot report their own angle, so the
operator is responsible for placing the chassis in roughly this folded
pose; the engine must not assume any other starting position. The first
state the engine enters is therefore `FOLDED`: the gait emits the
folded foot positions verbatim, ignores `cmd_vel`, and waits for an
operator trigger before doing anything. The trigger is a one-shot
`std_msgs/Empty` message on `/gait/initialize` (published by `hexa_teleop`
on the joystick start button's rising edge); the engine then transitions
to `INITIALIZE`, which runs an orchestrated ladder from `initial_pose`
to the standing pose:

1. **PLACE_FEET** — three sequential mirroring pairs swing one at a
   time from the folded foot position to the standing footprint with
   the foot held `place_feet_clearance` (~1 mm) above the floor, while
   the body stays on its belly. The small gap keeps the swing arc
   from scuffing the ground at touchdown and gives LIFT_BODY a clean
   ground-contact transition. Pair order is middle pair → front-left +
   rear-right diagonal → front-right + rear-left diagonal, chosen to
   keep the body's centre of mass near the chassis centre throughout
   (inactive legs hold their last positions; the body is supported on
   its belly and on whatever legs have already been placed). Each pair
   takes `pair_swing_time`.
2. **LIFT_BODY** — all six feet stay at their standing XY; their
   body-frame z ramps via a smoothstep S-curve from the PLACE_FEET
   endpoint (1 mm above the floor) down to each leg's standing z. The
   kinematics chain reads this as "legs extending down": as the feet
   make ground contact, the body lifts off its belly. Gait owns the
   lift here rather than handing off to posture, so the cold start
   needs no gait↔posture coordination topic.
3. **DONE** — the controller emits the nominal stance for every leg;
   the engine treats this as the cue to transition to `STAND`.

The ladder is non-preemptible — `cmd_vel` arriving mid-sequence is
ignored until the engine transitions to `STAND`, mirroring `STOPPING`'s
commit-to-completion contract. Tuning knobs (`pair_swing_time`,
`lift_body_time`, `swing_clearance`) live under `initialize:` in
`config/gait.yaml`.

## Stopping: idle and standing reset

When GaitParams arrives with zero velocity (sent by `hexa_control` when
`cmd_vel` goes idle), the engine does not simply freeze the phase clock
— that would leave any leg in mid-swing dangling in the air. Instead it
drains the legs to the nominal stance via a small queue of **group
swings**, where a group is a set of legs scheduled to swing in parallel.
Groups run back-to-back. The same algorithm works for every gait; only
the group composition changes.

The queue is built at the moment the engine enters STOPPING:

1. **Airborne group** — every leg currently in the swing window
   (`phase < 1 − β` per the active gait, or flagged airborne by the
   engine) goes in the first group. They are already in the air, so
   they must come down before anything else moves. For tripod this is
   one of the two natural triples; for ripple it can be one or two
   singletons that happened to overlap in the swing window; for wave
   it is at most one leg.
2. **Stance groups** — the remaining legs are bucketed by exact phase
   offset (legs sharing an offset are gait-natural parallel partners,
   e.g. tripod's three) and ordered by **descending current phase**.
   The bucket whose phase is closest to wrapping to 0 is the next leg
   the gait itself would have lifted off, so it goes next; the bucket
   furthest from a natural lift-off goes last. This continues the
   gait's cyclic rotation from where it stopped.
3. **Empty-group strip** — any group whose every leg is already at
   nominal is dropped (no twitch).

Each group's swings then run in parallel, rest-to-rest, with per-leg
adaptive duration `clamp(distance_xy / max_foot_speed, min_swing_time,
max_swing_time)`. The apex is the higher of `origin_z` and `target_z +
step_height`: a grounded leg lifts the full clearance; an airborne leg
already above that height descends with no extra bounce; an airborne
leg near the floor gets a partial lift to the same apex. Both endpoint
velocities are pinned to zero so the Bezier decelerates fully at
touchdown — landing at the steady-state stance velocity would slam the
foot into the floor and rock the chassis. The group completes when its
slowest leg lands; the next group starts immediately.

Once the queue is drained the controller emits the nominal stance with
`stance=True` for all six legs and the engine transitions to STAND.

Per-gait stop-time bound (at `min_swing_time = 0.3 s`, all legs near
nominal):

- **Tripod** (β=0.5, two offset groups) — ≤ 2 × `max_swing_time`.
- **Ripple** (β=2/3, up to five groups after merging the swing
  overlap) — ≤ 5 × `max_swing_time`.
- **Wave** (β=5/6, six groups) — ≤ 6 × `max_swing_time`.

### Stability

Sequential groups are strictly more conservative than the gait's
overlapping mid-walk swing windows. Tripod's two triples are each a
stable support set; ripple/wave never have more than one stance group
airborne at a time. The "support polygon contains the CoM" invariant is
*inherited from the gait itself* — β and the phase offsets are chosen
so it holds, and the disengagement controller's groups are subsets of
gait-natural lift-off events.

### Architectural note

The drain sequence is *stateful per leg* (each leg remembers where it
started and where it's going), which does not fit the
`(phase, params) → foot_target` pure-function contract of the gait
strategies. It is therefore implemented as a separate **disengagement
controller** (semantic counterpart to the engagement controller that
handles STAND → GAIT) alongside the strategies; the engine routes
between the active strategy and the disengagement controller based on
commanded velocity. The controller computes its own swing trajectories
rather than calling the strategy because at `stride = 0` the strategy
degenerates: PEP equals AEP equals nominal, the stance Bezier collapses
to a constant, and the swing arc reduces to a degenerate hop. Owning
the trajectory math here keeps the strategy contract clean while
letting the disengagement controller reuse the strategy's *schedule*
(phase offsets and duty factor) for free.

### Resume

If a non-zero velocity arrives mid-drain, the engine completes the
queue first and only then starts the new gait from the nominal stance.
Aborting mid-sequence would risk legs left in unsafe poses.
