# hexa_posture

Body posture controller. Turns the user's body-pose input and the current
locomotion state into a single, clamped body pose target for the IK node.

Parallel to `hexa_gait`: gait owns leg trajectories, posture owns the body
trajectory. `hexa_kinematics` composes the two downstream — neither depends on
the other.

## Layers

Mirrors the library/node split from `hexa_kinematics`:

- **Library** (`hexa_posture/`) — pure Python, no ROS imports.
  - `pose.py` — `BodyPose` dataclass, additive composition, safety-envelope clamp.
  - `animations/` — strategy interface (`Animation`, `AnimationContext`, `Stack`)
    plus `Still`, `Breathing`, `GaitSway`, `GaitBounce`. Each is a pure function
    from context → pose offset.
- **Node** (`posture_node.py`) — runs the animation stack on a timer, composes
  user pose + animations, clamps to the envelope, publishes `/body/pose_target`.

## Topics

Subscribes:

- `/body/pose` (`hexa_interfaces/BodyPose`) — user-commanded body offset. Latest
  sample wins.
- `/cmd_vel` (`geometry_msgs/Twist`) — read for the walking-vs-idle distinction;
  never modified.
- `/gait/state` (`std_msgs/String`) — engine state name. User pose + animations
  apply only in `stand` / `engaging` / `gait` / `stopping`; otherwise (or before
  the first message) the node publishes IDENTITY. This stops the user translating
  the chassis while legs are folded or mid-cold-start.
- `/legs/targets` (`hexa_interfaces/LegTargets`) — per-leg foot targets and stance
  flags, used to derive the support-polygon centroid and swing lift. Consumed via
  the topic contract only; `hexa_posture` does not import `hexa_gait` (keeps the
  velocity and body-pose chains decoupled per `CLAUDE.md`).

Publishes:

- `/body/pose_target` (`hexa_interfaces/BodyPose`) at 200 Hz — the final clamped
  pose for the IK node.

## Animation contract

Animations are pure functions of an `AnimationContext`. All animation state
(amplitude, period) lives in the instance; the node owns the clock, so animations
must not call `time.time()` or read ROS clocks.

- `t` — monotonic time (s), passed in explicitly for deterministic tests.
- `walking` — True iff `/cmd_vel` is non-zero. Lets animations gate to pose mode
  (`Breathing`) or gait-active mode (sway, bounce).
- `gait_phase` — reserved for phase-locked animations. Phase is not on the wire
  yet, so animations that want it fall back to a free-running sine on `t` or skip.
- `support_centroid_xy` — low-pass-filtered XY centroid of the support polygon
  (m, body frame), from `/legs/targets`. `None` until the first sample.
- `swing_lift_z` — low-pass-filtered max foot lift (m) above the stance polygon,
  from `/legs/targets`. `None` until first usable sample; `0.0` once observed with
  no leg in swing.

Composition is component-wise addition (`pose.add`), valid only for the small
offsets posture operates in (cm translations, single-digit-degree rotations).
Larger amplitudes would need real SE(3) — document that at the call site.

### GaitSway

Planar body translation tracking the live support-polygon centroid. Suppresses
the rocking mode that four-foot stance polygons (tetrapod, crawl, surf) excite:
their centroid is offset from the body origin, so feeding it back into the body
XY pose cancels the gravity-driven torque. For tripod (3-foot) and ripple
(5-foot) the centroid sits near the origin, so the output self-attenuates without
per-gait gating.

- Emits `BodyPose(x=gain·cx, y=gain·cy)` while walking; does nothing when
  `walking=False` or the centroid is unobserved (`None`).
- Knobs (`tuning.yaml`, `posture_node` block):
  - `gait_sway_gain` (1.0) — feedforward gain on the centroid; 1.0 tracks
    one-for-one.
  - `gait_sway_strength` (0.4) — user-facing attenuator in `[0, 1]`; 0.0 disables.
  - `support_centroid_tau` (0.1 s) — low-pass time constant on the centroid.
- Disabled by default in the node — add `gait_sway` to `enabled_animations`
  (shipped `tuning.yaml` already does, alongside `still`).

### GaitBounce

Vertical body lift synced to the gait so the chassis travels with the swinging
feet instead of rocking against them. Stacks on `GaitSway` (sway = XY, bounce = Z).

- Emits `BodyPose(z = arc_height · swing_lift / step_height_ref)`, clamped to
  `[0, arc_height]`: at rest (`z=0`) when no foot is lifted, at peak when the
  highest swinging foot is at its apex. Does nothing when `walking=False`, the
  lift is unobserved (`None`), or `arc_height=0`.
- Overlapping gaits: the node aggregates lift as the *max* across all legs. For
  non-overlapping gaits (tripod, tetrapod, ripple) only one group is airborne, so
  the body bounces `N` times per master cycle (`N` = swing groups). For overlapping
  gaits (crawl, surf) the max follows the main wave; the trough sits at the
  inter-peak overlap height, not zero.
- Knobs (`tuning.yaml`, `posture_node` block):
  - `gait_bounce_arc_height` (0.02 m) — peak body lift at swing apex.
  - `gait_bounce_step_height_ref` (0.08 m) — reference apex normalising the lift;
    mirrors `gait_node` `step_height` so `arc_height` stays a real metre value.
  - `swing_lift_tau` (0.04 s) — low-pass on the swing-lift signal. Keep small
    relative to the gait sub-cycle so the bounce doesn't lag the feet.

## Safety envelope

`PoseLimits` clamps each axis symmetrically against a static envelope — a coarse
upstream guard against runaway animations and teleop glitches. The real reachable
envelope is geometry-dependent and lives (or will live) in the IK node; don't rely
on this clamp as the only safety layer.

## Configuration

`hexa_description/config/tuning.yaml` (the `posture_node` block) is the single
source of truth for runtime knobs (animation stack, GaitSway tuning, filters). It
loads as a standard ROS2 parameter file; bringup passes it to the node, and
`--ros-args -p name:=value` still overrides individual values.

## Roadmap

- Phase-locked animations once `/gait/state` carries the cycle phase.
- YAML-driven animation registry — declare each layer with its own parameter block.
- Dynamic per-axis clamp from current foot positions and leg geometry.
- Service interface (`SetPose.srv`?) for one-shot pose commands from autonomy.
