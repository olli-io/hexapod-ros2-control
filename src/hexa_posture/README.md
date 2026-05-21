# hexa_posture

Body posture controller. Turns the user's body-pose input and the
current locomotion state into a single, clamped body pose target for
the IK node.

Sits parallel to `hexa_gait` in the stack: gait owns leg trajectories,
posture owns body trajectory. The two are composed downstream by
`hexa_kinematics` — neither depends on the other.

## Layers

Mirrors the library/node split used by `hexa_kinematics`:

- **Library** (`hexa_posture/`) — pure Python, no ROS imports.
  - `pose.py` — internal `BodyPose` dataclass, additive composition,
    static safety-envelope clamp.
  - `animations/` — animation strategy interface (`Animation`,
    `AnimationContext`, `Stack`) and starter implementations
    (`Still`, `Breathing`, `GaitSway`, `GaitBounce`). Each
    animation is a pure function from context → pose offset.
- **Node** (`posture_node.py`) — subscribes `/body/pose` (user input),
  `/cmd_vel` (walking-vs-idle state), and `/gait/state` (engine state
  gate). Runs the animation stack on a timer, composes user pose +
  animations, clamps to the envelope, publishes `/body/pose_target`.

## Topics

- Subscribes:
  - `/body/pose` (`hexa_interfaces/BodyPose`) — user-commanded body
    offset from teleop or autonomy. Latest sample wins.
  - `/cmd_vel` (`geometry_msgs/Twist`) — read for the
    walking-vs-idle distinction. Posture does not modify it.
  - `/gait/state` (`std_msgs/String`) — engine state name
    (`folded`, `initialize`, `stand`, `engaging`, `gait`, `stopping`,
    `folding`). Posture only applies user pose + animations when the
    state is one of `stand` / `engaging` / `gait` / `stopping`; in
    the other states (or before the first message arrives) the node
    publishes IDENTITY. This is what stops the user from translating
    the chassis while the legs are folded or mid-cold-start.
  - `/legs/targets` (`hexa_interfaces/LegTargets`) — per-leg foot
    targets and stance flags. Read to derive the support-polygon
    centroid for `GaitSway`. The signal is consumed in the posture
    chain but produced by `hexa_gait`; we depend only on the topic
    contract, not on the gait package — the velocity and body-pose
    chains stay decoupled per the architectural rule in
    `CLAUDE.md`.
- Publishes:
  - `/body/pose_target` (`hexa_interfaces/BodyPose`) at 50 Hz — the
    final clamped pose for the IK node to apply via
    `apply_body_pose`.

## Animation contract

Animations are pure functions of an `AnimationContext`:

- `t` — monotonic time (s), passed in explicitly so animations stay
  deterministic under test.
- `walking` — True iff `/cmd_vel` is non-zero. Lets animations gate
  themselves to pose mode (e.g. `Breathing`) or gait-active mode
  (e.g. sway, lean).
- `gait_phase` — reserved for phase-locked animations (gait-synced
  sway, body bob in time with stride). The current `/gait/state`
  message carries only the engine state name; phase is not on the
  wire yet, so animations that want it must fall back to a
  free-running sine on `t` or skip themselves.
- `support_centroid_xy` — low-pass-filtered XY centroid of the
  current support polygon (metres, body frame), derived from
  `/legs/targets`. `None` until the first sample arrives. The node
  owns the filter so animations stay stateless.
- `swing_lift_z` — low-pass-filtered max foot lift (metres) above
  the stance polygon across all legs, derived from `/legs/targets`.
  Drives the gait-synced vertical bounce. `None` until the first
  usable /legs/targets sample arrives; `0.0` once observed with no
  leg in swing. The filter rounds off the slope kink at leg
  handover in overlapping gaits.

All state lives in the animation instance (e.g. amplitude, period),
not in the context. The posture node owns the clock; animations must
not call `time.time()` or read ROS clocks directly.

Composition uses component-wise addition (`pose.add`). This is valid
only for small offsets, which is the regime posture operates in (cm
translations, single-digit-degree rotations). If an animation grows
larger amplitudes, the composition needs to graduate to real SE(3) —
document that at the call site.

### GaitSway

A planar body translation that tracks the live support-polygon
centroid. Suppresses the rocking mode that four-foot stance polygons
(tetrapod, ripple, surf) excite each cycle — the centroid is offset
from the body origin on those gaits, so feeding it back into the body
XY pose removes the gravity-driven torque on the rocking axis.

- **What it does** — emits `BodyPose(x=gain·cx, y=gain·cy)` while
  walking, where `(cx, cy)` is the filtered centroid.
- **When it does nothing** — `walking=False`, or the centroid hasn't
  been observed yet (`None`). For tripod (3-foot polygon) and wave
  (5-foot polygon) the centroid lands near the body origin, so the
  output naturally self-attenuates without per-gait gating.
- **Why it works without crossing chains** — the signal it needs is
  already on `/legs/targets`. `hexa_posture` does not import from
  `hexa_gait`; only the topic contract is shared.
- **Knobs** — three ROS params, all sourced from
  `hexa_posture/config/posture.yaml`:
  - `gait_sway_gain` (default 1.0) — physical feedforward gain on
    the centroid. 1.0 makes the body track the polygon centroid
    one-for-one.
  - `gait_sway_strength` (default 0.5) — user-facing attenuator in
    `[0, 1]` that multiplies the gain output. Lets you tone the
    sway down without changing the physical gain. 0.0 disables.
  - `support_centroid_tau` (default 0.1 s) — first-order low-pass
    time constant on the centroid signal.
  Disabled by default in the node — add `gait_sway` to
  `enabled_animations` (the shipped `posture.yaml` already does so
  alongside `still`).

### GaitBounce

A vertical body lift synced to the gait so the chassis travels with
the swinging feet instead of rocking against them. Stacks cleanly on
top of `GaitSway`: sway handles XY centroid tracking, bounce handles
Z.

- **What it does** — emits `BodyPose(z = arc_height · swing_lift /
  step_height_ref)`, clamped to `[0, arc_height]`. Body sits at rest
  (`z = 0`) when no foot is lifted (i.e. at AEP / touchdown) and at
  peak (`z = arc_height`) when the highest swinging foot is at its
  swing apex.
- **When it does nothing** — `walking=False`, the lift signal hasn't
  been observed yet (`None`), or `arc_height = 0`.
- **How it handles overlapping gaits** — the node aggregates
  `swing_lift_z` as the *max* foot lift across all legs. For
  non-overlapping gaits (tripod, tetrapod, wave) only one swing
  group is airborne at a time, so the max IS that group's arc and
  the body bounces `N` times per master cycle (`N` = number of
  swing groups). For overlapping gaits (ripple, surf) the max
  picks whichever airborne leg is closest to its apex, so the
  bounce follows the main wave and ignores the half-phase leg
  lagging or leading behind it. The trough sits at the inter-peak
  overlap height, not at zero.
- **Knobs** — two ROS params, both from
  `hexa_posture/config/posture.yaml`:
  - `gait_bounce_arc_height` (default 0.02 m) — peak body lift at
    swing apex.
  - `gait_bounce_step_height_ref` (default 0.06 m) — reference
    swing apex used to normalise the lift signal. Mirror
    `hexa_gait/config/gait.yaml`'s `step_height` so `arc_height`
    continues to represent the actual peak body lift in metres.
  - `swing_lift_tau` (default 0.04 s) — first-order low-pass time
    constant on the swing-lift signal feeding the animation.
    Rounds off the slope kink at leg handover in overlapping
    gaits; keep small relative to the gait sub-cycle so the body
    bounce doesn't visibly lag the feet.

## Safety envelope

`PoseLimits` clamps each axis symmetrically against a static envelope.
This is a coarse upstream guard against runaway animations and teleop
glitches; the real reachable envelope is geometry-dependent and lives
(or will live) in the IK node. Don't rely on this clamp as the only
safety layer.

## Configuration

`config/posture.yaml` is the single source of truth for the node's
runtime knobs (animation stack, GaitSway tuning, centroid filter).
It loads as a standard ROS2 parameter file under the `posture_node`
key. The bringup launch passes it to the node; overriding any value
on the command line via `--ros-args -p name:=value` still works.

## Roadmap

- Phase-locked animations once `/gait/state` carries the current
  cycle phase alongside the engine state name.
- YAML-driven animation registry — instead of the current name list
  plus per-animation params, declare each layer with its own
  parameter block.
- Smoother envelope: use the current foot positions and leg geometry
  to compute a per-axis dynamic clamp.
- Service interface (`SetPose.srv`?) for one-shot pose commands from
  autonomy, distinct from streamed teleop input.
