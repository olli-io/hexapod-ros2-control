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
    (`Still`, `Breathing`). Each animation is a pure function from
    context → pose offset.
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

All state lives in the animation instance (e.g. amplitude, period),
not in the context. The posture node owns the clock; animations must
not call `time.time()` or read ROS clocks directly.

Composition uses component-wise addition (`pose.add`). This is valid
only for small offsets, which is the regime posture operates in (cm
translations, single-digit-degree rotations). If an animation grows
larger amplitudes, the composition needs to graduate to real SE(3) —
document that at the call site.

## Safety envelope

`PoseLimits` clamps each axis symmetrically against a static envelope.
This is a coarse upstream guard against runaway animations and teleop
glitches; the real reachable envelope is geometry-dependent and lives
(or will live) in the IK node. Don't rely on this clamp as the only
safety layer.

## Roadmap

- Phase-locked animations once `/gait/state` carries the current
  cycle phase alongside the engine state name.
- Config-driven animation stack (`animations.yaml` listing layer
  classes + parameters, loaded at startup).
- Smoother envelope: use the current foot positions and leg geometry
  to compute a per-axis dynamic clamp.
- Service interface (`SetPose.srv`?) for one-shot pose commands from
  autonomy, distinct from streamed teleop input.
