# hexa_posture_cpp

C++ port of `hexa_posture`, built **side-by-side** with the Python package (both
compile). `hexa_bringup` still launches the Python `posture_node`; cutover and
deletion of the Python package are later tasks. `hexa_teleop` continues to read
the Python `hexa_posture.config` helper for the animation-mode list, so that
helper is intentionally **not** ported here.

The posture controller produces a body-pose offset on `/body/pose_target` by
summing a stack of pure animation layers on top of the user pose, gated by gait
state, then clamping to a static safety envelope. The IK node composes that
offset with the per-leg foot targets before solving IK.

## Layout

- **library / node split** — everything except `posture_node.cpp` is pure,
  ROS-free, and unit-testable. Only the node touches `rclcpp`.
- `include/hexa_posture_cpp/pose.hpp` — `BodyPose`, `PoseLimits`, `IDENTITY`,
  and `add` / `scale` / `clamp`. Port of `pose.py`.
- `include/hexa_posture_cpp/animations.hpp` — `AnimationContext`, the
  `Animation` interface, `Stack`, the seven concrete animations, and
  `build_animation_stack`. Consolidates all of `animations/*.py`.
- `include/hexa_posture_cpp/signals.hpp` — the pure telemetry helpers:
  `POSTURE_ACTIVE_STATES` / `is_posture_active`, `twist_is_zero`,
  `stance_centroid_xy`, `max_swing_lift_z`, and the low-pass filters.
  Consolidates the free functions from `posture_node.py`.
- `src/posture_node.cpp` — the `rclcpp` node: parameters, QoS, the 200 Hz tick,
  and message ↔ pure-type conversions.

## Topics

- **subscribes** — `/body/pose` (`hexa_interfaces/BodyPose`, user pose offset);
  `/cmd_vel` (`geometry_msgs/Twist`, walking-vs-idle); `/gait/state`
  (`std_msgs/String`, engine-state gate); `/gait/params`
  (`hexa_interfaces/GaitParams`, active gait name); `/legs/targets`
  (`hexa_interfaces/LegTargets`, support centroid / swing lift / master phase);
  `/animation/mode` (`std_msgs/String`, depth-1 transient-local).
- **publishes** — `/body/pose_target` (`hexa_interfaces/BodyPose`) at 200 Hz.

## Config

- Runtime knobs (animation stack selection and per-animation parameters) come
  from `hexa_description/config/tuning.yaml` (the `posture_node` block) — the
  single source of truth shared with the Python node. See its inline comments
  for what each parameter does.

## Standalone run

- `ros2 launch hexa_posture_cpp posture.launch.py` — brings up the node on a
  bench with `hexa_description`'s tuning.yaml applied.
