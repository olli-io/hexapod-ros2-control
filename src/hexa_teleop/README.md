# hexa_teleop

Human-input nodes that publish to `/cmd_vel` (body velocity, consumed
by `hexa_control`) and `/body/pose` (raw user body pose offset,
consumed by `hexa_posture`).

- `teleop_joy.py` — wraps `joy_node` (`sensor_msgs/Joy` → `Twist` +
  `BodyPose`). Axis mapping configurable via YAML — including a
  modifier (e.g. shoulder button) that switches stick mode between
  "drive" (`/cmd_vel`) and "pose" (`/body/pose`: yaw, x/y, height,
  pitch/roll).
- `teleop_key.py` — terminal keyboard fallback for bench testing.

Intentionally thin: this package exists so the rest of the stack has a
single, swappable producer of high-level commands. An autonomy node would
publish the same topics and replace teleop transparently.
