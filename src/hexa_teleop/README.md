# hexa_teleop

Human-input nodes that publish to `/cmd_vel` and `/body/pose`.

- `teleop_joy.py` — wraps `joy_node` (`sensor_msgs/Joy` → `Twist`). Axis
  mapping configurable via YAML.
- `teleop_key.py` — terminal keyboard fallback for bench testing.

Intentionally thin: this package exists so the rest of the stack has a
single, swappable producer of high-level commands. An autonomy node would
publish the same topics and replace teleop transparently.
