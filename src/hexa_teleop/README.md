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

## Run it (8BitDo Pro 2)

Power the controller on in **X-input mode**: hold `Start + X` until the
LEDs flash, then plug in via USB-C.

Inside the dev container — uncomment the `/dev/input` block in
`docker-compose.yaml` before `./hexapod.sh --dev` so `joy_node` can see
the device.

- Launch the sim (separate terminal): `ros2 launch hexa_bringup sim.launch.py`
- Launch teleop: `ros2 launch hexa_teleop teleop.launch.py`

Controller mapping:

- **Right stick** — body translation (posture mode) or linear velocity (gait mode). Stick forward → body `+x`; stick left → body `+y`.
- **Left stick X** — yaw rate (gait mode only).
- **Y button** — toggles posture ↔ gait on rising edge.

The node starts in **posture** mode (safer: no walking). The mapping,
deadband, and per-axis maxima live in
`config/teleop_joy.yaml` — override at launch with
`joy_config_file:=/path/to/file.yaml`.
