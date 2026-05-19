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
`docker-compose.yaml` before `./pod --dev` so `joy_node` can see
the device.

- Launch the sim (separate terminal): `ros2 launch hexa_bringup sim.launch.py`
- Launch teleop: `ros2 launch hexa_teleop teleop.launch.py`

Controller mapping:

- **Right stick** — body translation (posture mode) or linear velocity (gait mode). Stick forward → body `+x`; stick left → body `+y`.
- **Left stick (posture mode)** — body tilt toward the stick direction. Stick forward → pitch forward (front dips); stick left → roll left (left side dips).
- **Left stick X (gait mode)** — yaw rate.
- **L1 / R1 (posture mode)** — body yaw about `+z`. L1 yaws left (CCW from above), R1 yaws right. The buttons are binary, so the output is eased through a first-order low-pass (time constant `posture.yaw_tau_s`, default ~0.1 s) and saturates at `posture.yaw_max_deg` while held. Both buttons pressed cancel to zero. Inactive in gait mode.
- **L2 / R2 (posture mode)** — "wiggle". Same yaw as L1 / R1 (shared target — L1 + L2 does not double up) plus a body translation that holds a point `posture.wiggle_pivot_forward_m` ahead of body centre stationary, so the rear swings while the front stays planted. Triggers are read as analog axes and thresholded at `wiggle_trigger_threshold` (default 0.5 of the joy_node Xbox-style trigger range). Translation eases through the same low-pass as yaw so engaging the wiggle mid-L1-hold doesn't snap. Inactive in gait mode.
- **Y button** — toggles posture ↔ gait on rising edge.

The node starts in **posture** mode (safer: no walking). The axis
indices, deadband, posture translation limits, and toggle button live in
`config/teleop_joy.yaml` — override at launch with
`joy_config_file:=/path/to/file.yaml`.

Gait-mode stick scaling — the linear and angular velocity caps applied
to a full-stick deflection — is **not** owned here. It is loaded at
startup from `hexa_gait/config/gait.yaml` via
`hexa_gait.load_velocity_caps`:

- Linear is **isotropic**: full-stick forward and full-stick sideways
  both saturate at `linear_max = stride_length / (min_cycle_time × duty_factor)`.
- Angular cap is the explicit `angular_z_max` knob in `gait.yaml`.

To change those caps, edit `gait.yaml`; no teleop config edit needed.
