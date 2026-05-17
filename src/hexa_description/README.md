# hexa_description

Robot description package: URDF (via xacro), meshes, joint limits, and the
`robot_state_publisher` configuration.

This package is the single source of truth for the robot's kinematic structure.
Both the kinematics library and the simulation consume the URDF produced here.

Contents (to be added):
- `urdf/hexapod.urdf.xacro` — parameterised description (leg length, coxa/femur/tibia, body geometry).
- `urdf/hexapod.gazebo.xacro` — Gazebo plugin tags (ros2_control, IMU, etc.).
- `meshes/` — visual + collision meshes per leg segment.
- `config/geometry.yaml` — also carries a `joints:` block with per-joint servo center plus absolute `lower_limit_deg` / `upper_limit_deg`, expressed in intuitive per-joint degrees (`coxa.deg`, `femur.above_horizontal_deg`, `tibia.interior_deg`). The URDF and `hexa_kinematics.joint_config` convert these to IK-convention radians at load time (sign-aware: femur and tibia conversions are monotonically decreasing, so intuitive `upper` maps to URDF `lower` and vice versa). `mounts.*.yaw_deg` is also in degrees; the only radian values live inside generated URDF text.
- `config/standing_pose.yaml` — per-joint default at-rest angle in the same intuitive units. Decoupled from the servo center so an asymmetric build can set them independently. Consumed by the stub stance publisher (and the future `hexa_gait` STAND state).
- `launch/description.launch.py` — publishes the URDF on `/robot_description`.
