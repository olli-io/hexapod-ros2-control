# hexa_description

Robot description package: URDF (via xacro), meshes, joint limits, and the
`robot_state_publisher` configuration.

This package is the single source of truth for the robot's kinematic structure.
Both the kinematics library and the simulation consume the URDF produced here.

Contents (to be added):
- `urdf/hexapod.urdf.xacro` — parameterised description (leg length, coxa/femur/tibia, body geometry).
- `urdf/hexapod.gazebo.xacro` — Gazebo plugin tags (ros2_control, IMU, etc.).
- `meshes/` — visual + collision meshes per leg segment.
- `config/geometry.yaml` — also carries a `joints:` block with the absolute travel window (`lower_limit_deg` / `upper_limit_deg`) plus sim-only `effort` / `velocity`, expressed in intuitive per-joint degrees (coxa sweep, femur above horizontal, tibia interior). The URDF and `shared/motion_core/tools/gen_config.py` convert these to IK-convention radians at load time (sign-aware: femur and tibia conversions are monotonically decreasing, so intuitive `upper` maps to URDF `lower` and vice versa). `mounts.*.yaw_deg` is also in degrees; the only radian values live inside generated URDF text.
- The block is target-agnostic — limits apply equally to sim and the real robot. The joint angle at **servo center** is a property of the physical build and lives only in `config/hardware.yaml`'s `deg_at_center`; `gen_config.py` cross-checks it against the window above.
- The default at-rest stance is not a file here either — it lives in `config/tuning.yaml`'s `gait_node` `default_standing_pose` ros params, described by where the feet sit rather than by joint angles: one belly clearance for the body, plus a `tip_reach` (coxa axis to foot tip, in the ground plane) and a `coxa_deg` splay for each of the front, middle and rear pairs. Left and right mirror. Decoupled from the servo center so an asymmetric build can set them independently.
- `launch/description.launch.py` — publishes the URDF on `/robot_description`.
