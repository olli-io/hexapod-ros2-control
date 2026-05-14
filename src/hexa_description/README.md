# hexa_description

Robot description package: URDF (via xacro), meshes, joint limits, and the
`robot_state_publisher` configuration.

This package is the single source of truth for the robot's kinematic structure.
Both the kinematics library and the simulation consume the URDF produced here.

Contents (to be added):
- `urdf/hexapod.urdf.xacro` — parameterised description (leg length, coxa/femur/tibia, body geometry).
- `urdf/hexapod.gazebo.xacro` — Gazebo plugin tags (ros2_control, IMU, etc.).
- `meshes/` — visual + collision meshes per leg segment.
- `config/joint_limits.yaml` — per-joint min/max angles matching the real servos.
- `launch/description.launch.py` — publishes the URDF on `/robot_description`.
