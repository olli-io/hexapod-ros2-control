# hexa_kinematics_cpp

C++ (`ament_cmake`) port of `hexa_kinematics` — forward / inverse kinematics,
body-frame transforms, and the IK and joint-command-bridge nodes. Built
**side-by-side** with the Python `hexa_kinematics`; both packages compile.
`hexa_bringup` still launches the Python nodes until the C++ nodes are verified
and cut over.

## Layout

- `include/hexa_kinematics_cpp/` — public headers. `types.hpp` (`Point3 =
  Eigen::Vector3d`, `JointAngles = std::array<double, 3>`, `UnreachableTarget`),
  `leg_geometry.hpp` (`LegSpec`), `leg_ik.hpp`, `leg_specs.hpp`,
  `body_transform.hpp`, `joint_config.hpp`, `yaml_util.hpp`.
- `src/` — the pure library (`leg_ik`, `leg_specs`, `body_transform`,
  `joint_config`) plus the two `rclcpp` nodes (`ik_node.cpp`,
  `joint_command_bridge.cpp`).
- `launch/kinematics.launch.py` — standalone bench launcher for both nodes.

Geometry parameters come from `hexa_description` (loaded at startup), so the
library doesn't hard-code leg dimensions.

## Architecture

- The library links without `rclcpp` — every kinematics module is unit-testable
  standalone, matching the Python contract. Only the node executables touch ROS.
- Math uses Eigen (`Point3 = Eigen::Vector3d`); YAML uses `yaml-cpp` via the
  `require_scalar` / `load_file` helpers in `yaml_util.hpp`.
- `Point3` and `JointAngles` use the same underlying types as `hexa_gait_cpp`,
  so the gait engine consumes this library through a one-line namespace alias
  (`namespace kin = ::hexa_kinematics`) in place of its old kinematics stub.

## Nodes

- `ik_node` — subscribes to `/legs/targets` (`LegTargets`, `LegState[6]`) and
  `/body/pose_target` (`BodyPose`), composes the pose offset onto each foot
  target (`apply_body_pose → body_to_leg → inverse_kinematics`), and publishes
  `/joint_commands` (`sensor_msgs/JointState`) in the 18-joint URDF order. A
  transient `UnreachableTarget` on one leg holds that leg's last angles instead
  of zeroing joints.
- `joint_command_bridge` — adapts `/joint_commands` (`JointState`) to
  `std_msgs/Float64MultiArray` for ros2_control position group controllers,
  emitting a fixed-order array. Topics and joint order are parameters.

## Conventions

Frame, joint-angle, and branch conventions are documented in
`include/hexa_kinematics_cpp/leg_geometry.hpp`. The headlines:

- Coxa-mount frame is right-handed (REP-103): `+x` radially outward, `+y` left,
  `+z` up.
- Joint zeros: `θ_femur = 0` puts the femur horizontal, `θ_tibia = 0` puts the
  tibia colinear with the femur (leg fully extended).
- `inverse_kinematics` returns the **knee-up** branch.

## Known gaps

Carried over from the Python package:

- **Joint limits are not enforced.** `inverse_kinematics` returns the
  unconstrained mathematical solution. Per-joint-type limits are loadable via
  `load_joint_limits` (the `joints:` block of
  `hexa_description/config/geometry.yaml`) but not yet wired into `LegSpec` / IK.

## Build & test

    ./hexa dev
    pod build   # or: colcon build --packages-select hexa_kinematics_cpp
    colcon test --packages-select hexa_kinematics_cpp
    ros2 run hexa_kinematics_cpp ik_node
