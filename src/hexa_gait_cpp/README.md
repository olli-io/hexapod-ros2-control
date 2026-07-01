# hexa_gait_cpp

C++ (`ament_cmake`) port of `hexa_gait` — the stateful gait engine that turns a
commanded body velocity into per-leg foot targets. Built **side-by-side** with
the Python `hexa_gait`; both packages compile. `hexa_bringup` still launches the
Python `gait_node` until the C++ node is verified and cut over.

## Layout

- `include/hexa_gait_cpp/` — public headers (engine, clock, trajectory, gaits,
  controllers).
- `src/` — the pure engine library (no ROS) plus `gait_node.cpp` (the only
  `rclcpp` component).
- `config/gait.yaml` — engine knobs (copied from `hexa_gait`; `hexa_description`
  stays the single source of truth for geometry / standing pose).
- `launch/gait.launch.py` — standalone bench launcher.

## Architecture

- The engine library links without `rclcpp` — every gait module is unit-testable
  standalone, matching the Python contract.
- Gait strategies are pure `(phase, stride, leg) → foot_target` functions; the
  `Engine` owns the phase clock and the FOLDED / INITIALIZE / STAND / ENGAGING /
  GAIT / PAUSING / PAUSED / RESUMING / FOLDING / RESEATING state machine.
- Math uses Eigen (`Vec3 = Eigen::Vector3d`); YAML uses `yaml-cpp`.

## Kinematics

`include/hexa_gait_cpp/kinematics.hpp` includes the real C++ kinematics library
(`hexa_kinematics_cpp`) and aliases its namespace as `hexa_gait::kin`, providing
the surface the engine consumes (`LegSpec`, `load_leg_specs`, `leg_to_body`,
`forward_kinematics`, `load_standing_pose`, `load_initial_pose`). Because both
packages share the same underlying types (`Vec3 = Eigen::Vector3d`,
`JointAngles = std::array<double, 3>`), no engine code changed — nominal /
initial / reseat stance values are now real geometry.

This replaced the former compile-only `kinematics_stub.hpp` (which returned
zeros). The kinematics library's exported surface is ROS-free (Eigen + yaml-cpp
only), so the engine library still links without `rclcpp`.

## Tests

Deferred to a separate task: the 15 `hexa_gait` pytest suites are not yet ported
to `ament_cmake_gtest`. `CMakeLists.txt` has an empty `BUILD_TESTING` block with
a TODO; the pure engine library links without ROS so gtest targets can exercise
it directly (mirror `hexa_hardware`).

## Build & run

    ./hexa dev
    pod build   # or: colcon build --packages-select hexa_gait_cpp
    ros2 run hexa_gait_cpp gait_node
