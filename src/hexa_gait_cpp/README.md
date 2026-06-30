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

## Temporary kinematics stub

`include/hexa_gait_cpp/kinematics_stub.hpp` (namespace `hexa_gait::kin`) is a
**placeholder** for the `hexa_kinematics` surface (`LegSpec`, `load_leg_specs`,
`leg_to_body`, `forward_kinematics`, `load_standing_pose`, `load_initial_pose`),
which is still Python. Every stub returns zeros / degenerate geometry, so:

- The engine **builds** and the state machine **runs** — message flow and state
  transitions are exercisable.
- Nominal / initial / reseat stance **values are wrong** until `hexa_kinematics`
  is ported to C++ (and `leg_specs` moves to `hexa_description`). Replace the
  stub then — each function is tagged `// TODO(kinematics-port)` and the swap is
  a one-line include change plus a namespace alias.

## Tests

Deferred to a separate task: the 15 `hexa_gait` pytest suites are not yet ported
to `ament_cmake_gtest`. `CMakeLists.txt` has an empty `BUILD_TESTING` block with
a TODO; the pure engine library links without ROS so gtest targets can exercise
it directly (mirror `hexa_hardware`).

## Build & run

    ./hexa dev
    pod build   # or: colcon build --packages-select hexa_gait_cpp
    ros2 run hexa_gait_cpp gait_node
