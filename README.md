# Hexapod

ROS2 control stack for a 6-leg / 18-DOF hexapod robot.

- **Hardware target**: Raspberry Pi 3 or 4 ( recommended OS: Pi OS lite ) driving a Pimoroni Servo 2040 over USB serial.

## Build / run

[`docs/dev-environment.md`](docs/dev-environment.md) for the dev/desktop container workflow.
[`docs/robot-environment.md`](docs/robot-environment.md) for preparing a fresh Pi to receive deploys.

## Configuration

All tunable parameters live in YAML files under each package's `config/` directory — never hard-coded in node code. Edit the YAML, rebuild (`pod build` re-links instantly), relaunch.

- [`src/hexa_description/config/geometry.yaml`](src/hexa_description/config/geometry.yaml) — body dimensions, leg segment lengths / radii / masses, foot, per-leg hip mounts, and per-joint-type (coxa / femur / tibia) servo center, lower / upper travel limits, effort, and velocity. Single source of truth for the robot's shape and joint travel; loaded into the URDF via xacro.
- [`src/hexa_description/config/standing_pose.yaml`](src/hexa_description/config/standing_pose.yaml) — per-joint angles (coxa / femur / tibia) at rest. Drives nominal foot targets via FK; kept separate from servo center so an asymmetric build can diverge.
- [`src/hexa_teleop/config/teleop_joy.yaml`](src/hexa_teleop/config/teleop_joy.yaml) — joystick axis / button mapping, deadband, posture↔gait toggle button, initial mode, and the max `cmd_vel` and posture offsets each mode emits.
- [`src/hexa_control/config/control.yaml`](src/hexa_control/config/control.yaml) — default gait selection and `cmd_vel` ramp / snap tolerances used to shape teleop input before it hits the gait engine.
- [`src/hexa_gait/config/gait.yaml`](src/hexa_gait/config/gait.yaml) — gait engine knobs: controller tick, default gait, stride length, step height, swing width, and swing-time bounds that anchor per-gait cycle-time limits.
- [`src/hexa_posture/config/posture.yaml`](src/hexa_posture/config/posture.yaml) — posture node animation stack: which gait-coupled and animation-mode animations are enabled and their gain / strength / amplitude knobs.
- [`src/hexa_hardware/config/hardware.yaml`](src/hexa_hardware/config/hardware.yaml) — Servo 2040 wiring (transport, device, per-pin joint assignment), pulse-width calibration endpoints, electrical clamps, and aux ADC scales. Real-robot only.
- [`src/hexa_simulation/config/ros2_controllers.yaml`](src/hexa_simulation/config/ros2_controllers.yaml) — ros2_control controller-manager rate and joint-group controller's joint ordering. Sim-only.
- [`src/hexa_bringup/config/ros2_controllers.yaml`](src/hexa_bringup/config/ros2_controllers.yaml) — real-robot mirror of the sim controllers config, with `use_sim_time: false` and the 100 Hz update rate the gait/IK stack publishes at.

## Design principles

1. **Modular** — one ROS2 package per concern, with a one-way dependency graph (no cycles).
2. **Configurable** — gait choice, body parameters, and leg geometry are config-driven, not hard-coded. Leg count is fixed at 6.
3. **Controllable from anywhere** — the top of the stack listens to a standard `geometry_msgs/Twist` on `cmd_vel`, so teleop, autonomy, or external controllers are interchangeable.
4. **Sim-first** — every package must be runnable against the Gazebo model before any servo moves.

## Packages

This is a colcon workspace; all ROS2 packages live under `src/`. Format: `src/<package>/` (build type) — purpose.

- `src/hexa_interfaces/` (interface) — Custom msg/srv/action definitions (LegState, GaitParams, FootTarget…) used across the stack.
- `src/hexa_description/` (ament_cmake) — URDF (via xacro), meshes, joint limits, robot_state_publisher config. Source of truth for kinematics.
- `src/hexa_kinematics/` (ament_python) — Pure-Python FK/IK library (per-leg + body); no ROS deps at the library level, plus a thin ROS node.
- `src/hexa_hardware/` (ament_cmake) — C++ `hardware_interface` plugin for ros2_control. Real Servo 2040 + sim/mock variants.
- `src/hexa_gait/` (ament_python) — Gait engine node; emits foot targets given a body velocity. Tripod first; wave/ripple plug in via a strategy.
- `src/hexa_posture/` (ament_python) — Posture engine node; turns user body-pose input + gait state into a clamped body pose target. Owns body-pose animations (sway, breathing, lean…).
- `src/hexa_control/` (ament_python) — Velocity shaping + gait selection: maps `cmd_vel` to gait params and chooses which gait runs.
- `src/hexa_teleop/` (ament_python) — Joystick/keyboard → `cmd_vel` and `/body/pose`.
- `src/hexa_simulation/` (ament_cmake) — Gazebo launch files, worlds, sim-only ros2_control config.
- `src/hexa_bringup/` (ament_cmake) — Top-level launch files wiring everything together: `robot.launch.py`, `sim.launch.py`.

## Package dependency direction

Each arrow is "depends on" — the higher-level package imports the lower-level one (or subscribes to its topics).

- Main chain: `hexa_teleop` → `hexa_control` → `hexa_gait` → `hexa_kinematics` → `hexa_hardware` → Servo 2040 / Gazebo
- Body-pose side channel: `hexa_teleop` → `hexa_posture` → `hexa_kinematics` (parallel to the gait chain, composed in the IK node)
- `hexa_bringup` → `hexa_control`, `hexa_posture` (composes both chains via launch files)
- Leaves consumed by the above: `hexa_description`, `hexa_interfaces`, `hexa_simulation`
