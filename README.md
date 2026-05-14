# Hexapod

ROS2 control stack for a 6-leg / 18-DOF hexapod robot.

- **Hardware target**: Raspberry Pi 3 (Ubuntu Server 24.04, ARM64) driving a Pimoroni Servo 2040 over USB serial or I2C.
- **ROS2 distro**: Jazzy Jalisco (LTS, supported through 2029).
- **Simulator**: Gazebo Harmonic (paired with Jazzy, via `ros_gz`). The `hexa_hardware` package abstracts the servo bus so the same gait/control code runs in sim or on the real robot.
- **Dev environment**: Docker container (`./scripts/dev.sh`), so the Arch / non-Ubuntu host doesn't need ROS2 installed. See [`docs/dev-environment.md`](docs/dev-environment.md).

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
- `src/hexa_control/` (ament_python) — Body pose controller; translates `cmd_vel` and high-level pose commands into gait params + body trim.
- `src/hexa_teleop/` (ament_python) — Joystick/keyboard → `cmd_vel`.
- `src/hexa_simulation/` (ament_cmake) — Gazebo launch files, worlds, sim-only ros2_control config.
- `src/hexa_bringup/` (ament_cmake) — Top-level launch files wiring everything together: `robot.launch.py`, `sim.launch.py`.

## Package dependency direction

Each arrow is "depends on" — the higher-level package imports the lower-level one (or subscribes to its topics).

- Main chain: `hexa_teleop` → `hexa_control` → `hexa_gait` → `hexa_kinematics` → `hexa_hardware` → Servo 2040 / Gazebo
- `hexa_bringup` → `hexa_control` (composes the chain via launch files)
- Leaves consumed by the above: `hexa_description`, `hexa_interfaces`, `hexa_simulation`

## Runtime data flow

Each step: producer — purpose — topic (message type) — consumer.

1. teleop / autonomy — publish body velocity — `/cmd_vel` (`geometry_msgs/Twist`) → `hexa_control`
2. `hexa_control` — select gait, body height/tilt trim — `/gait/params` (`hexa_interfaces/GaitParams`) → `hexa_gait`
3. `hexa_gait` — per-leg phase + foot trajectory — `/legs/targets` (`hexa_interfaces/LegState[6]`) → `hexa_kinematics`
4. `hexa_kinematics` — IK: foot pose → 18 joint angles — `/joint_commands` (`sensor_msgs/JointState`) → `hexa_hardware`
5. `hexa_hardware` — ros2_control: joints → PWM → Servo 2040 (real) or Gazebo (sim)

## Build / run

All commands run inside the dev container. From the repo root on the host:

```
./scripts/dev.sh                    # interactive shell in the container
```

Then, inside the container:

```
# Build the workspace
colcon build --symlink-install
source install/setup.bash

# Simulated robot (no hardware required)
ros2 launch hexa_bringup sim.launch.py

# Real robot (RPi 3)
ros2 launch hexa_bringup robot.launch.py
```

(Exact entrypoints filled in as packages are implemented. See [`docs/dev-environment.md`](docs/dev-environment.md) for the full container story.)
