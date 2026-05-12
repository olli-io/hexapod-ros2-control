# Hexapod

ROS2 control stack for a 6-leg / 18-DOF hexapod robot.

- **Hardware target**: Raspberry Pi 3 (Ubuntu Server 22.04, ARM64) driving a Pimoroni Servo 2040 over USB serial or I2C.
- **ROS2 distro**: Humble Hawksbill (LTS).
- **Simulator**: Gazebo Classic (paired with Humble). The `hexapod_hardware` package abstracts the servo bus so the same gait/control code runs in sim or on the real robot.

## Design principles

1. **Modular** — one ROS2 package per concern, with a one-way dependency graph (no cycles).
2. **Scalable** — gait choice, body parameters, and leg count/geometry are config-driven, not hard-coded.
3. **Controllable from anywhere** — the top of the stack listens to a standard `geometry_msgs/Twist` on `cmd_vel`, so teleop, autonomy, or external controllers are interchangeable.
4. **Sim-first** — every package must be runnable against the Gazebo model before any servo moves.

## Workspace layout

This is a colcon workspace. All ROS2 packages live under `src/`.

```
src/
├── hexapod_interfaces/   custom msg/srv/action types (LegState, GaitParams, FootTarget…)
├── hexapod_description/  URDF/xacro, meshes, robot_state_publisher config
├── hexapod_kinematics/   forward/inverse kinematics library (per-leg + body)
├── hexapod_hardware/     ros2_control hardware_interface plugin (real Servo 2040 + sim)
├── hexapod_gait/         gait engine — emits foot targets given a body velocity
├── hexapod_control/      body pose controller; cmd_vel → gait params + body trim
├── hexapod_teleop/       joystick / keyboard teleop publishing cmd_vel
├── hexapod_simulation/   Gazebo launch files, worlds, sim-only configs
└── hexapod_bringup/      top-level launch files wiring everything together
```

## Package dependency direction

```
hexapod_bringup ──────────────┐
                              ▼
hexapod_teleop ──▶ hexapod_control ──▶ hexapod_gait ──▶ hexapod_kinematics ──▶ hexapod_hardware
                                                                                      │
                                                                                      ▼
                                                                              Servo 2040 / Gazebo

hexapod_description, hexapod_interfaces, hexapod_simulation are leaves consumed by the above.
```

Each arrow is "depends on" — the higher-level package imports the lower-level one (or subscribes to its topics).

## Runtime data flow

```
       teleop / autonomy
              │  geometry_msgs/Twist on /cmd_vel
              ▼
       hexapod_control            ── selects gait, body height/tilt trim
              │  hexapod_interfaces/GaitParams on /gait/params
              ▼
       hexapod_gait               ── per-leg phase + foot trajectory
              │  hexapod_interfaces/LegState[6] on /legs/targets
              ▼
       hexapod_kinematics         ── IK: foot pose → 18 joint angles
              │  sensor_msgs/JointState on /joint_commands
              ▼
       hexapod_hardware           ── ros2_control: joints → PWM
              │
     ┌────────┴────────┐
     ▼                 ▼
  Servo 2040        Gazebo
   (real)            (sim)
```

## Packages

| Package              | Type        | Purpose |
|----------------------|-------------|---------|
| `hexapod_interfaces` | interface   | Custom message/service/action definitions used across the stack. |
| `hexapod_description`| ament_cmake | URDF (via xacro), meshes, joint limits. Source of truth for kinematics. |
| `hexapod_kinematics` | ament_python| Pure-Python FK/IK; no ROS deps at the library level, plus a thin ROS node. |
| `hexapod_hardware`   | ament_cmake | C++ `hardware_interface` plugin for ros2_control. Real + mock variants. |
| `hexapod_gait`       | ament_python| Gait engine node. Tripod first; wave/ripple plug in via a strategy. |
| `hexapod_control`    | ament_python| Translates `cmd_vel` and high-level pose commands into gait parameters. |
| `hexapod_teleop`     | ament_python| Joystick/keyboard → `cmd_vel`. |
| `hexapod_simulation` | ament_cmake | Gazebo launch, worlds, sim-only ros2_control config. |
| `hexapod_bringup`    | ament_cmake | Composite launch files: `robot.launch.py`, `sim.launch.py`. |

## Build / run

```
# Build the workspace
cd ~/git/hexapod
colcon build --symlink-install
source install/setup.bash

# Simulated robot (no hardware required)
ros2 launch hexapod_bringup sim.launch.py

# Real robot (RPi 3)
ros2 launch hexapod_bringup robot.launch.py
```

(Exact entrypoints filled in as packages are implemented.)
