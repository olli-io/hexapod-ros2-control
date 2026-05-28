# Hexapod

ROS2 control stack for a 6-leg / 18-DOF hexapod robot.

- **Hardware target**: Raspberry Pi 3 (Ubuntu Server 24.04, ARM64) driving a Pimoroni Servo 2040 over USB serial or I2C.
- **ROS2 distro**: Jazzy Jalisco (LTS, supported through 2029).
- **Simulator**: Gazebo Harmonic (paired with Jazzy, via `ros_gz`). The `hexa_hardware` package abstracts the servo bus so the same gait/control code runs in sim or on the real robot.
- **Dev environment**: Docker container (`./hexa --dev`), so the Arch / non-Ubuntu host doesn't need ROS2 installed. See [`docs/dev-environment.md`](docs/dev-environment.md).

## Build / run

Clone this repository: ``` git clone git@github.com/olli-io/hexapod ```

All commands run inside the dev container. From the repo root on the host:

```
./hexa --dev            # open interactive shell in the container
./hexa --dev --launch   # opens shell in the container and launches the desktop sim environment
```

Other arguments:
```
--clean                 # Rebuilds the container and hexapod nodes
--tmux                  # Same as --dev --launch but with a tmux split for convenience
```

Inside the container, the workspace CLI is `pod`:

```
pod build                    # colcon build --symlink-install
pod sim                      # ros2 launch hexa_bringup sim.launch.py
```

For the real robot, build and deploy the production image to a rPi 4 or 5 from the host workstation:

```
./hexa --prod build              # cross-build ARM64 image, save to .deploy/
./hexa --prod deploy pi@<host>   # ship the image and start the service (cold)
ssh pi@<host> 'cd ~/hexa-prod && ./hexa --prod engage'   # arm the servos
```

See [`docs/dev-environment.md`](docs/dev-environment.md) for the full `--prod` lifecycle, and [`docs/robot-environment.md`](docs/robot-environment.md) for preparing a fresh Pi to receive deploys.

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

## Runtime data flow

Body velocity (gait-driving) and body pose (positioning/animation) flow as two parallel signals. The gait engine consumes velocity; the IK node composes pose with foot targets. This keeps gait strategies pure `(phase, params) → foot_target` functions while still allowing the body to translate, yaw, or sway — both with feet grounded (pose mode) and during a walking gait (body animation).

Each step: producer — purpose — topic (message type) — consumer.

1. teleop / autonomy — publish body velocity — `/cmd_vel` (`geometry_msgs/Twist`) → `hexa_control`, `hexa_posture`
2. teleop / autonomy — publish user body pose offset — `/body/pose` (`hexa_interfaces/BodyPose`) → `hexa_posture`
3. `hexa_control` — select gait, shape velocity for current gait — `/gait/params` (`hexa_interfaces/GaitParams`) → `hexa_gait`
4. `hexa_posture` — compose user pose + animations (sway, breathing, lean…), clamp to envelope — `/body/pose_target` (`hexa_interfaces/BodyPose`) → `hexa_kinematics`
5. `hexa_gait` — per-leg phase + foot trajectory in nominal body frame — `/legs/targets` (`hexa_interfaces/LegState[6]`) → `hexa_kinematics`
5b. `hexa_gait` — current engine state (FOLDED, INITIALIZE, STAND, ENGAGING, GAIT, STOPPING, FOLDING) — `/gait/state` (`std_msgs/String`) → `hexa_posture` (gates body-pose application so the chassis can't be tilted while folded or mid-cold-start)
6. `hexa_kinematics` — compose pose target with foot targets, then IK: foot pose → 18 joint angles — `/joint_commands` (`sensor_msgs/JointState`) → `hexa_hardware`
7. `hexa_hardware` — ros2_control: joints → PWM → Servo 2040 (real) or Gazebo (sim)

### Pose mode vs gait-active

- **Pose mode** — `cmd_vel` is zero, gait is in `STAND` (see `hexa_gait` README). The foot targets emitted by the gait engine are constant (nominal stance). `hexa_posture` runs idle animations (e.g. breathing) and forwards the user's pose offset; the IK node re-solves joint angles against the held foot positions — the body translates/yaws/tilts relative to planted feet.
- **Gait-active body animation** — `cmd_vel` is non-zero. Foot targets sweep through swing/stance trajectories in the nominal body frame. `hexa_posture` runs gait-coupled animations (sway, lean, bob) on top of the user pose; the IK node composes the resulting offset with each frame's foot targets before solving. Gait strategies stay stateless and ignorant of body pose.

## Build / run

All commands run inside the dev container. From the repo root on the host:

```
./hexa --dev                 # interactive shell in the container
```

Then, inside the container:

```
# Build the workspace
colcon build --symlink-install
source install/setup.bash

# Simulated robot (no hardware required)
ros2 launch hexa_bringup sim.launch.py
```

For the real robot (RPi 3), build and deploy the production image from the host workstation:

```
./hexa --prod build              # cross-build ARM64 image, save to .deploy/
./hexa --prod deploy pi@<host>   # ship the image and start the service (cold)
ssh pi@<host> 'cd ~/hexa-prod && ./hexa --prod engage'   # arm the servos
```

See [`docs/dev-environment.md`](docs/dev-environment.md) for the full `--prod` lifecycle, and [`docs/robot-environment.md`](docs/robot-environment.md) for preparing a fresh Pi to receive deploys.

(Exact entrypoints filled in as packages are implemented. See [`docs/dev-environment.md`](docs/dev-environment.md) for the full container story.)
