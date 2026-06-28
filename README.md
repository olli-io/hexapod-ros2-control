# hexapod-ros2-control

ROS 2 control stack for a 6-leg / 18-DOF hexapod robot.

> [!WARNING]
> **Work in progress** — This project is under active development. APIs,
> configuration, and behavior may change without notice, and some features are
> incomplete or untested. Use at your own risk.

**This repository is part of a multi-repo hexapod stack:**
- Driver firmware for Pimoroni servo2040 - ['olli-io/hexapod-servo2040-driver'](https://github.com/olli-io/hexapod-servo2040-driver)
- Esp32 firmware to drive an oled screen - ['olli-io/hexapod-esp32-display'](https://github.com/olli-io/hexapod-esp32-display)

## Hardware target

- Raspberry Pi 4 or 5 ( recommended OS: Pi OS lite ) driving servos over a Pimoroni Servo 2040 over USB serial. Current version tested on a 4GB rPi 4, but it may be runnable 2GB (no quarantees).
- (Optional) Xiao Seeed ESP32-C3 for driving a front display (eye animations).

## Quickstart ( Gazebo )

1. **Prerequisites**
   - Docker and docker-compose installed on the platform you are running this on.
   - An X server reachable as `$DISPLAY`. As an example, on Arch linux — `echo $DISPLAY` should print something like
     `:0`. No native ROS2 install needed.

2. **First-time setup and launch**

   ```
   git clone git@github.com:olli-io/hexapod-ros2-control.git
   cd hexapod-ros2-control
   ./hexa dev --launch
   ```

   The first run builds the `hexa-dev` image (a few minutes), creates a
   long-lived `hexa-dev` container, then opens a shell and launches the desktop
   sim environment (sim + webteleop + teleop in one pane). ROS2 is already
   sourced.

3. **Control the hex in sim**
   - Connect an XBox or XBox-equivalent controller to your setup (wired or bt) and control the hex.
   - Alternatively, you can take your phone (on the same local network) and connect to {local-pc-ip}:8080 and control the hexapod via webteleop (you may need to adjust your firewall settings).

That's the whole loop. See [`docs/dev-environment.md`](docs/dev-environment.md)
for the pieces.

> [!NOTICE]
> This has only been run on arch linux so far. Should be fairly straightforward on linux. MacOs and Windows (WSL) success may vary.

## Build / run

Development container ( simulation in Gazebo ) - [`docs/dev-environment.md`](docs/dev-environment.md).
Robot container ( build, deploy and run on rPi ) - [`docs/robot-environment.md`](docs/robot-environment.md).

All host-side commands go through the `hexa` dispatcher in the repo root:

- `./hexa dev` — drop into the ROS2 Jazzy dev container (`--clean` rebuilds it first, `--launch` runs the full sim stack instead of a shell).
- `./hexa dev --tmux` — two-pane tmux session sharing one dev container: pane 0 runs the full sim stack, pane 1 is an idle shell.
- `./hexa prod <subcommand>` — cross-build, deploy, and operate the production image on the robot.
- `./hexa kill` — stop and remove a running dev container.

See docs and `./hexa --help` 

## Configuration

All tunable parameters live in YAML files under each package's `config/` directory — never hard-coded in node code. Edit the YAML, rebuild (`pod build` re-links instantly), relaunch.

- [`src/hexa_description/config/geometry.yaml`](src/hexa_description/config/geometry.yaml) — body dimensions, leg segment lengths / radii / masses, foot, per-leg hip mounts, and per-joint-type (coxa / femur / tibia) servo center, lower / upper travel limits, effort, and velocity. Single source of truth for the robot's shape and joint travel; loaded into the URDF via xacro.
- [`src/hexa_description/config/standing_pose.yaml`](src/hexa_description/config/standing_pose.yaml) — per-joint angles (coxa / femur / tibia) at rest. Drives nominal foot targets via FK; kept separate from servo center so an asymmetric build can diverge.
- [`src/hexa_teleop/config/teleop_joy.yaml`](src/hexa_teleop/config/teleop_joy.yaml) — joystick axis / button mapping, deadband, posture↔gait toggle button, initial mode, and the max `cmd_vel` and posture offsets each mode emits.
- [`src/hexa_control/config/control.yaml`](src/hexa_control/config/control.yaml) — default gait selection and `cmd_vel` ramp / snap tolerances used to shape teleop input before it hits the gait engine.
- [`src/hexa_gait/config/gait.yaml`](src/hexa_gait/config/gait.yaml) — gait engine knobs: controller tick, default gait, stride length, step height, swing width, and swing-time bounds that anchor per-gait cycle-time limits.
- [`src/hexa_posture/config/posture.yaml`](src/hexa_posture/config/posture.yaml) — posture node animation stack: which gait-coupled and animation-mode animations are enabled and their gain / strength / amplitude knobs.
- [`src/hexa_hardware/config/hardware.yaml`](src/hexa_hardware/config/hardware.yaml) — Servo 2040 wiring (transport, device, per-pin joint assignment), pulse-width calibration endpoints, electrical clamps, and aux ADC scales. Real-robot only.
- [`src/hexa_display/config/display.yaml`](src/hexa_display/config/display.yaml) — face relay: `enabled` master switch (false = bringup skips the display node), transport (serial/stub), UART device + baud, gait-state → expression map, battery thresholds, and gaze deadband / hysteresis knobs.
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
- `src/hexa_gait/` (ament_python) — Gait engine node; emits foot targets given a body velocity. Tripod first; ripple/crawl plug in via a strategy.
- `src/hexa_posture/` (ament_python) — Posture engine node; turns user body-pose input + gait state into a clamped body pose target. Owns body-pose animations (sway, breathing, lean…).
- `src/hexa_control/` (ament_python) — Velocity shaping + gait selection: maps `cmd_vel` to gait params and chooses which gait runs.
- `src/hexa_teleop/` (ament_python) — Joystick/keyboard → `cmd_vel` and `/body/pose`.
- `src/hexa_webteleop/` (ament_python) — Web-app teleop: hosts an HTTP + WebSocket server for phone/tablet control, publishing the same topics as `hexa_teleop` via a shared mapping; arbitrates with the gamepad over `/teleop/owner`.
- `src/hexa_display/` (ament_python) — Face relay: maps gait state / `cmd_vel` / posture / battery to expression + gaze commands for the ESP32 OLED face over UART (stub transport in sim).
- `src/hexa_simulation/` (ament_cmake) — Gazebo launch files, worlds, sim-only ros2_control config.
- `src/hexa_bringup/` (ament_cmake) — Top-level launch files wiring everything together: `robot.launch.py`, `sim.launch.py`.

## Package dependency direction

Each arrow is "depends on" — the higher-level package imports the lower-level one (or subscribes to its topics).

- Main chain: `hexa_teleop` → `hexa_control` → `hexa_gait` → `hexa_kinematics` → `hexa_hardware` → Servo 2040 / Gazebo
- Body-pose side channel: `hexa_teleop` → `hexa_posture` → `hexa_kinematics` (parallel to the gait chain, composed in the IK node)
- Web teleop: `hexa_webteleop` → `hexa_teleop` (reuses its mapping) → `cmd_vel` / `/body/pose` (interchangeable with the gamepad at the top of both chains)
- `hexa_bringup` → `hexa_control`, `hexa_posture`, `hexa_display` (composes both chains via launch files)
- Sink: `hexa_display` subscribes to gait/posture/hardware topics; nothing depends on it.
- Leaves consumed by the above: `hexa_description`, `hexa_interfaces`, `hexa_simulation`
