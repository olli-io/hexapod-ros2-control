# hexapod-ros2-control

ROS 2 control stack for a 6-leg / 18-DOF hexapod robot.

> [!WARNING]
> **Work in progress.** APIs, configuration, and behavior may change without
> notice; some features are incomplete or untested.

Part of a multi-repo stack:
- [olli-io/hexapod-servo2040-driver](https://github.com/olli-io/hexapod-servo2040-driver) — Pimoroni Servo 2040 firmware.

## Hardware

- ROS2 : Raspberry Pi 4 or 5
- Pico firmware: Raspberry Pi Pico 2 W
- Pimoroni servo2040
- (Optional) 256×64 SH1122 OLED via SPI for eye animations.

## Quickstart (Gazebo)

Requires Docker + docker-compose and an X server on `$DISPLAY`. No native ROS 2 install needed.

```
git clone git@github.com:olli-io/hexapod-ros2-control.git
cd hexapod-ros2-control
./hexa sim up
```

The first run builds the `hexa-sim` image (a few minutes) and the workspace,
then brings the sim stack (sim + webteleop + teleop) up **detached**. Stream it
with `./hexa sim logs -f`, stop it with `./hexa sim down`.

Then drive the robot with an Xbox-style controller (wired or Bluetooth), or open
`{local-pc-ip}:8080` on a phone on the same network for web teleop.

Watch the face with `./hexa sim face` — a live terminal mirror of the eyes (the
sim has no OLED); press `q` to detach without stopping the stack.

> [!NOTE]
> Only tested on Arch Linux. Other Linux should be straightforward; macOS and Windows (WSL) may vary.

## Build / run

All host commands go through the `hexa` dispatcher in the repo root. The sim and
robot stacks each run as their container's PID 1, managed by `docker compose`, so
they share the same `up` / `down` lifecycle.

- `./hexa sim <up|down|logs|build|shell|face|status|cmd...>` — sim-container lifecycle. `up [--clean]` brings the stack up detached; `up --pico` runs the firmware-in-sim brain (the Pi Pico firmware walks the Gazebo hexapod) instead, mutually exclusive with the plain sim stack; `build` runs a colcon build in an ephemeral container; `shell` opens a ROS 2-sourced shell; `face` attaches the terminal eye emulator (a live mirror of the face, headless in sim; `q` to detach); any other command (e.g. `ros2 topic list`) runs one-off.
- `./hexa deploy <build|push <host>>` — cross-build and ship the production image to the robot.
- `./hexa robot <up|down|restart|boot|install-service|status|logs|shell>` — operate the robot container on the Pi (or `-H user@host` to target it remotely). `up` boots and energizes servos (teleop included); `down` is the safe-stop; `install-service` enables a systemd unit that runs `boot` (the unattended `up`) on every power-on.
- `./hexa kill` — stop and remove the sim + pico containers.

Details: [`docs/sim-environment.md`](docs/sim-environment.md) (Gazebo) and
[`docs/robot-environment.md`](docs/robot-environment.md) (Pi). See also `./hexa --help`.

## Configuration

Every tunable parameter lives in YAML under `config/` — never hard-coded in
nodes. Edit the YAML, rebuild (`./hexa sim build`), relaunch.

- [`hexa_description/config/geometry.yaml`](src/hexa_description/config/geometry.yaml) — single source of truth for the robot's shape: body dimensions, per-leg segment lengths / radii / masses, hip mounts, and per-joint-type servo center + travel / effort / velocity limits. Loaded into the URDF via xacro.
- [`hexa_description/config/tuning.yaml`](src/hexa_description/config/tuning.yaml) — consolidated node parameters (node-key source of truth): gait engine knobs, `cmd_vel` shaping, posture animation stack, and the standing pose (per-joint rest angles under `gait_node`).
- [`hexa_teleop/config/teleop_joy.yaml`](src/hexa_teleop/config/teleop_joy.yaml) — joystick mapping, deadband, posture↔gait toggle, and per-mode `cmd_vel` / posture limits.
- [`hexa_webteleop/config/webteleop.yaml`](src/hexa_webteleop/config/webteleop.yaml) — web teleop server + shared teleop mapping.
- [`hexa_description/config/hardware.yaml`](src/hexa_description/config/hardware.yaml) — Servo 2040 wiring, electrical clamps, ADC scales. Real-robot only.
- [`hexa_description/config/servo_calibration.yaml`](src/hexa_description/config/servo_calibration.yaml) — per-servo endpoint pulse-width calibration (`calibration_values`, indexed by pin). Real-robot only.
- [`hexa_display/config/display.yaml`](src/hexa_display/config/display.yaml) — face: enable switch, gait-state → expression map, gaze / battery knobs, and the SH1122 SPI/GPIO pins, render rate, headless switch.
- [`hexa_simulation/config/ros2_controllers.yaml`](src/hexa_simulation/config/ros2_controllers.yaml) and [`hexa_bringup/config/ros2_controllers.yaml`](src/hexa_bringup/config/ros2_controllers.yaml) — controller-manager rate and joint ordering, for sim and real-robot respectively.

## Design principles

1. **Modular** — one package per concern, one-way dependency graph (no cycles).
2. **Configurable** — gait, body, and geometry are config-driven. Leg count is fixed at 6.
3. **Controllable from anywhere** — the stack listens to a standard `geometry_msgs/Twist` on `cmd_vel`, so teleop, autonomy, and external controllers are interchangeable.
4. **Sim-first** — every package runs against the Gazebo model before any servo moves.

## Packages

Colcon workspace; all packages live under `src/`. Build type in parentheses.

- `hexa_interfaces` (interface) — custom msg/srv/action definitions used across the stack.
- `hexa_description` (ament_cmake) — URDF (via xacro), meshes, joint limits. Source of truth for kinematics.
- `hexa_hardware` (ament_cmake) — C++ `hardware_interface` plugin for ros2_control (real Servo 2040 + sim/mock).
- `hexa_locomotion` (ament_cmake) — the locomotion controller: one node running the whole velocity → gait → posture → compose/IK pipeline in a single 200 Hz loop. Compiles the shared control brain directly; publishes joint commands. Replaced the former `hexa_control`/`hexa_gait`/`hexa_posture`/`hexa_kinematics` node chain.
- `shared/motion_core` (source tree, not a package) — target-agnostic float control brain (`hexa::pipeline`/`gait`/`posture`/`control`/`supervisor`), compiled directly by the Pico firmware, `hexa_pico_bridge`, and `hexa_locomotion`. Host tests under `shared/motion_core/test/`.
- `shared/display_core` (source tree, not a package) — target-agnostic face policy: the pure expression/gaze policy (`hexa::display`) plus the vendored eye core (`core/`) and u8g2 C core (`u8g2/`), compiled directly by `hexa_display`, the Pico firmware, and the firmware host test (`pi-pico-firmware/test/host`), so the eyes rasterize bit-identically across targets.
- `hexa_teleop` (ament_python) — joystick/keyboard → `cmd_vel` and `/body/pose`.
- `hexa_webteleop` (ament_python) — HTTP + WebSocket server for phone/tablet control; arbitrates with the gamepad over `/teleop/owner`.
- `hexa_display` (ament_cmake) — face: maps robot state through an expression/gaze policy and rasterizes the eyes on a Pi-attached SH1122 OLED (headless in sim), in one process. Pure sink; nothing imports it. Owns the Linux SH1122 panel/SPI/GPIO driver + the rclcpp nodes; the shared policy + eye core live in `shared/display_core`.
- `hexa_simulation` (ament_cmake) — Gazebo launch files, worlds, sim-only ros2_control config.
- `hexa_bringup` (ament_cmake) — top-level launch files: `robot.launch.py`, `sim.launch.py`.

## Dependency direction

Each arrow is "depends on" — the higher-level package imports the lower one (or subscribes to its topics).

- Locomotion: `hexa_teleop` → `cmd_vel` (+ command topics) → `hexa_locomotion` → `/joint_group_position_controller/commands` → `hexa_hardware` → Servo 2040 / Gazebo. `hexa_locomotion` runs the whole velocity → gait → posture → compose/IK pipeline in-process (compiling `shared/motion_core`), so there is no separate gait/posture/kinematics node chain.
- Web teleop: `hexa_webteleop` → `hexa_teleop` (shared mapping) → `cmd_vel` / `/body/pose`
- `hexa_bringup` → `hexa_locomotion`, `hexa_display` (composes the launch)
- Face: `hexa_display` subscribes to `hexa_locomotion`'s `/gait/state` (+ hardware topics) and rasterizes the eyes on the SH1122 OLED in one process. Nothing else depends on it.
- Leaves: `hexa_description`, `hexa_interfaces`, `hexa_simulation`.
