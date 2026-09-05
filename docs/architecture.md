# Architecture

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
- `hexa_buttons` (ament_python) — front-panel GPIO buttons: battery/address and Bluetooth screens on the face's panel, and the pairing-scan request behind them. Event-driven via gpiozero on a pinned lgpio pin factory (no polling). Reaches the face over `/display/text` and `/bluetooth/scanning` rather than importing it. Real-robot only.
- `hexa_buzzer` (ament_python) — passive buzzer on the Pi's hardware PWM, the robot's only audible channel. Plays the tune an event named on `/buzzer/play` maps to (`config/buzzer.yaml`'s `events:` into `config/tunes.yaml`) straight onto the sysfs PWM channel bind-mounted at `/pwm`. Pure sink; nothing imports it. Its `tunes`/`catalog`/`pwm`/`player` modules are stdlib-only so the host's boot and shutdown systemd units run the same player without a ROS environment. Real-robot only.
- `hexa_simulation` (ament_cmake) — Gazebo launch files, worlds, sim-only ros2_control config.
- `hexa_bringup` (ament_cmake) — top-level launch files: `robot.launch.py`, `sim.launch.py`.

## Dependency direction

Each arrow is "depends on" — the higher-level package imports the lower one (or subscribes to its topics).

- Locomotion: `hexa_teleop` → `cmd_vel` (+ command topics) → `hexa_locomotion` → `/joint_group_position_controller/commands` → `hexa_hardware` → Servo 2040 / Gazebo. `hexa_locomotion` runs the whole velocity → gait → posture → compose/IK pipeline in-process (compiling `shared/motion_core`), so there is no separate gait/posture/kinematics node chain.
- Web teleop: `hexa_webteleop` → `hexa_teleop` (shared mapping) → `cmd_vel` / `/body/pose`
- `hexa_bringup` → `hexa_locomotion`, `hexa_display`, `hexa_buttons`, `hexa_buzzer` (composes the launch)
- Face: `hexa_display` subscribes to `hexa_locomotion`'s `/gait/state` (+ hardware topics) and rasterizes the eyes on the SH1122 OLED in one process. Nothing else depends on it.
- Leg set: `hexa_locomotion` also publishes `/gait/leg_set` (`std_msgs/String`, latched, `hexapod` | `quadruped`) — the set the engine has **applied**. Report only; the leg set is still commanded by naming a gait on `/cmd_gait`. It exists because that command topic is latched, so a request the engine refuses stays on it and a UI reading it would show a leg set the robot never took. Both teleops read it; the face does not.
- Buttons: `hexa_buttons` → `/display/text` + `/bluetooth/scanning` → `hexa_display`. One-way and topic-only, so the face stays a pure sink; a future Bluetooth scanning utility plugs in at `/bluetooth/status`.
- Buzzer: `hexa_hardware` → `/buzzer/play` → `hexa_buzzer`. One-way and topic-only, same shape as the face. The `boot` and `shutdown` tunes are host systemd units instead, because no container is running that early or that late.
- Leaves: `hexa_description`, `hexa_interfaces`, `hexa_simulation`.
