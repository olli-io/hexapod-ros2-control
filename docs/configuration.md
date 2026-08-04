# Configuration

Every tunable parameter lives in YAML under `config/` — never hard-coded in
nodes. Edit the YAML, rebuild (`./hexa sim build`), relaunch.

## Main config files

- [`hexa_description/config/geometry.yaml`](../src/hexa_description/config/geometry.yaml) — single source of truth for the robot's shape: body dimensions, per-leg segment lengths / radii / masses, hip mounts, and per-joint-type servo center + travel / effort / velocity limits. Loaded into the URDF via xacro.
- [`hexa_description/config/tuning.yaml`](../src/hexa_description/config/tuning.yaml) — consolidated node parameters (node-key source of truth): gait engine knobs, `cmd_vel` shaping, posture animation stack, and the default standing pose (per-leg-group foot placement under `gait_node`).
- [`hexa_description/config/hardware.yaml`](../src/hexa_description/config/hardware.yaml) — Servo 2040 wiring, electrical clamps, ADC scales. Real-robot only.
- [`hexa_description/config/servo_calibration.yaml`](../src/hexa_description/config/servo_calibration.yaml) — per-servo endpoint pulse-width calibration (`calibration_values`, indexed by pin). Real-robot only.

## Secondary config files

- [`hexa_teleop/config/teleop_joy.yaml`](../src/hexa_teleop/config/teleop_joy.yaml) — joystick mapping, deadband, posture↔gait toggle, and per-mode `cmd_vel` / posture limits.
- [`hexa_webteleop/config/webteleop.yaml`](../src/hexa_webteleop/config/webteleop.yaml) — web teleop server + shared teleop mapping.
- [`hexa_display/config/display.yaml`](../src/hexa_display/config/display.yaml) — face: enable switch, gait-state → expression map, gaze / battery knobs, and the SH1122 SPI/GPIO pins, render rate, headless switch.
- [`hexa_buttons/config/buttons.yaml`](../src/hexa_buttons/config/buttons.yaml) — front-panel GPIO buttons: enable switch, line numbers and wiring polarity, debounce / hold / timeout clocks, and the pack voltage → percentage span shown on the battery screen. Real-robot only.
- [`hexa_buzzer/config/buzzer.yaml`](../src/hexa_buzzer/config/buzzer.yaml) — buzzer: enable switch, the PWM device path and channel, how long to wait for the PWM driver to probe, and the `events:` map from an event on `/buzzer/play` to a tune below. Real-robot only.
- [`hexa_buzzer/config/tunes.yaml`](../src/hexa_buzzer/config/tunes.yaml) — the named tones themselves, as RTTTL strings. Read by both the node and the host's boot/shutdown player. No tempo setting — each tune carries its own in its string. Real-robot only.
- [`hexa_simulation/config/ros2_controllers.yaml`](../src/hexa_simulation/config/ros2_controllers.yaml) and [`hexa_bringup/config/ros2_controllers.yaml`](../src/hexa_bringup/config/ros2_controllers.yaml) — controller-manager rate and joint ordering, for sim and real-robot respectively.
