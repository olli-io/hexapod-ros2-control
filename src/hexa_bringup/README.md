# hexa_bringup

Top-level launch files. No code of its own — just composition.

- `launch/sim.launch.py` — full simulated stack: Gazebo + ros2_control +
  kinematics + gait + control + posture + teleop.
- `launch/robot.launch.py` — real-robot stack: hexa_hardware (Servo 2040)
  + ros2_control + kinematics + gait + control + posture + teleop, plus the
  optional display, buttons, buzzer and `hexa_imu` nodes, each gated on the
  `enabled` flag in its package YAML.
- `launch/components/` — reusable sub-launches included by both above.

Parameters (gait selection, body geometry, joystick mapping) come from
YAML files in `config/`.
