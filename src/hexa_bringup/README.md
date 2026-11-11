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

## Choosing the face

`config/face.yaml` has one setting, `backend`, that says which face drives the
OLED in `robot.launch.py`:

- **`none`** — no face node. This is the default.
- **`hexa_display`** — the 1-bit face (`hexa_display`). Its own `enabled` flag in
  `display.yaml` still applies.
- **`eyes_cpp`** — the 16-grey face (`hexa_eyes`), which takes its policy knobs
  and pins from `display.yaml` and its own from `eyes.yaml`.

The launch starts only the node you chose and never both; any other value
stops it with an error naming the valid ones. The sim launch is not affected: it
always runs `hexa_display` headless, with no panel to share.

`hexa_bringup/face_backend.py` holds the check; `test/test_face_backend.py`
covers it.
