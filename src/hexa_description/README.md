# hexa_description

Robot description package and the single source of truth for the robot's
geometry, joint limits, tuning and servo wiring. Nothing else duplicates these
values. Every consumer loads them at runtime, or generates code from them.

## Contents

- `urdf/hexapod.urdf.xacro` — the robot model. Reads all dimensions and joint
  limits from `config/geometry.yaml` via `xacro.load_yaml`. No mesh files; links
  are primitives.
- `urdf/hexapod.gazebo.xacro` — Gazebo overlay (materials, foot friction).
  Included only with `use_sim:=true`, so it never reaches the real robot.
- `launch/description.launch.py` — runs `robot_state_publisher` and publishes
  the URDF on `/robot_description`. Arguments: `use_sim`, `use_sim_time`.
- `config/geometry.yaml` — body and mount dimensions, plus a `legs:` block per
  leg group (`front`, `middle`, `rear`) with segment lengths, radii, masses and
  the `joints:` travel window. Left and right legs of a group share the block.
  `folded_pose` and `initialized_pose` give one entry per left leg; the right
  leg mirrors the coxa. Angles are in intuitive per-joint degrees. Conversion to
  IK-convention radians happens at load time.
- `config/tuning.yaml` — gait, control, posture and teleop tuning, including
  the `presets` list and `default_standing_pose`. Shared by sim, web teleop and
  the Pico firmware.
- `config/gestures.yaml` — gestures: keyframed leg + body motions played from
  a stand. Leg keyframes as joint angles in geometry.yaml's joint sense, body
  keyframes as pose offsets, written for the default preset's stance. The file
  header documents the format.
- `config/hardware.yaml` — Servo2040 connection, servo pin wiring, direction,
  `deg_at_center`, and the undervoltage ladder.
- `config/servo_calibration.yaml` — per-servo endpoint pulse widths, one
  entry per Servo2040 pin. Kept separate so a tool can rewrite it.

## Consumers

- `hexa_locomotion` — loads `geometry.yaml` + `tuning.yaml` + `gestures.yaml`
  into a `PipelineConfig` at startup.
- `shared/motion_core/tools/gen_config.py` — bakes all five files into the
  Pico firmware's constexpr config. A parity test keeps the two paths equal.
- `hexa_hardware` — reads `hardware.yaml` and `servo_calibration.yaml`.
- `hexa_teleop`, `hexa_webteleop`, `hexa_common` — read preset ids and limits
  from `tuning.yaml` and `geometry.yaml`.
- `hexa_simulation`, `hexa_bringup` — include the launch file.

## Editing

Bad values can damage the robot. Nothing validates them. After an edit: sim
`hexa sim restart`; robot `hexa deploy` then `hexa robot restart`; Pico reflash.
