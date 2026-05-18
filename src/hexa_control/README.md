# hexa_control

Velocity shaping. Sits between teleop / autonomy and the gait engine.

v1 is a thin pass-through:

- Subscribe to `/cmd_vel` (`geometry_msgs/Twist`).
- Clamp linear x / y and angular z against the per-axis caps from
  `config/control.yaml`.
- Republish as `GaitParams` on `/gait/params` at 50 Hz, with
  `gait_name` and the walk-cycle knobs (`cycle_time`, `duty_factor`,
  `step_height`) read from the same YAML.

Out of scope for v1:

- **Acceleration limits / shaping.** Teleop already smooths input.
- **Deadband.** `hexa_teleop` zeroes stick noise at its boundary.
- **Multi-gait selection.** Only `tripod` ships in v1; selection by
  commanded speed (wave / ripple / tripod) drops in here once those
  strategies land in `hexa_gait`.

Does **not** own body pose — that lives in `hexa_posture`, which runs
in parallel and publishes `/body/pose_target` directly to the IK node.
`hexa_control` may still read `/cmd_vel` and gait state, but the body
pose signal does not flow through here.
