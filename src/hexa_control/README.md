# hexa_control

Velocity shaping. Sits between teleop / autonomy and the gait engine.

v1 is a thin pass-through:

- Subscribe to `/cmd_vel` (`geometry_msgs/Twist`).
- Clamp linear x / y and angular z against the velocity caps loaded at
  startup from `hexa_gait/config/gait.yaml` via
  `hexa_gait.load_velocity_caps`. `control.yaml` only owns `gait_name`.
- Republish as `GaitParams` on `/gait/params` at 50 Hz, with `gait_name`
  read from the local YAML and only the commanded velocity on the wire.

Walk-cycle knobs (`stride_length`, `min_cycle_time`, `max_cycle_time`,
`duty_factor`, `step_height`) and the velocity caps live in
`hexa_gait/config/gait.yaml` — the single source of truth.
`cycle_time` is derived inside the gait engine each tick from the
commanded velocity — it is not configured here.

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
