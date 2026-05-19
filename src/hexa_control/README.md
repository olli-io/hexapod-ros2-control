# hexa_control

Velocity shaping. Sits between teleop / autonomy and the gait engine.

Pass-through with two side jobs:

- Subscribe to `/cmd_vel` (`geometry_msgs/Twist`).
- Clamp linear x / y and angular z against the velocity caps loaded at
  startup from `hexa_gait/config/gait.yaml` via
  `hexa_gait.load_velocity_caps`. `control.yaml` owns `default_gait`
  only.
- Subscribe to `/cmd_gait` (`std_msgs/String`, `transient_local`
  durability) — the active gait selection from teleop / autonomy.
  Validates the name against `hexa_gait.gaits.STRATEGIES`; unknown
  names are warned and dropped.
- Republish as `GaitParams` on `/gait/params` at 50 Hz with the active
  gait name in `GaitParams.gait_name` and the shaped velocity in
  `linear_*` / `angular_z`.

Walk-cycle knobs (`stride_length`, `min_swing_time`, `max_cycle_time`,
per-gait `duty_factor`, `step_height`) and the velocity caps live in
`hexa_gait/config/gait.yaml` — the single source of truth.
`cycle_time` is derived inside the gait engine each tick from the
commanded velocity — it is not configured here.

Out of scope:

- **Acceleration limits / shaping.** Teleop already smooths input.
- **Deadband.** `hexa_teleop` zeroes stick noise at its boundary.
- **Speed-based gait selection.** Selection is operator-driven via the
  D-pad cycler; an autonomy node could publish `/cmd_gait` directly
  with the same contract.

Does **not** own body pose — that lives in `hexa_posture`, which runs
in parallel and publishes `/body/pose_target` directly to the IK node.
`hexa_control` may still read `/cmd_vel` and gait state, but the body
pose signal does not flow through here.
