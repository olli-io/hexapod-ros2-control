# hexa_control

Velocity shaping + gait selection. Sits between teleop / autonomy and
the gait engine.

Responsibilities:
- Subscribe to `/cmd_vel` (`geometry_msgs/Twist`) and translate it into
  `GaitParams` for the gait engine. Decide *which* gait to use based on
  commanded speed (e.g. wave below a threshold, tripod above).
- Shape velocity for the active gait (acceleration limits, deadband,
  per-gait speed caps).

Does **not** own body pose — that lives in `hexa_posture`, which runs
in parallel and publishes `/body/pose_target` directly to the IK node.
`hexa_control` may still read `/cmd_vel` and gait state, but the body
pose signal does not flow through here.
