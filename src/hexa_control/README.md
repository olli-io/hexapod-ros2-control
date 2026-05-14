# hexa_control

Body-level controller. Sits between high-level inputs (teleop, autonomy)
and the gait engine.

Responsibilities:
- Subscribe to `/cmd_vel` (`geometry_msgs/Twist`) and translate it into
  `GaitParams` for the gait engine.
- Subscribe to `/body/pose` (`hexa_interfaces/BodyPose`) for trim
  (height, roll/pitch/yaw offsets) and apply it to the gait engine's
  body frame.
- Decide *which* gait to use based on commanded speed (e.g. wave below a
  threshold, tripod above).

Does not own the gait phase or kinematics — it only shapes the inputs to
`hexa_gait`.
