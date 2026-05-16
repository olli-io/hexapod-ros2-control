# hexa_interfaces

Custom ROS2 message, service, and action definitions shared across the stack.

Lives at the bottom of the dependency graph — every other package depends on this, and it depends on nothing hexapod-specific.

Defined types:
- `msg/BodyPose.msg` — 6-DOF body pose offset (x/y/z + roll/pitch/yaw) from the nominal walking pose. Used both for pose mode (gait idle, feet grounded) and gait-active body animation. See [`msg/BodyPose.msg`](msg/BodyPose.msg) for the full frame and rotation-order spec.
- `msg/LegState.msg` — foot target (in the nominal body frame) plus gait phase and stance/swing flag for one leg.
- `msg/LegTargets.msg` — wrapper carrying a `LegState[6]` array; published by hexa_gait on `/legs/targets` and consumed by hexa_kinematics.

Planned types (add as producer + consumer land together):
- `msg/GaitParams.msg` — gait selection, step height, cycle time, body velocity.

Action/service definitions (e.g. `CalibrateServos.srv`) will be added as needed.
