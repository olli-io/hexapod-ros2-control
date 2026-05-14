# hexa_interfaces

Custom ROS2 message, service, and action definitions shared across the stack.

Lives at the bottom of the dependency graph — every other package depends on this, and it depends on nothing hexapod-specific.

Planned types (initial):
- `msg/LegState.msg` — foot target pose + phase (stance/swing) for one leg.
- `msg/GaitParams.msg` — gait selection, step height, cycle time, body velocity.
- `msg/BodyPose.msg` — body trim: height + roll/pitch/yaw offsets.

Action/service definitions (e.g. `CalibrateServos.srv`) will be added as needed.
