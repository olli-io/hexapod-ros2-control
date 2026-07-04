# hexa_interfaces

Custom ROS2 message, service, and action definitions shared across the stack.

Lives at the bottom of the dependency graph — every other package depends on this, and it depends on nothing hexapod-specific.

Defined types:
- `msg/BodyPose.msg` — 6-DOF body pose offset (x/y/z + roll/pitch/yaw) from the nominal walking pose. Used both for pose mode (gait idle, feet grounded) and gait-active body animation. Published on `/body/pose` by `hexa_teleop` / `hexa_webteleop`; consumed by `hexa_locomotion` (pipeline pose input) and `hexa_display` (expression policy). See [`msg/BodyPose.msg`](msg/BodyPose.msg) for the full frame and rotation-order spec.

The former `GaitParams` / `LegState` / `LegTargets` messages were the on-the-wire currency of the old `hexa_control → hexa_gait → hexa_kinematics` node chain. That chain was consolidated into the single `hexa_locomotion` node, which passes the same data in-process as C++ structs, so those messages were removed.

Action/service definitions (e.g. `CalibrateServos.srv`) will be added as needed.
