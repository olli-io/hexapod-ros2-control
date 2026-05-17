# hexa_kinematics

Forward and inverse kinematics for the hexapod.

Two layers:
- **Library** (`hexa_kinematics/`): pure Python, no ROS imports. Per-leg
  3-DOF IK (coxa/femur/tibia), body-frame transforms, and body-pose
  composition (`apply_body_pose`). Importable and unit-testable on its
  own.
- **Node** (`hexa_kinematics/ik_node.py`): subscribes to `/legs/targets`
  (`LegState[6]`) **and** `/body/pose_target` (`BodyPose`), composes the
  pose offset with each foot target, then publishes `/joint_commands`
  (`sensor_msgs/JointState`).

Geometry parameters come from `hexa_description` (loaded at startup), so
the library doesn't hard-code leg dimensions.

## Body pose composition

The IK node holds the latest `BodyPose` from `/body/pose_target`
(published by `hexa_posture`, defaulting to identity) and applies it to
every incoming foot target before solving IK. This is the single
composition point in the stack — gait strategies stay pure and unaware
of body pose, and pose mode (feet grounded, gait idle) and gait-active
body animation share the exact same code path: the gait engine emits
foot targets in the nominal body frame, and the IK node re-expresses
them in the offset body frame via `apply_body_pose`.

`/body/pose_target` is latched-style (the IK node uses the most recent
sample); foot targets drive the publish rate of `/joint_commands`. The
node does **not** subscribe to `/body/pose` directly — the user input
is shaped, animated, and clamped by `hexa_posture` first.

## Conventions

Frame, joint-angle, and branch conventions are documented in
`hexa_kinematics/leg_geometry.py`. The headlines:

- Coxa-mount frame is right-handed (REP-103): `+x` radially outward from
  the body, `+y` left, `+z` up.
- Joint zero positions: `θ_femur = 0` puts the femur horizontal,
  `θ_tibia = 0` puts the tibia colinear with the femur (leg fully
  extended).
- `inverse_kinematics` returns the **knee-up** branch (knee on the
  upper-z side of the chord from femur joint to foot).

## Known gaps

Tracked for when the IK node lands:

- **Joint limits are not enforced.** The library returns the unconstrained
  mathematical IK solution. Per-joint-type limits are now loadable via
  `joint_config.load_joint_limits` (reading the `joints:` block of
  `hexa_description/config/geometry.yaml`); wire them into `LegSpec`
  and have IK validate (raise or clip).
- **URDF joint-zero offsets.** The library uses kinematic zeros
  (horizontal femur, extended tibia); the URDF will likely pick a
  different reference pose so joint zero sits mid-range. The IK node
  must apply a per-joint offset map when publishing
  `sensor_msgs/JointState`.
