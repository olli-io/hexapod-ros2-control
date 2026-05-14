# hexa_kinematics

Forward and inverse kinematics for the hexapod.

Two layers:
- **Library** (`hexa_kinematics/`): pure Python, no ROS imports. Per-leg
  3-DOF IK (coxa/femur/tibia) and body-frame transforms. Importable and
  unit-testable on its own.
- **Node** (`hexa_kinematics/ik_node.py`): subscribes to `/legs/targets`
  (`LegState[6]`), publishes `/joint_commands` (`sensor_msgs/JointState`).

Geometry parameters come from `hexa_description` (loaded at startup), so
the library doesn't hard-code leg dimensions.

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
  mathematical IK solution. Once `hexa_description` publishes
  `config/joint_limits.yaml`, `LegSpec` should grow `(min, max)` ranges
  per joint and IK should validate (raise or clip).
- **URDF joint-zero offsets.** The library uses kinematic zeros
  (horizontal femur, extended tibia); the URDF will likely pick a
  different reference pose so joint zero sits mid-range. The IK node
  must apply a per-joint offset map when publishing
  `sensor_msgs/JointState`.
