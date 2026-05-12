# hexapod_kinematics

Forward and inverse kinematics for the hexapod.

Two layers:
- **Library** (`hexapod_kinematics/`): pure Python, no ROS imports. Per-leg
  3-DOF IK (coxa/femur/tibia) and body-frame transforms. Importable and
  unit-testable on its own.
- **Node** (`hexapod_kinematics/ik_node.py`): subscribes to `/legs/targets`
  (`LegState[6]`), publishes `/joint_commands` (`sensor_msgs/JointState`).

Geometry parameters come from `hexapod_description` (loaded at startup), so
the library doesn't hard-code leg dimensions.
