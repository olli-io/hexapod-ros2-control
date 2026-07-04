# hexa_simulation

Everything Gazebo-specific lives here, so the real-robot packages stay
sim-free.

## Contents

- `launch/sim.launch.py` — launches `gz_sim` with a world, includes
  `hexa_description`'s `description.launch.py` (with `use_sim:=true`),
  spawns the model from `/robot_description`, bridges `/clock`, and
  spawns the `joint_state_broadcaster` and `joint_group_position_controller`.
- `worlds/empty.sdf` — flat ground plane, sun, and the gz-sim systems
  needed for physics, sensors, contacts, and `ros_gz_sim create`. A
  rougher terrain world for gait stress-testing will land alongside this
  one when gait development begins.
- `config/ros2_controllers.yaml` — `controller_manager` + controller
  parameters loaded by the `gz_ros2_control-system` plugin (the plugin
  tag itself lives in `hexa_description/urdf/hexapod.gazebo.xacro` where
  the URDF needs it). Declares `joint_state_broadcaster` and a single
  `JointGroupPositionController` covering all 18 joints.

The real-robot bringup never loads this package.
