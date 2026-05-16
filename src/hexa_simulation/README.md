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

## Run

From inside the dev container:

```
colcon build --symlink-install
source install/setup.bash
ros2 launch hexa_simulation sim.launch.py
```

The model spawns at `z=0.25` by default and falls onto the ground plane.
Send a joint group command to verify ros2_control is alive:

```
ros2 topic pub --once /joint_group_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0]}"
```

Joint order matches `config/ros2_controllers.yaml`:
`l_front`, `l_middle`, `l_rear`, `r_front`, `r_middle`, `r_rear`, each
`coxa, femur, tibia`.
