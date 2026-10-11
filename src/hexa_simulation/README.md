# hexa_simulation

Everything Gazebo-specific lives here, so the real-robot packages stay
sim-free.

## Contents

- `launch/sim.launch.py` — launches `gz_sim` with a world, includes
  `hexa_description`'s `description.launch.py` (with `use_sim:=true`),
  spawns the model from `/robot_description`, bridges `/clock` and the base
  IMU (gz `imu` → `/imu/data`, the topic `hexa_imu` publishes on the robot),
  and spawns the `joint_state_broadcaster` and `joint_group_position_controller`.
  The default world is `worlds/$HEXA_SIM_WORLD.sdf` (default `empty`), so
  `HEXA_SIM_WORLD=slope hexa sim up` loads the slope.
- `worlds/empty.sdf` — flat ground plane, sun, and the gz-sim systems
  needed for physics, sensors (IMU included), contacts, and
  `ros_gz_sim create`.
- `worlds/slope.sdf` — `empty.sdf` with the ground tilted 8° in roll and 5° in
  pitch, for body stabilization. `log/probe_level.py` compares the IMU tilt
  with leveling off, on and walking.
- `config/ros2_controllers.yaml` — `controller_manager` + controller
  parameters loaded by the `gz_ros2_control-system` plugin (the plugin
  tag itself lives in `hexa_description/urdf/hexapod.gazebo.xacro` where
  the URDF needs it). Declares `joint_state_broadcaster` and a single
  `JointGroupPositionController` covering all 18 joints.

The real-robot bringup never loads this package.
