# hexapod_simulation

Everything Gazebo-specific lives here, so the real-robot packages stay
sim-free.

Contents (to be added):
- `launch/gazebo.launch.py` — spawns Gazebo with the world and robot URDF.
- `worlds/empty.world`, `worlds/rough.world` — flat ground + a heightmap
  for gait stress-testing.
- `config/gazebo_controllers.yaml` — ros2_control config wiring the
  `gazebo_ros2_control` plugin to the joints declared in
  `hexapod_description`.

The real-robot bringup never loads this package.
