# hexa_hardware

`ros2_control` hardware interface plugin. Translates the controller manager's
joint commands into PWM signals on the real robot, or into Gazebo joint
commands in simulation.

Two `SystemInterface` implementations:
- `hexa_hardware/servo2040_system.cpp` — talks to the Pimoroni Servo 2040
  (USB serial or I2C — protocol TBD; abstracted behind a small `ServoBus` class).
- `hexa_hardware/mock_system.cpp` — loopback for bench testing without
  hardware or simulator.

Gazebo's `gazebo_ros2_control` plugin is used in sim and is loaded via
`hexa_simulation`, so this package only owns the real-robot path.

Implemented in C++ because `hardware_interface` plugins are loaded via
pluginlib.
