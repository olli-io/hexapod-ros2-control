# hexa_pico_bridge

Runs the Pico firmware's control brain (`shared/motion_core`, the same float
source compiled for the RP2350) against the Gazebo hexapod, no hardware.

`hexa_locomotion` already runs that brain in sim off `/cmd_vel`. The bridge
exists to smoke the two **firmware-specific seams** it bypasses:

- **Input — `map_joy`.** Subscribes `/joy`, rescales it to the int16 axes /
  button bitmask `bt_teleop` emits, and calls the pipeline's joy overload.
- **Config — baked constexpr.** The build bakes `config_generated.hpp` from the
  repo YAMLs via `tools/gen_config.py`, as the Pico does, instead of loading
  YAML at runtime.

Output taps the pipeline at `JointAngles` (before `to_pulse_us`) and publishes
`Float64MultiArray` on `/joint_group_position_controller/commands`; the joint
order matches `ros2_controllers.yaml`, no remap. The node clock (sim time) stands
in for `time_us_64()`.

## Run

```sh
./hexa pico up                                          # sim + joy + bridge
ros2 launch hexa_pico_bridge bridge.launch.py sim:=false   # against a running sim
```

Launch args: `sim`, `joy` (false to feed `/joy` yourself), `headless`.

On the gamepad: **start** stands the robot up, sticks walk it.

## Tests

The logic is the shared pipeline, covered off-target by
`shared/motion_core/test` (golden traces + `test_pipeline`). `map_joy` is covered
by `test_joy_mapping`; the baked config is parity-tested against the runtime
loader in `hexa_locomotion/test/test_config_loader.cpp`. The Gazebo run is the
behavioral confirmation only.

A firmware-binary sim (Wokwi / Renode) for boot, USB stdio and UART framing is
not implemented; Bluetooth cannot be simulated.
