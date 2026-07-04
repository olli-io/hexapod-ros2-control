# hexa_pico_bridge — Gazebo-in-the-loop firmware seam smoke

Runs the Pico 2 W firmware's control brain against the **simulated** hexapod,
with no hardware. It links the target-agnostic control-brain sources
(`shared/motion_core`) **directly** — the exact same float source compiled for
the RP2350 — and runs the real 200 Hz control tick, so the firmware walks the
Gazebo model. This honors the repo's **sim-first** rule: every feature runs
against the Gazebo model before any servo code is touched.

## Why this exists (and what it is *not*)

The control brain — teleop mapping → velocity shaping → gait engine → posture →
compose/IK, plus the failsafe supervisor — is one Pico-SDK-free, ROS-free class,
`hexa::pipeline::Pipeline` (see `shared/motion_core/pipeline.hpp`). That brain
is shared **verbatim** with `hexa_locomotion`, which is the production sim
locomotion path and already runs it against Gazebo off `/cmd_vel`. So this bridge
is **not** where the gait/kinematics/posture math is validated — that is fully
shared code, exercised off-target by the host harness (see below) and demoed in
sim by `hexa_locomotion`.

What the bridge uniquely exercises are the two **firmware-specific seams**
`hexa_locomotion` deliberately bypasses, giving them an in-Gazebo smoke before
the Pico is flashed:

- **Input seam — `map_joy`.** The bridge subscribes `/joy` (`sensor_msgs/Joy`),
  converts it into the exact raw int16 `axes[]` / button bitmask the firmware's
  `bt_teleop` emits, and calls the pipeline's **joy overload**, so `map_joy` runs
  identically to on-hardware. `hexa_locomotion` instead builds a `CommandIntent`
  from `/cmd_vel` and calls the **core tick**, skipping `map_joy` entirely. `/joy`
  is the layout the existing `joy_publisher` already produces
  (`hexa_teleop/config/teleop_joy.yaml` `base` block); the firmware applies the
  axis signs itself, so the bridge only rescales `[-1, 1]` → int16.
- **Config seam — baked constexpr.** The build bakes `config_generated.hpp` from
  the repo YAMLs via `tools/gen_config.py` — the same constants the RP2350
  compiles. `hexa_locomotion` instead loads `geometry.yaml` + `tuning.yaml` at
  runtime (`pipeline_config_loader.cpp`).

The remaining seams are a straight tap of the shared pipeline, no
firmware-specific logic:

- **Output** — tap the pipeline at the `JointAngles` stage (before `to_pulse_us`,
  the Pico's `servo_out`) and publish `std_msgs/Float64MultiArray` (radians) on
  `/joint_group_position_controller/commands` — the interface `gz_ros2_control`
  exposes. The firmware's joint order
  (`l_front,l_middle,l_rear,r_front,r_middle,r_rear` × `coxa,femur,tibia`) is
  **identical** to the controller's `joints:` list
  (`hexa_simulation/config/ros2_controllers.yaml`), so the array publishes with
  **no remap**.
- **Clock** — the node clock (sim time under `use_sim_time`) stands in for
  `time_us_64()`, keeping the tick in lockstep with the `controller_manager` /
  Gazebo instead of drifting against the real-time factor on wall time.

## Build

Inside the sim container (`./hexa sim`):

```sh
colcon build --packages-select hexa_pico_bridge
```

The build bakes `config_generated.hpp` with the same `tools/gen_config.py` the
Pico and host builds use, so the sim brain runs the identical constants (leg
geometry, gait knobs, teleop mapping, posture stack).

## Run

Everything at once (sim + joy publisher + bridge) — from the host, this is
`./hexa pico up` (see the top-level README); directly, inside the container:

```sh
ros2 launch hexa_pico_bridge bridge.launch.py
```

Or against an already-running sim (e.g. `hexa sim up` in another pane):

```sh
ros2 launch hexa_pico_bridge bridge.launch.py sim:=false
```

Then, on a connected gamepad (the same one `joy_publisher` reads):

- press **init** (start) → the robot stands up (FOLDED → INITIALIZE → STAND),
- drive with the sticks → it **walks the Gazebo hexapod**,
- pose mode tilts the body with feet planted; the gait cyclers change cadence.

Launch args: `sim` (default `true`), `joy` (default `true` — set `false` to feed
`/joy` yourself), `headless` (default `false`, forwarded to the sim).

## Where the port risk is actually tested

The bridge is a thin ROS shell; the logic it runs is the firmware pipeline, which
is exercised off-target by the host harness (`shared/motion_core/test`) — the
golden-trace suites (`test_gait`, `test_kinematics`, `test_joy_mapping`,
`test_posture`) and `test_pipeline`, which drives the whole composed brain
(stand → walk → safe-stop). The two seams the bridge smokes in Gazebo also have
host coverage: `map_joy` in `test_joy_mapping`, and the baked config is
parity-tested field-by-field against the runtime YAML loader
(`hexa_locomotion/test/test_config_loader.cpp`). That harness is the regression
gate; the Gazebo run is the behavioral confirmation of the on-hardware input +
baked-config seams.

## Not covered here (optional/deferred)

A firmware-**binary** smoke sim (Wokwi / Renode running the actual RP2350 `.elf`)
would validate the Pico boot banner, USB-CDC `stdio`, the 200 Hz scheduler
cadence, and the Chica UART framing — the things a host/Gazebo run can't. It
needs an ARM build and pinned `wokwi-cli` / Renode RP2350 support;
**Bluetooth / CYW43 cannot be simulated** (stub BT input). Explicitly optional
and not implemented.
