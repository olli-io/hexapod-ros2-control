# hexa_pico_bridge — Gazebo-in-the-loop firmware bridge

Plan part 10, **Tier 3**. Runs the Pico 2 W firmware's control brain against the
**simulated** hexapod, with no hardware. It links the target-agnostic firmware
pipeline sources (`pi-pico-firmware/src`) **directly** — the exact same float
source compiled for the RP2350 — and runs the real 200 Hz control tick, so the
firmware walks the Gazebo model. This honors the repo's **sim-first** rule:
every feature runs against the Gazebo model before any servo code is touched.

## What it is

The firmware factors its whole control brain — teleop mapping → velocity shaping
→ gait engine → posture → compose/IK, plus the failsafe supervisor — into one
Pico-SDK-free, ROS-free class, `hexa::pipeline::Pipeline` (see
`pi-pico-firmware/src/pipeline.hpp`). The only thing that differs between the Pico
firmware and this bridge is the **hardware seam**, swapped at link time (no
`#ifdef`s):

- **Input** — the Pico reads Bluepad32; the bridge subscribes `/joy`
  (`sensor_msgs/Joy`) and converts it into the exact raw int16 `axes[]` / button
  bitmask the firmware's `bt_teleop` emits, so `map_joy` runs identically. `/joy`
  is the layout the existing `joy_publisher` already produces
  (`hexa_teleop/config/teleop_joy.yaml` `base` block); the firmware applies the
  axis signs itself, so the bridge only rescales `[-1, 1]` → int16.
- **Output** — the Pico feeds `servo_out` (Chica UART → pulses); the bridge taps
  the pipeline at the `JointAngles` stage and publishes `std_msgs/Float64MultiArray`
  (radians) on `/joint_group_position_controller/commands` — the interface
  `gz_ros2_control` exposes. The firmware's joint order
  (`l_front,l_middle,l_rear,r_front,r_middle,r_rear` × `coxa,femur,tibia`) is
  **identical** to the controller's `joints:` list
  (`hexa_simulation/config/ros2_controllers.yaml`), so the array publishes with
  **no remap**.
- **Clock** — a `std::chrono` steady clock stands in for `time_us_64()`.

## Build

Inside the sim container (`./hexa sim`):

```sh
colcon build --packages-select hexa_pico_bridge
```

The build bakes `config_generated.hpp` from the repo YAMLs with the same
`tools/gen_config.py` the Pico and host builds use, so the sim brain runs the
identical constants (leg geometry, gait knobs, teleop mapping, posture stack).

## Run

Everything at once (sim + joy publisher + bridge):

```sh
ros2 launch hexa_pico_bridge bridge.launch.py
```

Or against an already-running sim (e.g. `pod sim` in another pane):

```sh
ros2 launch hexa_pico_bridge bridge.launch.py sim:=false
```

Then, on a connected gamepad (the same one `joy_publisher` reads):

- press **init** (start) → the robot stands up (FOLDED → INITIALIZE → STAND),
- drive with the sticks → it **walks the Gazebo hexapod**,
- pose mode tilts the body with feet planted; the gait cyclers change cadence.

Launch args: `sim` (default `true`), `joy` (default `true` — set `false` to feed
`/joy` yourself), `headless` (default `false`, forwarded to the sim).

**Cross-check (sim-first):** the same teleop input should produce motion visually
identical to the ROS2 node chain (`ik_node` + `gait_node` + `posture_node`)
running the same world — that chain and this bridge share the ported kinematics /
gait / posture math, one in `double` (ROS2) and one in `float` (firmware), pinned
to `≤1e-3 rad` by the host golden suites.

## Where the port risk is actually tested

The bridge is a thin ROS shell; the logic it runs is the firmware pipeline, which
is exercised off-target by the host harness (`pi-pico-firmware/test/host`) — the
golden-trace suites (`test_gait`, `test_kinematics`, `test_joy_mapping`,
`test_posture`) against the untouched `double` ROS2 engines, and `test_pipeline`,
which drives the whole composed brain (stand → walk → safe-stop). That harness is
the regression gate; the Gazebo run is the behavioral confirmation.

## Not covered here (Tier 4, optional/deferred)

A firmware-**binary** smoke sim (Wokwi / Renode running the actual RP2350 `.elf`)
would validate the Pico boot banner, USB-CDC `stdio`, the 200 Hz scheduler
cadence, and the Chica UART framing — the things a host/Gazebo run can't. It
needs the ARM build from part 01 and pinned `wokwi-cli` / Renode RP2350 support;
**Bluetooth / CYW43 cannot be simulated** (stub BT input). It is explicitly
optional in the plan and not implemented.
