# Part 10 — Linux build & simulator testing

**Goal:** build and exercise the firmware on Linux with **no hardware** — the
pure logic as native unit / golden tests, and the full in-process pipeline
driving the existing **Gazebo** hexapod (sim-first). This is the desktop test
loop every earlier part leans on before flashing.

**Depends on:** layered — Tier 1 rides on 04; Tier 3 needs the pipeline through
07/08. **Blocks:** nothing (it is the safety net, not a feature).

## The constraint that shapes everything

There is **no full-system emulator for the RP2350 + CYW43439 Bluetooth radio**.
You cannot simulate Bluepad32 pairing or a paired gamepad. So Linux testing is
**layered**: exercise the *logic* natively (where essentially all the port risk
lives — float conversion, gait, IK, posture, teleop mapping), and reserve real
hardware for the two things a host can't reproduce: the BT radio and true servo
PWM timing. "Test in a simulator" therefore means two different simulators — the
**Gazebo** robot model for behavior (primary), and optionally a **Pico
instruction-level sim** for the binary's boot / serial I/O (smoke only).

## Tier 1 — host-native pure-lib build + golden tests (extends part 04)

- Reuse part 04's host harness. A CMake target (exposed via `pod`, e.g.
  `pod pico-host-test`) compiles the forked **float** libs with the system
  compiler, **no Pico SDK**, `-fexceptions`: `vec3`, kinematics, gait, posture,
  control velocity-shaping, `joy_mapping`, the `bt_teleop` **adapter** function,
  and part 03's Chica pulse conversion.
- **Golden-trace tests:** sweep a `cmd_vel` / pose script through the float port
  and compare per-joint angle traces against the untouched `hexa_gait_cpp` /
  `hexa_kinematics_cpp` (double) engine within ~1e-3 rad. The 14 `hexa_gait_cpp`
  gtest suites are the reference behavior.
- **Adapter / conversion unit tests** (host gtest, mirror `hexa_hardware`'s test
  setup): feed synthetic gamepad frames → assert the int16 `axes[]`/`buttons[]`
  layout matches `teleop_joy.yaml` (stick signs, trigger rest at `+32767`, dpad
  encoding, button bit indices); angle → `to_pulse_us` → assert `min_us`/`max_us`
  clamps.

## Tier 2 — the portability seam (establish early, alongside Tier 1)

The pure pipeline must be **target-agnostic**, so the host build is a **link-time
swap, not `#ifdef` soup**. Each hardware-touching module is a header-declared
surface with a Pico impl and a host impl:

- **Input** — `bt_teleop::read(axes, buttons)`. Pico impl: `uni.h` (part 02).
  Host impl: scripted axes, a host joystick, or bridged from ROS `/joy` — same
  layout part 02 emits.
- **Servo out** (part 03) — the Chica UART sink. Pico impl: real UART. Host
  impl: publish joint angles into the sim / log frames.
- **Clock** — `time_us_64()` on Pico vs a `std::chrono` steady clock on host.

`main.cpp` composes the Pico impls; a `main_host.cpp` (Tier 3) composes the host
impls. The 50 Hz tick and the whole velocity/posture/compose/IK pipeline are
shared source, compiled unchanged for both targets.

## Tier 3 — Gazebo-in-the-loop (the primary "in a simulator" path)

- Build a small **firmware-bridge node** (`ament_cmake` + `rclcpp`) in the ROS
  workspace that **links the firmware pipeline sources directly** (via a shared
  CMake interface library, or relative sources) and runs the real 50 Hz tick:
  - **Input:** subscribe `/joy` (reuse the existing `joy_publisher` +
    `teleop_joy.yaml`) or drive a host joystick — the exact layout part 02
    produces, so `map_joy` runs identically to on-hardware.
  - **Output:** tap the pipeline at the `JointAngles` stage (before
    `to_pulse_us`), reorder to the controller's joint order, and publish
    `std_msgs/Float64MultiArray` on `/joint_group_position_controller/commands`
    (radians). That is the interface `gz_ros2_control` exposes — see
    `hexa_simulation/config/ros2_controllers.yaml` (18 joints:
    `l_front,l_middle,l_rear,r_front,r_middle,r_rear`, each `coxa,femur,tibia`).
- Launch it alongside `ros2 launch hexa_simulation sim.launch.py` (`pod sim`).
  The **firmware brain now walks the simulated hexapod**: teleop → gait/IK →
  Gazebo. Cross-check against the ROS2 node chain running the same world — same
  teleop input should produce visually identical motion.
- Honors the repo's **sim-first** rule: every feature runs against the Gazebo
  model before any servo code is touched.

## Tier 4 (optional) — firmware-binary smoke sim (Wokwi / Renode)

- Run the actual RP2350 `.elf` / `.uf2` in **Wokwi** (Pico 2 board) or
  **Renode**: smoke-test the boot banner, USB-CDC `stdio`, the 50 Hz scheduler
  cadence, and — once part 03 lands — **Chica UART framing** against a loopback /
  simulated Servo 2040 on the modeled GP0/GP1 pins.
- **Bluetooth / CYW43 cannot be simulated:** stub BT input (a compile-time fake
  feeding a scripted axes sequence, or inject axes over USB-CDC), so this tier
  validates **timing + serial I/O**, not pairing.
- Needs `wokwi-cli` + `diagram.json` + `wokwi.toml` (CI-runnable with a token),
  or a Renode RP2350 platform description. RP2350 support in both is newer than
  RP2040 — pin the versions and note gaps.

## What you need on Linux

- **Already in the dev container** (`hexa dev`): gcc/clang + CMake (host build),
  ROS2 Jazzy + Gazebo Harmonic + `gz_ros2_control` (Tier 3), colcon.
- **Add for Tier 1/2:** a host CMake preset / `pod` subcommand for the native
  firmware build + gtest.
- **Add for Tier 4 (optional):** `wokwi-cli` (or Renode). No ARM hardware or ARM
  toolchain required for Tiers 1–3; Tier 4 needs the ARM build from part 01 but
  still no physical board.

## Done when / verification

- `pod` host target builds the float port with **no Pico SDK**; golden tests
  pass (≤1e-3 rad vs the double engine); adapter + pulse unit tests pass.
- Firmware bridge + `pod sim`: gamepad / keyboard teleop **walks the Gazebo
  hexapod**; posture mode tilts the body with feet planted; gait switching
  changes cadence — behavior matches the ROS2 node chain in the same world.
- (optional) Wokwi / Renode: boot banner + heartbeat over USB-CDC; Chica SET
  frames observed on the simulated UART.
