# Pico 2 W hexapod firmware — overview & shared context

Port the ROS2 hexapod brain onto a single **Raspberry Pi Pico 2 W (RP2350)**,
bare-metal on the Pico SDK — no ROS, no microROS. Teleop arrives over
**Bluetooth**; everything else (gait, kinematics, control velocity-shaping, the
full posture animation stack, battery telemetry) runs onboard. The Pico 2 W
replaces the Linux host and keeps the existing **Servo 2040** board as a dumb
PWM slave over UART using the current "Chica" protocol.

This file holds the context shared by every part. Each `NN-*.md` part is an
individually testable increment that references back here.

## Locked decisions

- **Servo output:** keep the Servo 2040 slave; reuse the Chica UART protocol (two MCUs).
- **Scope:** full parity — gait + IK + control + posture (7 animations) + battery. Drop only the LED-face display (`hexa_display`).
- **Code strategy:** standalone fork under `pi`; copy the pure C++ libs, convert `double→float`, bake config in at build time. The `hexa_*_cpp` ROS2 packages stay untouched.

## Target hardware & constraints

- **RP2350:** dual Cortex-M33 @ 150 MHz, **single-precision FPU only** (no HW double), 520 kB SRAM, 4 MB flash. Use `float` + `sinf/cosf/sqrtf` throughout. 18-joint IK at 50 Hz is ~0.2% of one core — FP cost is a non-issue.
- **Bluetooth:** CYW43439 (BT Classic + BLE). Pico SDK bundles BTstack (license pre-paid). Use **Bluepad32** (**Pico SDK ≥ 2.1**, `PICO_BOARD=pico2_w`).
- **Servo 2040 link:** UART carries Chica SET (pulse widths), GET (battery ADC), and the relay pin. No PWM code on the Pico. Servo power stays on the Servo 2040 rail; Pico shares ground only.

## Data flow (50 Hz tick, mirrors the ROS graph as in-process calls)

```
BT gamepad ─Bluepad32(async)─> axes[]/buttons[]
  │ joy_mapping.map_joy(...)                → JoyOutput{cmd_vel, body_pose, mode, gait_select?, init?, anim?}
  ├─ velocity: scale_to_envelope → limiter.step → engine.update(dt,{vx,vy},wz)
  │            → per-leg LegOutput{foot_target(nominal), phase, stance}, master_phase, engine_state
  └─ posture:  derive(centroid, swing_lift) → LPF → gate(engine_state)
               → AnimationContext → stack(ctx) → clamp(user_pose + animated) = body_pose_target
  compose/leg: apply_body_pose(foot_target, body_pose_target) → body_to_leg
               → inverse_kinematics → JointAngles → to_pulse_us → Chica SET ─UART─> Servo 2040 → 18 PWM
  periodic:    Chica GET ─UART─> battery voltage/current
```

The composition (`apply_body_pose` → `body_to_leg` → `inverse_kinematics` + the
joint→pin ordering) currently lives in `src/hexa_kinematics_cpp/src/ik_node.cpp`
and `joint_command_bridge.cpp` — **read those to replicate the exact ordering**.

## Repo layout (new tree; ROS2 workspace untouched)

```
pi-pico-firmware/
  CMakeLists.txt  pico_sdk_import.cmake  btstack_config.h
  tools/gen_config.py            # repo YAMLs → config_generated.hpp
  src/ main.cpp bt_teleop.* joy_mapping.* control.* servo_out.*
       kinematics/ gait/ posture/ vec3.hpp leg_index.hpp config_generated.hpp
```

## Cross-cutting conventions (apply in every part)

- **Float math:** `Vec3` = hand-rolled `struct{float x,y,z;}` (replaces `Eigen::Vector3d`); `JointAngles` = `std::array<float,3>`. No Eigen (avoids M33 alignment/SIMD tuning).
- **Config codegen:** no runtime filesystem. `tools/gen_config.py` reads the repo YAMLs and emits `constexpr` structs into `config_generated.hpp` at build time, keeping values sourced from `hexa_description`. See part 04.
- **Exceptions:** IK throws `UnreachableTarget`; reseat throws `invalid_argument`. Build `-fexceptions`; wrap per-leg IK in try/catch → hold last-good angle on unreachable. Convert to `std::optional` later if flash/latency demands.
- **`std::map<std::string,...>` currency:** the forked engine keys legs by name and allocates per tick. Keep as-is for the initial port; measure heap churn at part 06; array-refactor (indexed by `Leg` enum in `leg_index.hpp`) is the follow-up optimization.
- **Scheduler:** single-core cooperative loop on core0; BTstack + Bluepad32 run in the cyw43 background context (`pico_cyw43_arch_threadsafe_background`), delivering controller state via `uni_platform` callbacks — so the loop just reads the latest snapshot (`bt_teleop::read`) and runs one control tick every 20 ms (50 Hz, matching the ROS node rate). Bluepad32 uses the C platform API (`uni.h`), not the Arduino `BP32`/`ControllerPtr` API. USB-CDC `stdio` for debug logs.
- **Failsafes:** BT input timeout ~0.5 s → `cmd_vel=0` → engine pauses/settles; relay energized only after link-up + stand; `min_us`/`max_us` clamps are the electrical backstop. Full treatment in part 09.

## Parts & dependency graph

- `01-skeleton-toolchain.md` — Pico SDK project builds, LED, USB-CDC stdio.
- `02-bt-teleop.md` — Bluepad32 pairs a gamepad, dumps axes/buttons. (needs 01)
- `03-servo-slave-link.md` — Chica protocol over Pico UART, one servo + relay + battery GET. (needs 01)
- `04-foundations-config.md` — `vec3.hpp`, `leg_index.hpp`, `gen_config.py` → `config_generated.hpp`; host-unit-testable. (needs 01)
- `05-kinematics.md` — fork `leg_ik`+`body_transform` (float); stand pose on 18 joints. (needs 03, 04)
- `06-gait-engine.md` — fork engine+strategies+clock+trajectory+sub-controllers; tripod walk-in-place. (needs 04, 05)
- `07-control-teleop-mapping.md` — `map_joy`, `BodyVelocityLimiter`, `scale_to_envelope`; full velocity teleop. (needs 02, 04, 06)
- `08-posture.md` — animation stack + node signal-derivation/filter/gate; pose + animations into IK. (needs 04, 05, 06)
- `09-integration-telemetry-failsafes.md` — battery readout, status LED, failsafes, soak test. (needs all)
- `10-linux-build-sim-testing.md` — build & test on Linux with no hardware: native float golden tests + the pipeline driving the Gazebo hexapod (sim-first); optional Wokwi/Renode binary smoke. (cross-cutting: Tier 1 rides on 04, Tier 3 on 07/08)

Ordering: 01 → {02, 03, 04 in parallel} → 05 → 06 → {07, 08} → 09. Part 10 is
cross-cutting — its host-test seam (Tiers 1–2) is stood up with 04 and the
Gazebo-in-the-loop bridge (Tier 3) follows the pipeline as 05–08 land.

## Host golden-trace testing (shared harness, introduced in 04, reused in 05/06/07/08)

The forked pure libs compile natively (x86, float). Build a small host harness
that feeds a `cmd_vel`/pose sweep through the float port and compares per-joint
angle traces against the untouched ROS2 `hexa_gait_cpp`/`hexa_kinematics_cpp`
(double) engine within a loosened tolerance (~1e-3 rad). This catches port
regressions before any flashing. The 14 existing `hexa_gait_cpp` gtest suites
are the reference behavior. Part 10 builds on this harness: it adds the
target-agnostic input/servo/clock seam and a Gazebo-in-the-loop bridge so the
whole firmware pipeline (not just gait/IK) can be driven against the simulated
hexapod on Linux with no hardware.
