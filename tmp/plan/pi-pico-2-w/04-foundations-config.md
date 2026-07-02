# Part 04 — Shared foundations: vec3, leg index, config codegen, host harness

**Goal:** the shared primitives every math/port part needs — a float 3-vector, a
leg enum + name tables, the build-time config generator, and the host
golden-trace test harness. All host-unit-testable with no target hardware.

**Depends on:** 01. **Blocks:** 05, 06, 07, 08.

## Scope

- `src/vec3.hpp` — `struct Vec3{float x,y,z;}` with the ops the ported code uses: `+ - `, scalar `*`, dot, `Zero()`, element access, and free `hypotf`-based helpers. Drop-in for `Eigen::Vector3d`. `JointAngles = std::array<float,3>`.
- `src/leg_index.hpp` — `enum class Leg{L_FRONT,L_MIDDLE,L_REAR,R_FRONT,R_MIDDLE,R_REAR}`, the 6 name strings (matching `LEG_NAMES` in the libs), and index⇄name helpers. Prepares the later `std::map`→`std::array<T,6>` optimization.
- `tools/gen_config.py` — reads the repo YAMLs and emits `src/config_generated.hpp` as `constexpr` structs (run by CMake pre-build). Sources & keys:
  - `hexa_description/config/geometry.yaml` → per-leg `LegSpec` (coxa 0.042, femur 0.08, tibia 0.134 m; mounts l_front (0.083,0.0575,30°), l_middle (0,0.082,90°), **expand 6 by symmetry: rear mirrors x & yaw→π−yaw, right mirrors y & yaw→−yaw**), joint limits (deg→rad: coxa `deg`, femur `−deg`, tibia `π−deg`), `coxa_to_bottom` (0.03), `initial_pose`.
  - `hexa_description/config/standing_pose.yaml` → nominal stance (coxa 0°, femur 35°, tibia 68°).
  - `hexa_gait_cpp/config/gait.yaml` → `EngineConfig` (~20 knobs) + `VelocityCaps` (per-gait `linear_max`, `angular_z_max` 3.0, `yaw_bias` 0.6).
  - `hexa_teleop/config/teleop_joy.yaml` → button/axis indices, signs, per-mode bindings, posture scalar limits, `gait_cycle`.
  - `hexa_posture/config/posture.yaml` → `enabled_animations`, per-animation amplitudes/phases, `animation_mode_animations`.
  - `hexa_control/config/control.yaml` → ramp times, snap tolerances.
  - `hexa_hardware/config/hardware.yaml` → 18× {pin, joint_position, us_at_±45, min/max_us}, relay pin, battery scales.
- **Host harness** (`firmware/pico/test/host/`): CMake target compiling the *forked* pure libs natively (float) + a driver that replays a `cmd_vel`/pose sweep and diffs per-joint angles against the untouched ROS2 `hexa_gait_cpp`/`hexa_kinematics_cpp` (double) reference, tolerance ~1e-3 rad. Reused by parts 05–08.

## Done when / verification

- `vec3` unit tests pass (dot/cross/add/scale/hypot against hand-computed values).
- `gen_config.py` runs in CMake and produces a `config_generated.hpp` that compiles; spot-check a few emitted constants against the YAMLs (e.g. l_middle mount, tripod duty 0.5, stride_length 0.1).
- Symmetry expansion verified: print all 6 `LegSpec`s, confirm r_rear = (−0.083,−0.0575,−150°) etc.
- Host harness builds and runs (initially against just FK/IK once part 05 lands) — establishes the regression gate.
