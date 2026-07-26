# Host test harness

Native (x86, **float**) build of the firmware's pure libs — **no Pico SDK, no
ROS** — so the whole port's logic is unit- and golden-testable off-target, which
is where essentially all the port risk lives (float conversion, kinematics,
gait, posture, teleop mapping). This is plan part 04's foundation, extended by
parts 05–08 and formalized as Tier 1 of part 10.

## What builds today (part 04)

- **`test_vec3`** — the hand-rolled float `Vec3` (`src/vec3.hpp`): dot, cross,
  add/sub/negate, scalar mul/div, `norm` (hypotf), `normalized`, `Zero`,
  indexed access, and `constexpr` usability, against hand-computed values.
- **`test_config`** — spot-checks of `src/config_generated.hpp` (emitted by
  `tools/gen_config.py`): the six-leg mount **symmetry expansion** (incl.
  `r_rear = (-0.083, -0.0575, -150°)`), deg→rad joint conventions, standing /
  initial pose, gait engine knobs, the **derived** per-gait velocity caps
  (`tripod linear_max = 1/3 m/s`, stability flags), teleop hardware identity,
  posture stack, control ramps, and the 18 servo calibrations. These pin the
  generator against the ROS2 loaders it ports (`description_loader.cpp`,
  `limits.cpp`, `joint_calibration.cpp`).
- **`dump_config`** — not a test; prints the six symmetry-expanded leg specs and
  the derived velocity caps for the part-04 eyeball check.

The config header is regenerated from the repo YAMLs at configure/build time, so
these tests always run against the current `hexa_description` values.

## Build & run

Host (needs `cmake`, a C++20 compiler, GoogleTest, `python3` + PyYAML), or the
same inside `hexa sim`:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/dump_config      # eyeball the leg specs / velocity caps
```

CMake regenerates `../../src/config_generated.hpp` first (custom command over
`tools/gen_config.py` + the source YAMLs).

## Golden-trace suites (Tier 1 of part 10)

As each pure lib is forked into `../../src`, add its gtest suite via the
`hexa_host_test()` helper (built under `-Wdouble-promotion`, the firmware gate),
plus a **golden-trace driver** that diffs the float port against the untouched
double `hexa_kinematics_cpp` / `hexa_gait_cpp` engines. The golden targets
compile both the float and double sources into one binary (distinct namespaces,
no ODR clash) and deliberately bridge float↔double, so they drop
`-Wdouble-promotion` and link Eigen (+ yaml-cpp for gait).

Landed:

- **`test_kinematics` (part 05)** — FK/IK/body-transform/compose, float vs double
  within 1e-4 m / 1e-3 rad.
- **`test_gait_unit` (part 06)** — float-only behavioural tests of the clock,
  trajectory, strategies, and the engine state machine (cold-start → walk → stop
  → gait switch); the `-Wdouble-promotion` compile is itself part of the gate.
- **`test_gait` (part 06)** — replays a cmd_vel profile
  (cold-start → stand → walk → stop → settle) through the float engine and the
  double `hexa_gait_cpp` engine, both built from the same baked config, and diffs
  per-leg foot target / phase / stance every phase-locked tick (< 2e-3 m). The
  deterministic wall-clock ladders (INITIALIZE / RESEATING) are excluded from the
  strict diff: float-vs-double accumulation rounding can shift a ramp boundary by
  one tick, a transient self-correcting offset, so only the phase-clock-driven
  STAND/ENGAGING/GAIT/SETTLING window is diffed tightly.
- **`test_control` (part 07)** — float-only unit tests of the
  `BodyVelocityLimiter` (constant-max-accel linear/vectorial/angular slew,
  flip-through-zero at one bounded rate, snap-to-zero, positive-accel guard) and
  the `Control` stage (settles at the gait linear cap, resets the limiter on
  leaving the walking set, recomputes the accel cap on a gait switch).
- **`test_joy_mapping` (part 07)** — golden-trace **parity** of the float
  `map_joy` port against the untouched Python `hexa_teleop.joy_mapping`
  reference. `gen_joy_golden.py` imports the pure reference (no rclpy), builds a
  `JoyConfig` from the same YAMLs the firmware bakes, and replays a scripted
  axes/buttons trace (drive, cyclers, posture tilt/pose, height, yaw/wiggle,
  record, two-press init/revert, animation entry/cycle/exit) through it, baking
  the frames + expected `JoyOutput`s into `joy_golden_generated.hpp`. The C++
  test drives the port over identical inputs and asserts the same output
  frame-for-frame (< 2e-4). This is what caught the `gait_cycle`
  unstable-filtering divergence during the port.
- **`test_posture` (part 08)** — two tiers in one float-only suite. The
  behavioural tier is a direct port of the three `hexa_posture` pytest suites
  (`pose` algebra, the seven animations, the node's signal-derivation +
  low-pass helpers, the `POSTURE_ACTIVE_STATES` gate), with expected values
  copied verbatim from the Python tests. The golden tier drives one
  `PostureController` across a recorded engine trace (per-leg foot targets +
  stance, master phase, walking flag, engine state, user pose, animation-mode
  selection) and asserts the full `body_pose_target` matches the **real** pure
  Python `hexa_posture` animation stack within 1e-4. `gen_posture_golden.py`
  imports the untouched pure `pose` + `animations` modules (no rclpy) and
  transcribes the node's signal/filter math — which the C++ port of the same
  helpers is independently checked against in the behavioural tier — baking the
  frames + expected `BodyPose`s into `posture_golden_generated.hpp`.
- **`test_pipeline` (part 10, Tier 2/3 seam)** — integration/behaviour test of
  the target-agnostic control brain (`src/pipeline.cpp`,
  `hexa::pipeline::Pipeline`). Drives the WHOLE firmware pipeline — teleop
  mapping → velocity shaping → gait → posture → compose/IK, plus the supervisor —
  natively, the exact source the Pico firmware (`main.cpp`) and the Gazebo bridge
  (`hexa_pico_bridge/firmware_bridge_node.cpp`) compile. It is the off-target
  proof that the extraction composes and the full chain runs: a cold pipeline
  holds FOLDED, the init button stands it up and arms the relay, a stick command
  engages the gait (legs move, master phase advances, every foot stays
  reachable), and a lost link safe-stops (watchdog fires, command force-zeroed,
  posture gated off). Per-stage numeric fidelity is the golden suites' job; this
  pins the composition. Float-only, so it compiles under `-Wdouble-promotion`
  like the rest of the port.
- **`test_supervisor` (part 09)** — float-only unit tests of the integration
  supervisor (`src/supervisor.cpp`, no Pico SDK): the battery hysteresis debounce
  (disabled-threshold no-op, hold-time latch, brief-recovery reset, hysteresis
  clear, critical latch — ported from `hexa_display`'s `BatteryMonitor`), the
  stale-input watchdog (fresh / timed-out / disconnected / no-frame-yet), the
  relay-arming state machine (arms on link-up in any non-fault state — including
  a boot-time FOLDED — holds through a stale link and through being held folded,
  drops on the completed-fold edge / critical battery / fault, won't re-arm while
  critical), the per-leg energize sweep that staggers the inrush at the relay
  edge (`energize_sweep.hpp`), the status-LED mapping (solid only when linked + walking +
  fresh, fast blink on fault, slow otherwise), and the tick-jitter accounting
  (interval min/max, deadline overruns vs the period + margin).

This is the regression gate that must be green before flashing.
