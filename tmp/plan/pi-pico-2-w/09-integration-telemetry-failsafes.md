# Part 09 — Integration, telemetry & failsafes

**Goal:** turn the working chain into a robust, self-contained robot — battery
telemetry, status feedback, safety behavior, and an endurance soak. Final part.

**Depends on:** all prior parts.

## Scope

- **Battery telemetry:** rate-limit Chica GET (every N ticks, mirroring `get_period_ticks=10`); decode voltage/current (scales 0.00366 V, 0.00098 A). Log over USB-CDC; expose a low-battery threshold.
- **Status feedback:** use the onboard LED (via `cyw43_arch`) to signal state — e.g. slow blink = idle/stand, solid = BT connected + walking, fast blink = fault/low battery / BT lost.
- **Failsafes:**
  - **BT input timeout** ~0.5 s (mirror `webteleop safety.input_timeout_s`): no fresh gamepad packet → force `cmd_vel=0` → engine pauses → feet settle. Never walk on stale input.
  - **Relay discipline:** energize only after BT link-up + stand transition; de-energize on fault, timeout, low battery, or fold request.
  - **IK guard:** unreachable target caught → hold last-good angle (from part 05).
  - **Startup:** always boot FOLDED, run `InitializeController` before accepting gait commands.
- **Scheduler hardening:** confirm the 20 ms tick holds under worst case (full engine + posture + 6 SET frames + GET). If margin is thin, move servo UART or IK to core1, or drop GET frequency. Measure and log tick jitter.
- **Heap check:** run a multi-hour soak; monitor free heap for `std::map`-driven fragmentation. If it drifts, execute the `std::map`→`std::array<T,6>` refactor (via `leg_index.hpp`) flagged in the overview.

## Done when / verification

- **Failsafe drills:** (a) walk the gamepad out of BT range mid-gait → robot pauses & settles within the timeout, relay handling correct; (b) drop battery below threshold → status LED + safe stop; (c) power-cycle → clean folded→stand→walk sequence.
- **Telemetry:** logged battery matches a multimeter across a discharge; low-battery path fires at the set threshold.
- **Soak:** N-hour continuous walk/pose session → no heap exhaustion, no tick-deadline misses (logged jitter within budget), no servo command spikes.
- **Full-parity acceptance:** drive, switch gaits, enter posture mode + animations, revert, fold — the on-robot behavior matches the ROS2 system's (spot-checked against the same maneuvers on the Gazebo/host build).
