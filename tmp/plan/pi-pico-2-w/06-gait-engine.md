# Part 06 — Gait engine (walk in place → walk)

**Goal:** port the full gait engine so the robot walks from a hardcoded `cmd_vel`.
The largest fork; the core of locomotion.

**Depends on:** 04, 05. **Blocks:** 07, 08.

## Scope

Fork the `hexa_gait_cpp` pure libs into `src/gait/` (convert `double`→`float`,
drop yaml loaders → use `config_generated.hpp`, keep `std::map<string,...>` for now):

- `clock.cpp` — `GaitClock`/`PhaseOffsets` (master phase advance, per-leg `(master+offset) mod 1`). The only place time enters the chain.
- `trajectory.cpp` — quartic Bézier (`quartic_bezier`, `_dot`, primary/secondary swing nodes, stance nodes). Syropod-style, C0/C1/C2 continuity.
- `gaits/base.cpp` + `gaits/registry.cpp` — `Strategy` base, `per_leg_planar_velocity` (`v = v_body + ω×r`), `stride_vector`, `derive_cycle_time`, `swing_arc`; the 5 gaits (tripod β0.5, tetrapod 2/3, surf, crawl, ripple) with phase offsets. `phased_foot_target` splits swing window `[0,1−β)` / stance `[1−β,1)`.
- Sub-controllers: `engagement.cpp` (STAND↔GAIT smoothstep), `pause.cpp` (lower airborne feet), `stand_transition.cpp` (`InitializeController` folded→stand, `FoldController`), `reseat.cpp` (+`ReseatGeometry`; pulls `kin::` from part 05), `limits.cpp` (`scale_to_envelope`, `VelocityCaps`). **Drop `stability.cpp`** (not wired into runtime).
- `engine.cpp` (~701 lines) — `Engine` + `EngineState` FSM (`FOLDED,INITIALIZE,STAND,ENGAGING,GAIT,PAUSING,PAUSED,RESUMING,FOLDING,RESEATING`), `StanceIntegrator`, `SwingPlanner`. Main API: `update(dt,{vx,vy},wz) → map<leg,LegOutput>`; plus `state()`, `master_phase()`, `set_strategy()`, `start_initialize()`, `request_fold()`, `set_target_height()`. Build the engine from `config_generated.hpp` (replaces the `*_from_yaml` builders).
- `main.cpp`: run `engine.update` in the 50 Hz tick, feed a **hardcoded** `cmd_vel`, pipe `LegOutput.foot_target` → compose (part 05) → IK → pulse. Start FOLDED → `start_initialize()` → STAND before commanding gait.

## Done when / verification

- **Host golden-trace:** replay a `cmd_vel` profile through the float engine and diff per-leg `foot_target`/`phase`/`stance` and final joint angles against the double `hexa_gait_cpp` within ~1e-3; run against the 14 existing gtest scenarios as reference.
- **Target (on a stand):** hardcode forward `cmd_vel` → tripod gait: correct legs swing/stance in phase, feet trace a lift-and-place arc. Set `cmd_vel=0` → engine pauses, feet settle. Switch strategy (tripod→tetrapod) → offsets change correctly.
- Cold start: FOLDED → initialize → stand transition runs smoothly (no snap), then walks.
- Measure per-tick compute time on target → confirm the full engine tick + IK fits in 20 ms with margin.
