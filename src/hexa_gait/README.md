# hexa_gait

Gait engine. Given a desired body velocity and gait parameters, emits a
stream of per-leg foot targets in the body frame.

Designed around a strategy pattern so additional gaits drop in cleanly:
- `gaits/tripod.py` — alternating 3+3 (fast, default).
- `gaits/wave.py`   — one leg at a time (slow, max stability).
- `gaits/ripple.py` — overlapping wave (medium).

Inputs:
- `/gait/params` (`hexa_interfaces/GaitParams`) — gait selection, step
  height, cycle time, target body velocity.

Outputs:
- `/legs/targets` (`hexa_interfaces/LegState[6]`) — foot pose + phase.

The engine is stateful (it owns the phase clock) but the gait strategies
themselves are pure functions of `(phase, params) → foot_target`.

See [`docs/leg-phases.md`](../../docs/leg-phases.md) for the shared
vocabulary (stance, swing, AEP, PEP, duty factor, support polygon) used
below and throughout the codebase.

## Stopping: idle and standing reset

When GaitParams arrives with zero velocity (sent by `hexa_control` when
`cmd_vel` goes idle), the engine does not simply freeze the phase clock
— that would leave any leg in mid-swing dangling in the air. Instead it
runs a three-state reset sequence that brings the robot from an
arbitrary mid-cycle pose to a clean standing pose:

1. **FORCE_TOUCHDOWN** — for each leg currently in swing, drive the
   foot straight down (current XY, ground Z) until it touches down.
   Stance legs hold position. Exits when all six feet are grounded.
2. **RECENTER** — move legs from their current grounded positions to
   their nominal stance positions (the AEP for zero velocity). Done one
   leg at a time, in wave order, using the normal swing-arc trajectory
   (lift → translate → place). Sequential motion guarantees the support
   polygon stays valid throughout.
3. **STAND** — hold the nominal stance with phase frozen. The engine
   stays here until a non-zero velocity arrives.

### Architectural note

The reset sequence is *stateful per leg* (each leg remembers where it
started and where it's going), which does not fit the
`(phase, params) → foot_target` pure-function contract of the gait
strategies. It is therefore implemented as a separate **transition
controller** alongside the strategies; the engine routes between the
active strategy and the transition controller based on commanded
velocity. RECENTER reuses the strategy's swing-arc helper so trajectory
code is not duplicated.

### Resume

If a non-zero velocity arrives mid-sequence, the engine completes the
reset first (FORCE_TOUCHDOWN → RECENTER → STAND) and only then starts
the new gait from the nominal stance. The reset is short (≤ one
wave-style cycle ≈ 6 leg moves), so the latency cost is acceptable, and
aborting mid-sequence would risk legs left in unsafe poses.
