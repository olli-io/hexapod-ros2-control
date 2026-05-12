# hexapod_gait

Gait engine. Given a desired body velocity and gait parameters, emits a
stream of per-leg foot targets in the body frame.

Designed around a strategy pattern so additional gaits drop in cleanly:
- `gaits/tripod.py` — alternating 3+3 (fast, default).
- `gaits/wave.py`   — one leg at a time (slow, max stability).
- `gaits/ripple.py` — overlapping wave (medium).

Inputs:
- `/gait/params` (`hexapod_interfaces/GaitParams`) — gait selection, step
  height, cycle time, target body velocity.

Outputs:
- `/legs/targets` (`hexapod_interfaces/LegState[6]`) — foot pose + phase.

The engine is stateful (it owns the phase clock) but the gait strategies
themselves are pure functions of `(phase, params) → foot_target`.
