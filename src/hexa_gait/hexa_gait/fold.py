"""Warm shutdown: standing → ``initial_pose``.

Reverse of ``InitializeController``. The ``FoldController`` brings the
chassis from a steady standing pose back to the folded ``initial_pose``
(legs tucked above the body, chassis resting on its belly) under
operator command — typically the same start button that ran the
cold-start.

The ladder is the time-reverse of INITIALIZE:

1. **LOWER_BODY** — all six feet stay at standing XY; their body-frame
   z ramps via a smoothstep S-curve from ``nominal.z`` up to
   ``-coxa_to_bottom + place_feet_clearance`` (legs retract, world-
   frame body lowers onto its belly).

2. **LIFT_FEET** — three sequential mirroring pairs swing one at a
   time from the standing footprint (at the LOWER_BODY endpoint) back
   up to the folded ``initial_pose`` foot position. Pair order is the
   reverse of ``PAIR_ORDER`` (each diagonal in turn, then the middle
   pair). The chassis rests on its belly throughout, so static
   stability is not the constraint; the reversed order mirrors the
   cold-start for symmetry.

3. **DONE** — emit ``initial_stance`` for every leg. The engine treats
   this as the cue to transition to ``FOLDED``.

Stateful per leg (mirrors ``InitializeController``): each leg
remembers its current foot position so non-active legs hold while one
pair is mid-arc.
"""

from __future__ import annotations

from enum import Enum
from typing import Mapping

from .clock import LEG_NAMES
from .gaits.base import identity_y_sign, swing_arc
from .initialize import PAIR_ORDER, _smoothstep
from .disengagement import LegOutput


__all__ = [
    "FoldController",
    "FoldState",
    "PAIR_ORDER_REVERSED",
]


Vec3 = tuple[float, float, float]


# Reverse of initialize's PAIR_ORDER. With the chassis on its belly
# throughout LIFT_FEET, weight-bearing is not the constraint here;
# the reversed order is chosen for symmetry with the cold-start.
PAIR_ORDER_REVERSED: tuple[tuple[str, str], ...] = tuple(reversed(PAIR_ORDER))


class FoldState(Enum):
    LOWER_BODY = "lower_body"
    LIFT_FEET = "lift_feet"
    DONE = "done"


class FoldController:
    """LOWER_BODY → LIFT_FEET → DONE ladder.

    Constructed each time the operator triggers a fold (engine's
    ``start_fold``); the engine ticks ``update(dt)`` every cycle while
    in the ``FOLDING`` state.
    """

    def __init__(
        self,
        initial_stance: Mapping[str, Vec3],
        nominal_stance: Mapping[str, Vec3],
        coxa_to_bottom: float,
        pair_swing_time: float,
        lift_body_time: float,
        swing_clearance: float,
        place_feet_clearance: float,
        swing_width: float,
        controller_dt: float,
    ) -> None:
        missing = set(LEG_NAMES) - set(initial_stance)
        if missing:
            raise ValueError(f"initial_stance missing legs: {sorted(missing)}")
        missing = set(LEG_NAMES) - set(nominal_stance)
        if missing:
            raise ValueError(f"nominal_stance missing legs: {sorted(missing)}")
        if pair_swing_time <= 0.0:
            raise ValueError(f"pair_swing_time must be positive; got {pair_swing_time}")
        if lift_body_time <= 0.0:
            raise ValueError(f"lift_body_time must be positive; got {lift_body_time}")

        self._initial: dict[str, Vec3] = {n: tuple(initial_stance[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._nominal: dict[str, Vec3] = {n: tuple(nominal_stance[n]) for n in LEG_NAMES}  # type: ignore[misc]
        # LOWER_BODY endpoint == LIFT_FEET swing origin: standing XY,
        # body-frame z at the body-on-belly height. Same expression as
        # ``InitializeController._lift_start_z`` so a fold-then-init
        # round trip lines up exactly.
        self._lower_end_z = -coxa_to_bottom + place_feet_clearance
        self._ground_targets: dict[str, Vec3] = {
            n: (self._nominal[n][0], self._nominal[n][1], self._lower_end_z)
            for n in LEG_NAMES
        }
        self._pair_swing_time = pair_swing_time
        self._lift_body_time = lift_body_time
        self._swing_clearance = swing_clearance
        self._swing_width = swing_width
        self._controller_dt = controller_dt

        # Running per-leg foot position; legs that have not yet lifted
        # sit at their ground target, lifted legs sit at their
        # initial_stance entry, and the active pair are mid-arc.
        self._positions: dict[str, Vec3] = dict(self._nominal)
        self._state = FoldState.LOWER_BODY
        self._pair_idx = 0
        self._t_in_pair = 0.0
        self._t_in_lower = 0.0

    @property
    def state(self) -> FoldState:
        return self._state

    @property
    def done(self) -> bool:
        return self._state is FoldState.DONE

    def update(self, dt: float) -> dict[str, LegOutput]:
        if self._state is FoldState.LOWER_BODY:
            return self._tick_lower_body(dt)
        if self._state is FoldState.LIFT_FEET:
            return self._tick_lift_feet(dt)
        return self._emit_initial()

    def _tick_lower_body(self, dt: float) -> dict[str, LegOutput]:
        # All six feet stay at standing XY; body-frame z ramps via a
        # smoothstep S-curve from nominal.z up to _lower_end_z (less
        # negative — foot closer to body). World-frame: legs retract,
        # body lowers onto its belly. Phase reported as the unitless
        # ramp progress (same convention as InitializeController's
        # LIFT_BODY tick).
        self._t_in_lower += dt
        tau = self._t_in_lower / self._lift_body_time
        s = _smoothstep(tau)
        out: dict[str, LegOutput] = {}
        for name in LEG_NAMES:
            nx, ny, nz = self._nominal[name]
            z = nz + s * (self._lower_end_z - nz)
            point = (nx, ny, z)
            self._positions[name] = point
            out[name] = LegOutput(foot_target=point, phase=tau, stance=True)
        if tau >= 1.0:
            # Snap to the LOWER_BODY endpoint so downstream sees no
            # drift from the smoothstep arithmetic and advance to
            # LIFT_FEET.
            for name in LEG_NAMES:
                self._positions[name] = self._ground_targets[name]
                out[name] = LegOutput(
                    foot_target=self._ground_targets[name], phase=1.0, stance=True
                )
            self._state = FoldState.LIFT_FEET
        return out

    def _tick_lift_feet(self, dt: float) -> dict[str, LegOutput]:
        self._t_in_pair += dt
        phase = self._t_in_pair / self._pair_swing_time
        active = PAIR_ORDER_REVERSED[self._pair_idx]

        out: dict[str, LegOutput] = {}
        if phase >= 1.0:
            # Snap the active pair to their initial_stance entries and
            # advance.
            for name in active:
                self._positions[name] = self._initial[name]
            self._pair_idx += 1
            self._t_in_pair = 0.0
            if self._pair_idx >= len(PAIR_ORDER_REVERSED):
                self._state = FoldState.DONE
            for name in LEG_NAMES:
                out[name] = LegOutput(
                    foot_target=self._positions[name], phase=0.0, stance=True
                )
            return out

        # Mid-pair: active legs follow a rest-to-rest swing arc from
        # the ground target up to the folded initial_stance position.
        # Endpoint velocities pinned to zero so the foot reaches its
        # tucked pose without overshoot (same pattern as
        # InitializeController.PLACE_FEET).
        for name in LEG_NAMES:
            if name in active:
                origin = self._ground_targets[name]
                target = self._initial[name]
                point = swing_arc(
                    phase_in_swing=phase,
                    swing_origin=origin,
                    target=target,
                    swing_clearance=self._swing_clearance,
                    swing_width=self._swing_width,
                    identity_y_sign=identity_y_sign(origin),
                    swing_time=self._pair_swing_time,
                    controller_dt=self._controller_dt,
                    swing_origin_velocity=(0.0, 0.0, 0.0),
                    swing_target_velocity=(0.0, 0.0, 0.0),
                )
                self._positions[name] = point
                out[name] = LegOutput(foot_target=point, phase=phase, stance=False)
            else:
                out[name] = LegOutput(
                    foot_target=self._positions[name], phase=0.0, stance=True
                )
        return out

    def _emit_initial(self) -> dict[str, LegOutput]:
        return {
            n: LegOutput(foot_target=self._initial[n], phase=0.0, stance=True)
            for n in LEG_NAMES
        }
