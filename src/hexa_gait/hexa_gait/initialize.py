"""Cold-start initialization: ``initial_pose`` → standing.

At power-on the hexapod sits on its belly with the legs folded above
the body (see ``geometry.yaml`` ``initial_pose:``). Jumping straight to
the standing pose would snap all 18 servos in a single control step —
fine in sim with light masses, unsafe on the real robot where some
servos cannot report their own angle and the operator is the one
responsible for placing the chassis in the assumed ``initial_pose``.

The ``InitializeController`` runs an orchestrated startup sequence:

1. **PLACE_FEET** — three sequential mirroring pairs swing one at a
   time from the folded ``initial_pose`` foot position onto the standing
   footprint at ground level, while the body stays on its belly. The
   pair order keeps the CoM near the body centre throughout (the
   inactive legs hold their last positions, so the body is supported on
   its belly and on whatever legs have already been placed):
   - Pair 1: ``l_middle`` + ``r_middle``
   - Pair 2: ``l_front`` + ``r_rear`` (diagonal)
   - Pair 3: ``r_front`` + ``l_rear`` (other diagonal)
   Each pair takes ``pair_swing_time`` seconds.

2. **LIFT_BODY** — all six feet stay at standing XY; their body-frame
   z ramps via a smoothstep S-curve from ``-coxa_to_bottom``
   (belly-on-ground) to the leg's standing z. The kinematics chain
   reads this as "feet pressed down, body lifts" — gait owns the lift
   here so no posture-coordination topic is needed for a one-time
   startup.

3. **DONE** — emit ``nominal_stance`` for every leg. The engine treats
   this as the cue to transition to ``STAND``.

The controller is stateful per leg (each leg remembers its current
position so non-active legs can hold), which does not fit the strategy
contract of pure ``(phase, stride, leg) → target``. Modelled on
``TransitionController``: an enum-driven ladder with ``update(dt)``
emitting one ``LegOutput`` per leg per tick.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping

from .clock import LEG_NAMES
from .gaits.base import identity_y_sign, swing_arc
from .transition import LegOutput


__all__ = [
    "InitializeController",
    "InitializeState",
    "PAIR_ORDER",
]

Vec3 = tuple[float, float, float]

# Three sequential mirroring pairs, in the order picked to keep the
# body's centre of mass near the chassis centre while it rests on its
# belly: middle pair first (closest to centre), then each diagonal pair
# in turn. Changing this order changes the static-stability profile
# during PLACE_FEET; revisit any change against the CoM rationale.
PAIR_ORDER: tuple[tuple[str, str], ...] = (
    ("l_middle", "r_middle"),
    ("l_front", "r_rear"),
    ("r_front", "l_rear"),
)


class InitializeState(Enum):
    PLACE_FEET = "place_feet"
    LIFT_BODY = "lift_body"
    DONE = "done"


def _smoothstep(t: float) -> float:
    """Hermite smoothstep ``3t² − 2t³`` on ``[0, 1]``. Same envelope as
    ``EngagementController`` so timing feel matches across cold-start
    transients."""
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    return t * t * (3.0 - 2.0 * t)


class InitializeController:
    """PLACE_FEET → LIFT_BODY → DONE ladder.

    Constructed once at engine start; the engine ticks ``update(dt)``
    every cycle while in the ``INITIALIZE`` state. ``begin`` is not
    needed — the controller starts in PLACE_FEET with its first pair's
    swing origins seeded from ``initial_stance``.
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
        # Per-leg target at the end of PLACE_FEET: standing XY, IK
        # target at body-frame z = -coxa_to_bottom + place_feet_clearance.
        # ``place_feet_clearance`` is the body-frame offset of the IK
        # target above the floor with the body on its belly — NOT the
        # foot sphere bottom's clearance. The URDF places the IK
        # target at the sphere bottom only when the tibia is vertical;
        # the PLACE_FEET endpose has a partially-folded tibia, so the
        # sphere bottom sits up to ~foot_radius below the IK target.
        # The configured clearance must absorb that offset or contact
        # resolution will lift the chassis off its belly before
        # LIFT_BODY runs (see gait.yaml comment).
        self._lift_start_z = -coxa_to_bottom + place_feet_clearance
        self._ground_targets: dict[str, Vec3] = {
            n: (self._nominal[n][0], self._nominal[n][1], self._lift_start_z)
            for n in LEG_NAMES
        }
        self._coxa_to_bottom = coxa_to_bottom
        self._place_feet_clearance = place_feet_clearance
        self._pair_swing_time = pair_swing_time
        self._lift_body_time = lift_body_time
        self._swing_clearance = swing_clearance
        self._swing_width = swing_width
        self._controller_dt = controller_dt

        # Running per-leg foot position; legs that have not yet swung
        # sit at their initial_stance entry, swung legs sit at their
        # ground target, and the active pair are mid-arc.
        self._positions: dict[str, Vec3] = dict(self._initial)
        self._state = InitializeState.PLACE_FEET
        self._pair_idx = 0
        self._t_in_pair = 0.0
        self._t_in_lift = 0.0

    @property
    def state(self) -> InitializeState:
        return self._state

    @property
    def done(self) -> bool:
        return self._state is InitializeState.DONE

    def update(self, dt: float) -> dict[str, LegOutput]:
        if self._state is InitializeState.PLACE_FEET:
            return self._tick_place_feet(dt)
        if self._state is InitializeState.LIFT_BODY:
            return self._tick_lift_body(dt)
        return self._emit_nominal()

    def _tick_place_feet(self, dt: float) -> dict[str, LegOutput]:
        self._t_in_pair += dt
        phase = self._t_in_pair / self._pair_swing_time
        active = PAIR_ORDER[self._pair_idx]

        out: dict[str, LegOutput] = {}
        if phase >= 1.0:
            # Snap the active pair to their ground targets and advance.
            for name in active:
                self._positions[name] = self._ground_targets[name]
            self._pair_idx += 1
            self._t_in_pair = 0.0
            if self._pair_idx >= len(PAIR_ORDER):
                self._state = InitializeState.LIFT_BODY
            for name in LEG_NAMES:
                out[name] = LegOutput(
                    foot_target=self._positions[name], phase=0.0, stance=True
                )
            return out

        # Mid-pair: active legs follow a rest-to-rest swing arc from
        # their initial_stance entry to the ground target. Endpoint
        # velocities pinned to zero (same pattern as
        # TransitionController.FORCE_TOUCHDOWN) so each leg sets down
        # gently rather than at steady-state stance velocity.
        for name in LEG_NAMES:
            if name in active:
                origin = self._initial[name]
                target = self._ground_targets[name]
                point = swing_arc(
                    phase_in_swing=phase,
                    swing_origin=origin,
                    target=target,
                    swing_clearance=self._swing_clearance,
                    swing_width=self._swing_width,
                    identity_y_sign=identity_y_sign(target),
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

    def _tick_lift_body(self, dt: float) -> dict[str, LegOutput]:
        # All six feet stay at standing XY; body-frame z ramps via a
        # smoothstep S-curve from the PLACE_FEET endpoint (1 mm above
        # the floor, see _lift_start_z) down to nominal_stance.z (more
        # negative — foot further from body). World-frame: as the legs
        # extend, the feet make ground contact and the body lifts.
        # Phase reported as the unitless ramp progress so downstream
        # telemetry sees a single increasing scalar.
        self._t_in_lift += dt
        tau = self._t_in_lift / self._lift_body_time
        s = _smoothstep(tau)
        out: dict[str, LegOutput] = {}
        for name in LEG_NAMES:
            nx, ny, nz = self._nominal[name]
            z = self._lift_start_z + s * (nz - self._lift_start_z)
            point = (nx, ny, z)
            self._positions[name] = point
            out[name] = LegOutput(foot_target=point, phase=tau, stance=True)
        if tau >= 1.0:
            # Snap to nominal so downstream sees no drift from the
            # smoothstep arithmetic and advance to DONE.
            for name in LEG_NAMES:
                self._positions[name] = self._nominal[name]
                out[name] = LegOutput(
                    foot_target=self._nominal[name], phase=1.0, stance=True
                )
            self._state = InitializeState.DONE
        return out

    def _emit_nominal(self) -> dict[str, LegOutput]:
        return {
            n: LegOutput(foot_target=self._nominal[n], phase=0.0, stance=True)
            for n in LEG_NAMES
        }
