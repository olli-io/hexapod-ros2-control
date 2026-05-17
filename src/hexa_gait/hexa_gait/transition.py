"""Stop transition: bring all six legs from an arbitrary mid-cycle pose
to the nominal standing pose, then hold.

The strategy contract is ``(phase, stride, leg) -> foot_target`` —
stateless. The stop transition does not fit that mould because each leg
has to remember where it was when ``cmd_vel`` went idle. We split the
behaviour out into a separate stateful controller; the engine routes
between the active strategy and this controller based on the commanded
velocity.

State ladder (matches ``src/hexa_gait/README.md``):

1. **FORCE_TOUCHDOWN** — every leg currently in swing descends straight
   (hold XY, drive Z toward the ground at ``force_touchdown_speed``).
   Legs already in stance hold position. Exits when all six are
   grounded.
2. **RECENTER** — sweep one leg at a time, in ``recenter_order``, from
   its grounded position to its nominal stance via the standard swing
   arc. With a single leg airborne and five planted, the support
   polygon stays valid throughout (5/1 stance/swing).
3. **STAND** — emit nominal stance for all six legs, ``stance=True``.
   Terminal state inside the transition.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping, Sequence

from .clock import LEG_NAMES
from .gaits.base import identity_y_sign, swing_arc


__all__ = [
    "LegOutput",
    "TransitionController",
    "TransitionState",
]

Vec3 = tuple[float, float, float]


class TransitionState(Enum):
    FORCE_TOUCHDOWN = "force_touchdown"
    RECENTER = "recenter"
    STAND = "stand"
    DONE = "done"


@dataclass(frozen=True)
class LegOutput:
    """One leg's contribution to a ``LegTargets`` message.

    Phase and stance are gait metadata; ``stance=True`` means the foot
    is on the ground this tick. During FORCE_TOUCHDOWN / RECENTER the
    phase value is informational (set to 0 for grounded legs, to the
    sweep's fractional progress for the leg currently airborne).
    """

    foot_target: Vec3
    phase: float
    stance: bool


class TransitionController:
    """FORCE_TOUCHDOWN -> RECENTER -> STAND ladder.

    Construct once at engine startup. Call ``begin(...)`` whenever the
    engine enters the STOPPING state, then ``update(dt)`` each tick
    until ``state == DONE``.
    """

    def __init__(
        self,
        nominal_stance: Mapping[str, Vec3],
        force_touchdown_speed: float,
        recenter_swing_time: float,
        recenter_order: Sequence[str],
        swing_clearance: float,
        swing_width: float,
        controller_dt: float,
    ) -> None:
        missing = set(LEG_NAMES) - set(nominal_stance)
        if missing:
            raise ValueError(f"nominal_stance missing legs: {sorted(missing)}")
        if set(recenter_order) != set(LEG_NAMES):
            raise ValueError(
                f"recenter_order must list all six legs exactly once; "
                f"got {list(recenter_order)}"
            )

        self._nominal: dict[str, Vec3] = {n: tuple(nominal_stance[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._force_touchdown_speed = force_touchdown_speed
        self._recenter_swing_time = recenter_swing_time
        self._recenter_order: tuple[str, ...] = tuple(recenter_order)
        self._swing_clearance = swing_clearance
        self._swing_width = swing_width
        self._controller_dt = controller_dt

        self._state = TransitionState.DONE
        self._positions: dict[str, Vec3] = dict(self._nominal)
        self._swing_origins: dict[str, Vec3] = dict(self._nominal)
        self._recenter_idx = 0
        self._recenter_elapsed = 0.0

    @property
    def state(self) -> TransitionState:
        return self._state

    def begin(
        self,
        last_targets: Mapping[str, Vec3],
        swing_flags: Mapping[str, bool],
    ) -> None:
        """Seed the controller with the legs' current pose at stop time."""
        missing = set(LEG_NAMES) - set(last_targets)
        if missing:
            raise ValueError(f"last_targets missing legs: {sorted(missing)}")
        self._positions = {n: tuple(last_targets[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._swing_origins = dict(self._positions)
        # Skip FORCE_TOUCHDOWN if no leg is airborne — common when the
        # engine stops with all six feet already grounded.
        if any(swing_flags.get(n, False) for n in LEG_NAMES):
            self._state = TransitionState.FORCE_TOUCHDOWN
        else:
            self._state = TransitionState.RECENTER
            self._recenter_idx = 0
            self._recenter_elapsed = 0.0
            self._swing_origins[self._recenter_order[0]] = self._positions[
                self._recenter_order[0]
            ]
        self._swing_flags: dict[str, bool] = {n: bool(swing_flags.get(n, False)) for n in LEG_NAMES}

    def update(self, dt: float) -> dict[str, LegOutput]:
        if self._state is TransitionState.FORCE_TOUCHDOWN:
            return self._tick_force_touchdown(dt)
        if self._state is TransitionState.RECENTER:
            return self._tick_recenter(dt)
        # STAND / DONE both emit the nominal stance.
        return {
            n: LegOutput(foot_target=self._nominal[n], phase=0.0, stance=True)
            for n in LEG_NAMES
        }

    def _tick_force_touchdown(self, dt: float) -> dict[str, LegOutput]:
        out: dict[str, LegOutput] = {}
        all_grounded = True
        for name in LEG_NAMES:
            ground_z = self._nominal[name][2]
            x, y, z = self._positions[name]
            if self._swing_flags[name] and z > ground_z:
                z = max(ground_z, z - self._force_touchdown_speed * dt)
                self._positions[name] = (x, y, z)
                if z > ground_z:
                    all_grounded = False
                    out[name] = LegOutput((x, y, z), phase=0.0, stance=False)
                    continue
                # Just touched down this tick.
                self._swing_flags[name] = False
            out[name] = LegOutput(self._positions[name], phase=0.0, stance=True)

        if all_grounded:
            self._state = TransitionState.RECENTER
            self._recenter_idx = 0
            self._recenter_elapsed = 0.0
            self._swing_origins = dict(self._positions)
        return out

    def _tick_recenter(self, dt: float) -> dict[str, LegOutput]:
        if self._recenter_idx >= len(self._recenter_order):
            self._state = TransitionState.STAND
            return {
                n: LegOutput(foot_target=self._nominal[n], phase=0.0, stance=True)
                for n in LEG_NAMES
            }

        active = self._recenter_order[self._recenter_idx]
        self._recenter_elapsed += dt
        phase_in_swing = self._recenter_elapsed / self._recenter_swing_time

        out: dict[str, LegOutput] = {}
        if phase_in_swing >= 1.0:
            # Snap to target and advance to the next leg.
            self._positions[active] = self._nominal[active]
            self._recenter_idx += 1
            self._recenter_elapsed = 0.0
            if self._recenter_idx < len(self._recenter_order):
                self._swing_origins[self._recenter_order[self._recenter_idx]] = (
                    self._positions[self._recenter_order[self._recenter_idx]]
                )
        else:
            origin = self._swing_origins[active]
            target = self._nominal[active]
            point = swing_arc(
                phase_in_swing=phase_in_swing,
                swing_origin=origin,
                target=target,
                swing_clearance=self._swing_clearance,
                swing_width=self._swing_width,
                identity_y_sign=identity_y_sign(self._nominal[active]),
                swing_time=self._recenter_swing_time,
                controller_dt=self._controller_dt,
                swing_origin_velocity=(0.0, 0.0, 0.0),
            )
            self._positions[active] = point

        for name in LEG_NAMES:
            if name == active and self._recenter_idx < len(self._recenter_order):
                out[name] = LegOutput(
                    foot_target=self._positions[name],
                    phase=phase_in_swing,
                    stance=False,
                )
            else:
                out[name] = LegOutput(
                    foot_target=self._positions[name],
                    phase=0.0,
                    stance=True,
                )

        if self._recenter_idx >= len(self._recenter_order):
            self._state = TransitionState.STAND
        return out
