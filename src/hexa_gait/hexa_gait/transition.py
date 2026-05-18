"""Stop transition: bring all six legs from an arbitrary mid-cycle pose
to the nominal standing pose, then hold.

The strategy contract is ``(phase, stride, leg) -> foot_target`` —
stateless. The stop transition does not fit that mould because each leg
has to remember where it was when ``cmd_vel`` went idle. We split the
behaviour out into a separate stateful controller; the engine routes
between the active strategy and this controller based on the commanded
velocity.

State ladder (matches ``src/hexa_gait/README.md``):

1. **FORCE_TOUCHDOWN** — every leg airborne at stop time swings to its
   nominal stance position via the standard swing arc, rising through
   ``swing_clearance`` before descending, over ``recenter_swing_time``.
   The swing-arc is run with both endpoint velocities pinned to zero,
   so the Bezier decelerates to a true rest at touchdown rather than
   landing at the steady-state stance velocity — that softens the
   impact and keeps the body from rocking. All airborne legs move in
   parallel; the originally-grounded legs hold their stop-time
   positions verbatim so the body stays immobile. The forced
   lift-and-descend matters when a leg stops just above the ground —
   a straight-line move there would skim the floor instead of clearing
   it. Skipped entirely if no leg was airborne.
2. **SETTLE** — hold every foot still for ``touchdown_settle_time``
   seconds after force-touchdown lands. Lets the chassis stop swaying
   before any further leg moves. Skipped when ``touchdown_settle_time``
   is zero or no leg was airborne (nothing to settle from).
3. **RECENTER** — sweep the originally-grounded legs to nominal, one at
   a time, via the standard swing arc, in canonical ``LEG_NAMES`` order.
   By this point every leg is on the ground, so the support polygon is
   always 5/1 stance/swing and the body remains stable.
4. **STAND** — emit nominal stance for all six legs, ``stance=True``.
   Terminal state inside the transition.

Stability invariant: no grounded foot is repositioned while any other
foot is airborne. FORCE_TOUCHDOWN holds the stance legs perfectly still
while the swing legs settle, and RECENTER only starts once all six are
grounded.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping

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
    SETTLE = "settle"
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
    """FORCE_TOUCHDOWN -> SETTLE -> RECENTER -> STAND ladder.

    Construct once at engine startup. Call ``begin(...)`` whenever the
    engine enters the STOPPING state, then ``update(dt)`` each tick
    until ``state == DONE``.
    """

    def __init__(
        self,
        nominal_stance: Mapping[str, Vec3],
        recenter_swing_time: float,
        swing_clearance: float,
        swing_width: float,
        controller_dt: float,
        touchdown_settle_time: float = 0.0,
    ) -> None:
        missing = set(LEG_NAMES) - set(nominal_stance)
        if missing:
            raise ValueError(f"nominal_stance missing legs: {sorted(missing)}")

        self._nominal: dict[str, Vec3] = {n: tuple(nominal_stance[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._recenter_swing_time = recenter_swing_time
        self._swing_clearance = swing_clearance
        self._swing_width = swing_width
        self._controller_dt = controller_dt
        self._touchdown_settle_time = touchdown_settle_time

        self._state = TransitionState.DONE
        self._positions: dict[str, Vec3] = dict(self._nominal)
        self._swing_origins: dict[str, Vec3] = dict(self._nominal)
        # Built per-stop in ``begin()`` from the swing flags.
        self._swing_flags: dict[str, bool] = {n: False for n in LEG_NAMES}
        self._recenter_order: tuple[str, ...] = ()
        self._recenter_idx = 0
        self._recenter_elapsed = 0.0
        self._touchdown_settle_elapsed = 0.0

    @property
    def state(self) -> TransitionState:
        return self._state

    def begin(
        self,
        last_targets: Mapping[str, Vec3],
        swing_flags: Mapping[str, bool],
    ) -> None:
        """Seed the controller with the legs' current pose at stop time.

        FORCE_TOUCHDOWN handles every airborne leg in parallel;
        RECENTER then sweeps the originally-grounded legs one at a
        time in canonical ``LEG_NAMES`` order. Skipping straight to
        RECENTER when no leg was airborne is fine — the controller
        still has work to do on legs that stopped mid-stance away from
        their nominal positions.
        """
        missing = set(LEG_NAMES) - set(last_targets)
        if missing:
            raise ValueError(f"last_targets missing legs: {sorted(missing)}")
        self._positions = {n: tuple(last_targets[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._swing_origins = dict(self._positions)
        self._swing_flags = {n: bool(swing_flags.get(n, False)) for n in LEG_NAMES}

        self._recenter_order = tuple(n for n in LEG_NAMES if not self._swing_flags[n])
        self._recenter_idx = 0
        self._recenter_elapsed = 0.0
        self._touchdown_settle_elapsed = 0.0

        if any(self._swing_flags.values()):
            self._state = TransitionState.FORCE_TOUCHDOWN
        else:
            # No airborne legs ⇒ no force-touchdown impact ⇒ no need to
            # settle. Drop straight to RECENTER.
            self._enter_recenter()

    def update(self, dt: float) -> dict[str, LegOutput]:
        if self._state is TransitionState.FORCE_TOUCHDOWN:
            return self._tick_force_touchdown(dt)
        if self._state is TransitionState.SETTLE:
            return self._tick_settle(dt)
        if self._state is TransitionState.RECENTER:
            return self._tick_recenter(dt)
        # STAND / DONE both emit the nominal stance.
        return {
            n: LegOutput(foot_target=self._nominal[n], phase=0.0, stance=True)
            for n in LEG_NAMES
        }

    def _tick_force_touchdown(self, dt: float) -> dict[str, LegOutput]:
        # All airborne legs follow a rest-to-rest swing arc in parallel
        # from their stop-time pose to nominal over ``recenter_swing_time``.
        # The arc lifts each foot through ``swing_clearance`` so legs
        # stopped just above the ground still clear it on the way home.
        # Both endpoint velocities are pinned to zero so the Bezier
        # decelerates fully at touchdown — landing at the steady-state
        # stance velocity would slam the foot into the floor and rock
        # the body. Originally-grounded legs hold position; the body
        # stays immobile until every foot is on the ground.
        self._recenter_elapsed += dt
        phase = self._recenter_elapsed / self._recenter_swing_time

        out: dict[str, LegOutput] = {}
        if phase >= 1.0:
            for name in LEG_NAMES:
                if self._swing_flags[name]:
                    self._positions[name] = self._nominal[name]
                out[name] = LegOutput(
                    foot_target=self._positions[name], phase=0.0, stance=True
                )
            self._advance_after_force_touchdown()
            return out

        for name in LEG_NAMES:
            if self._swing_flags[name]:
                point = swing_arc(
                    phase_in_swing=phase,
                    swing_origin=self._swing_origins[name],
                    target=self._nominal[name],
                    swing_clearance=self._swing_clearance,
                    swing_width=self._swing_width,
                    identity_y_sign=identity_y_sign(self._nominal[name]),
                    swing_time=self._recenter_swing_time,
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

    def _advance_after_force_touchdown(self) -> None:
        """Route FORCE_TOUCHDOWN -> SETTLE (if configured) or RECENTER.

        Shared by the natural completion path and the begin-time fast
        path that skips FORCE_TOUCHDOWN entirely.
        """
        self._swing_origins = dict(self._positions)
        if self._touchdown_settle_time > 0.0:
            self._state = TransitionState.SETTLE
            self._touchdown_settle_elapsed = 0.0
        else:
            self._enter_recenter()

    def _enter_recenter(self) -> None:
        self._state = TransitionState.RECENTER
        self._recenter_idx = 0
        self._recenter_elapsed = 0.0
        if self._recenter_order:
            self._swing_origins[self._recenter_order[0]] = self._positions[
                self._recenter_order[0]
            ]

    def _tick_settle(self, dt: float) -> dict[str, LegOutput]:
        # Hold every foot still while the chassis stops swaying after
        # FORCE_TOUCHDOWN. We emit the held positions verbatim so any
        # tiny numerical drift from the swing curve does not propagate
        # downstream as motion.
        self._touchdown_settle_elapsed += dt
        out = {
            n: LegOutput(foot_target=self._positions[n], phase=0.0, stance=True)
            for n in LEG_NAMES
        }
        if self._touchdown_settle_elapsed >= self._touchdown_settle_time:
            self._enter_recenter()
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
