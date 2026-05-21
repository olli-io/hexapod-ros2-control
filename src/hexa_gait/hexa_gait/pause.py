"""Pause: soft GAIT release that preserves the in-flight gait state.

Counterpart to ``engagement.py`` (STAND → GAIT) — but in pause/resume we
preserve the gait clock so the operator can release the stick briefly
and pick the gait back up without re-engaging from master = 0.

Sequence:

1. **LOWERING** — the controller lowers each currently-airborne leg
   straight down to ``nominal.z`` (XY frozen). Stance legs do not move.
   Per-leg duration scales with the Z drop: ``clamp(distance_z /
   descent_speed, min_reset_time, max_reset_time)``. The engine derives
   ``descent_speed`` from ``stride_length / min_swing_time`` so the
   pause descent runs at the fastest gait's per-leg foot-velocity
   ceiling.
2. **PAUSED** — once every descent has landed, the controller holds
   each foot at its current position. The engine ticks its own
   ``pause_to_reseat_delay`` while in this state; on cmd_vel non-zero
   it hands off to the engagement controller's resume entry point.

The "no grounded foot moves while another is airborne" invariant is
preserved structurally: stance legs literally do not move during
LOWERING. The legs that lower are exactly the set that were already
airborne at pause time, so the simultaneous-airborne count never
exceeds the gait's mid-walk maximum.

The ``LegOutput`` dataclass lives here because pause is now the
nearest equivalent of the deleted disengagement module — every leg-
trajectory controller in this package imports it from one place.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping

from .clock import LEG_NAMES
from .gaits.base import identity_y_sign, swing_arc


__all__ = [
    "LegOutput",
    "PauseController",
    "PauseState",
]

Vec3 = tuple[float, float, float]


class PauseState(Enum):
    LOWERING = "lowering"
    PAUSED = "paused"


@dataclass(frozen=True)
class LegOutput:
    """One leg's contribution to a ``LegTargets`` message.

    ``stance=True`` means the foot is on the ground this tick. During
    a descent the phase value is informational — the descending leg's
    fractional progress through its rest-to-rest curve. Legs already
    on the ground report ``phase=0`` and ``stance=True``.
    """

    foot_target: Vec3
    phase: float
    stance: bool


@dataclass
class _LegDescent:
    """In-progress rest-to-rest descent from origin straight down to target."""

    origin: Vec3
    target: Vec3
    duration: float
    elapsed: float = 0.0


class PauseController:
    """Lower the currently-airborne legs straight down, then hold.

    Construct once at engine startup. Call ``begin(...)`` when the
    engine enters PAUSING, then ``update(dt)`` each tick. ``state``
    flips from LOWERING to PAUSED once every descent has landed; after
    that ``update`` emits the held positions.
    """

    def __init__(
        self,
        nominal_stance: Mapping[str, Vec3],
        swing_clearance: float,
        swing_width: float,
        controller_dt: float,
        descent_speed: float,
        min_reset_time: float,
        max_reset_time: float,
    ) -> None:
        missing = set(LEG_NAMES) - set(nominal_stance)
        if missing:
            raise ValueError(f"nominal_stance missing legs: {sorted(missing)}")
        if descent_speed <= 0.0:
            raise ValueError(f"descent_speed must be positive; got {descent_speed}")
        if min_reset_time <= 0.0:
            raise ValueError(f"min_reset_time must be positive; got {min_reset_time}")
        if max_reset_time < min_reset_time:
            raise ValueError(
                f"max_reset_time {max_reset_time} < min_reset_time {min_reset_time}"
            )

        self._nominal: dict[str, Vec3] = {n: tuple(nominal_stance[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._swing_clearance = swing_clearance
        self._swing_width = swing_width
        self._controller_dt = controller_dt
        self._descent_speed = descent_speed
        self._min_reset_time = min_reset_time
        self._max_reset_time = max_reset_time

        self._state = PauseState.PAUSED
        self._positions: dict[str, Vec3] = dict(self._nominal)
        self._descents: dict[str, _LegDescent] = {}

    @property
    def state(self) -> PauseState:
        return self._state

    def begin(
        self,
        last_targets: Mapping[str, Vec3],
        swing_flags: Mapping[str, bool],
    ) -> None:
        """Seed the controller with the legs' current pose at pause time.

        ``swing_flags[n] == True`` means leg ``n`` was airborne at pause
        time and will be lowered straight down to ``nominal.z``. Stance
        legs hold at their snapshot positions through both LOWERING and
        PAUSED. If no leg is airborne, the controller flips straight to
        PAUSED without scheduling any descent.
        """
        missing = set(LEG_NAMES) - set(last_targets)
        if missing:
            raise ValueError(f"last_targets missing legs: {sorted(missing)}")

        self._positions = {n: tuple(last_targets[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._descents = {}

        for name in LEG_NAMES:
            if not swing_flags.get(name, False):
                continue
            x, y, z_high = self._positions[name]
            z_low = self._nominal[name][2]
            # Already at or below nominal Z — no descent needed; just
            # treat the leg as already landed.
            if z_high <= z_low + 1e-9:
                self._positions[name] = (x, y, z_low)
                continue
            duration = self._adaptive_descent_time(z_high - z_low)
            self._descents[name] = _LegDescent(
                origin=(x, y, z_high),
                target=(x, y, z_low),
                duration=duration,
            )

        if not self._descents:
            self._state = PauseState.PAUSED
        else:
            self._state = PauseState.LOWERING

    def update(self, dt: float) -> dict[str, LegOutput]:
        if self._state is PauseState.PAUSED:
            return self._emit_held()
        return self._tick(dt)

    def _tick(self, dt: float) -> dict[str, LegOutput]:
        out: dict[str, LegOutput] = {}
        for name in LEG_NAMES:
            descent = self._descents.get(name)
            if descent is None:
                out[name] = LegOutput(
                    foot_target=self._positions[name], phase=0.0, stance=True
                )
                continue
            descent.elapsed += dt
            if descent.elapsed >= descent.duration:
                self._positions[name] = descent.target
                del self._descents[name]
                out[name] = LegOutput(
                    foot_target=self._positions[name], phase=0.0, stance=True
                )
            else:
                point = self._descent_point(descent)
                self._positions[name] = point
                out[name] = LegOutput(
                    foot_target=point,
                    phase=descent.elapsed / descent.duration,
                    stance=False,
                )
        if not self._descents:
            self._state = PauseState.PAUSED
        return out

    def _emit_held(self) -> dict[str, LegOutput]:
        return {
            n: LegOutput(foot_target=self._positions[n], phase=0.0, stance=True)
            for n in LEG_NAMES
        }

    def _adaptive_descent_time(self, distance_z: float) -> float:
        raw = distance_z / self._descent_speed
        if raw < self._min_reset_time:
            return self._min_reset_time
        if raw > self._max_reset_time:
            return self._max_reset_time
        return raw

    def _descent_point(self, descent: _LegDescent) -> Vec3:
        # XY stays at the origin; only Z evolves. swing_arc with zero
        # endpoint velocities and zero clearance degenerates to a
        # rest-to-rest interpolation along the stride vector (here,
        # purely -z). swing_clearance = 0 means the apex sits at
        # max(origin_z, target_z) = origin_z, so the foot does not bounce
        # above its starting height.
        phase = descent.elapsed / descent.duration
        return swing_arc(
            phase_in_swing=phase,
            swing_origin=descent.origin,
            target=descent.target,
            swing_clearance=0.0,
            swing_width=self._swing_width,
            identity_y_sign=identity_y_sign(descent.target),
            swing_time=descent.duration,
            controller_dt=self._controller_dt,
            swing_origin_velocity=(0.0, 0.0, 0.0),
            swing_target_velocity=(0.0, 0.0, 0.0),
        )

    @property
    def positions(self) -> dict[str, Vec3]:
        """Last per-leg foot positions emitted (for engine handoff)."""
        return dict(self._positions)
