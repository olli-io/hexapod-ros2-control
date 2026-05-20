"""Disengagement: bring all six legs from an arbitrary mid-cycle pose
to the nominal standing pose, in the active gait's natural lift-off
ordering. Counterpart to ``engagement.py`` (STAND → GAIT); this module
handles GAIT → STAND.

The strategy contract is ``(phase, stride, leg) -> foot_target`` —
stateless. Disengagement does not fit that mould because each leg has
to remember where it was when ``cmd_vel`` went idle. We split it out
into a stateful controller; the engine routes between the active
strategy and this controller based on the commanded velocity.

Sequence:

1. **Snapshot** — record each leg's stop-time pose and per-leg phase
   ``(master + offset) mod 1``.
2. **Build a queue of groups.** A group is a set of legs scheduled to
   swing in parallel.

   - The first group is every leg currently in the swing window
     ``phase < 1 − β`` (or flagged airborne by the engine). They are
     in the air, so they come down first. For tripod this is a single
     offset triple; for ripple it can be two singletons that happened
     to overlap in the swing window; for wave it is at most one leg.
   - The remaining (stance) legs are grouped by exact phase offset
     (legs sharing an offset are gait-natural parallel partners, e.g.
     tripod's three) and ordered by **descending current phase** —
     the group whose phase is closest to wrapping to 0 goes next,
     because that is the next lift-off the gait itself would have
     produced.
   - Any group whose every leg is already at nominal is dropped (no
     twitch).
3. **Drain the queue.** Each group's swings run in parallel,
   rest-to-rest, with per-leg adaptive duration
   ``clamp(distance_xy / max_foot_speed, min_swing_time, max_swing_time)``
   so a leg displaced by ~stride_length lands inside the gait's natural
   swing budget while a leg close to nominal does not slam. The apex is
   the **higher of** ``origin_z`` and ``target_z + swing_clearance``:
   grounded legs lift the full clearance; airborne legs already above
   that height descend without an extra bounce; airborne legs near the
   floor get a partial lift to the same apex a grounded leg would use.
   Endpoint velocities are pinned to zero so the Bezier decelerates
   fully at touchdown. Groups run back-to-back — when the slowest leg
   in the head group lands, the next group starts immediately.
4. **STAND** once the queue is empty.

Per-gait total time bound (at ``min_swing_time = 0.3 s``):

- Tripod (β=0.5, two offset groups) — ≤ 2 × max_swing_time.
- Ripple (β=2/3, up to five groups after merging the swing overlap) —
  ≤ 5 × max_swing_time.
- Wave (β=5/6, six groups) — ≤ 6 × max_swing_time.

The "no grounded foot moves while another is airborne" invariant is
*inherited from the gait itself* — each parallel group is a support
set that the gait already validates as stable. Sequential groups are
strictly more conservative than the gait's overlapping swing windows.

We compute trajectories ourselves rather than calling the strategy
because at ``stride = 0`` the strategy degenerates: PEP equals AEP
equals nominal, the stance Bezier collapses to a constant, and the
swing arc reduces to a degenerate hop. Owning the trajectory math
here keeps the strategy contract clean while reusing the strategy's
*schedule* (phase offsets and duty factor) for free.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Mapping

from .clock import LEG_NAMES, PhaseOffsets
from .gaits.base import identity_y_sign, swing_arc


__all__ = [
    "LegOutput",
    "DisengagementController",
    "DisengagementState",
]

Vec3 = tuple[float, float, float]


class DisengagementState(Enum):
    RUNNING = "running"
    STAND = "stand"


@dataclass(frozen=True)
class LegOutput:
    """One leg's contribution to a ``LegTargets`` message.

    ``stance=True`` means the foot is on the ground this tick. During
    a group swing the phase value is informational — the in-swing
    leg's fractional progress through its rest-to-rest curve. Legs not
    currently swinging report ``phase=0`` and ``stance=True``.
    """

    foot_target: Vec3
    phase: float
    stance: bool


@dataclass
class _LegSwing:
    """In-progress rest-to-rest swing from origin to nominal."""

    origin: Vec3
    target: Vec3
    duration: float
    elapsed: float = 0.0


class DisengagementController:
    """Group-queue disengagement controller.

    Construct once at engine startup. Call ``begin(...)`` whenever the
    engine enters STOPPING, then ``update(dt)`` each tick until
    ``state == STAND``.
    """

    def __init__(
        self,
        nominal_stance: Mapping[str, Vec3],
        swing_clearance: float,
        swing_width: float,
        controller_dt: float,
        max_foot_speed: float,
        min_swing_time: float,
        max_swing_time: float,
    ) -> None:
        missing = set(LEG_NAMES) - set(nominal_stance)
        if missing:
            raise ValueError(f"nominal_stance missing legs: {sorted(missing)}")
        if max_foot_speed <= 0.0:
            raise ValueError(f"max_foot_speed must be positive; got {max_foot_speed}")
        if min_swing_time <= 0.0:
            raise ValueError(f"min_swing_time must be positive; got {min_swing_time}")
        if max_swing_time < min_swing_time:
            raise ValueError(
                f"max_swing_time {max_swing_time} < min_swing_time {min_swing_time}"
            )

        self._nominal: dict[str, Vec3] = {n: tuple(nominal_stance[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._swing_clearance = swing_clearance
        self._swing_width = swing_width
        self._controller_dt = controller_dt
        self._max_foot_speed = max_foot_speed
        self._min_swing_time = min_swing_time
        self._max_swing_time = max_swing_time

        self._state = DisengagementState.STAND
        self._positions: dict[str, Vec3] = dict(self._nominal)
        self._queue: list[list[str]] = []
        self._swings: dict[str, _LegSwing] = {}

    @property
    def state(self) -> DisengagementState:
        return self._state

    def begin(
        self,
        last_targets: Mapping[str, Vec3],
        swing_flags: Mapping[str, bool],
        phase_offsets: PhaseOffsets,
        duty_factor: float,
        master_phase: float,
    ) -> None:
        """Seed the controller with the legs' current pose at stop time.

        ``phase_offsets`` and ``duty_factor`` come from the active gait
        strategy; ``master_phase`` from the engine's ``GaitClock.master``
        at the moment STOPPING is entered. The ordering of stance
        groups follows the gait's natural lift-off sequence projected
        from that master phase.
        """
        missing = set(LEG_NAMES) - set(last_targets)
        if missing:
            raise ValueError(f"last_targets missing legs: {sorted(missing)}")
        if not (0.0 < duty_factor < 1.0):
            raise ValueError(f"duty_factor must be in (0, 1); got {duty_factor}")
        if not (0.0 <= master_phase < 1.0):
            raise ValueError(f"master_phase must be in [0, 1); got {master_phase}")

        self._positions = {n: tuple(last_targets[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._swings = {}

        offsets = phase_offsets.offsets
        leg_phase: dict[str, float] = {
            n: (master_phase + offsets[n]) % 1.0 for n in LEG_NAMES
        }

        # Airborne group: whichever legs the engine flagged as
        # currently in swing. The engine derives the flags from its
        # own ``_last_stance`` tracking, which is authoritative — no
        # phase-based fallback here (a fallback would also misclassify
        # ripple's exact-boundary case at master_phase = 0, where the
        # touchdown phase value falls on the swing/stance cutoff).
        swing_group = sorted(n for n in LEG_NAMES if swing_flags.get(n, False))

        # Stance groups: bucket by exact offset value, then order
        # buckets by descending current phase. Tripod's two triples
        # come out as two parallel groups; ripple/wave's six singletons
        # come out as up to six groups in gait-natural lift-off order.
        remaining = [n for n in LEG_NAMES if n not in set(swing_group)]
        by_offset: dict[float, list[str]] = {}
        for n in remaining:
            by_offset.setdefault(offsets[n], []).append(n)
        stance_groups = sorted(
            by_offset.values(),
            key=lambda grp: -leg_phase[grp[0]],
        )

        queue: list[list[str]] = []
        if swing_group:
            queue.append(swing_group)
        queue.extend(stance_groups)

        self._queue = [
            grp for grp in queue if any(not self._is_at_nominal(n) for n in grp)
        ]
        if not self._queue:
            self._state = DisengagementState.STAND
            return

        self._state = DisengagementState.RUNNING
        self._start_head_group()

    def update(self, dt: float) -> dict[str, LegOutput]:
        if self._state is DisengagementState.STAND:
            return self._emit_stand()
        return self._tick(dt)

    def _start_head_group(self) -> None:
        # Build swings for every leg in the head group that isn't
        # already at nominal. At-nominal legs would otherwise generate
        # a no-op hop arc — visible twitch, no purpose.
        head = self._queue[0]
        self._swings = {}
        for name in head:
            if self._is_at_nominal(name):
                continue
            duration = self._adaptive_swing_time(
                self._positions[name], self._nominal[name]
            )
            self._swings[name] = _LegSwing(
                origin=self._positions[name],
                target=self._nominal[name],
                duration=duration,
            )

    def _tick(self, dt: float) -> dict[str, LegOutput]:
        out: dict[str, LegOutput] = {}
        for name in LEG_NAMES:
            swing = self._swings.get(name)
            if swing is None:
                out[name] = LegOutput(
                    foot_target=self._positions[name], phase=0.0, stance=True
                )
                continue
            swing.elapsed += dt
            if swing.elapsed >= swing.duration:
                self._positions[name] = swing.target
                del self._swings[name]
                out[name] = LegOutput(
                    foot_target=self._positions[name], phase=0.0, stance=True
                )
            else:
                point = self._swing_point(swing)
                self._positions[name] = point
                out[name] = LegOutput(
                    foot_target=point,
                    phase=swing.elapsed / swing.duration,
                    stance=False,
                )
        if not self._swings:
            self._queue.pop(0)
            if not self._queue:
                self._state = DisengagementState.STAND
            else:
                self._start_head_group()
        return out

    def _emit_stand(self) -> dict[str, LegOutput]:
        return {
            n: LegOutput(foot_target=self._nominal[n], phase=0.0, stance=True)
            for n in LEG_NAMES
        }

    def _adaptive_swing_time(self, origin: Vec3, target: Vec3) -> float:
        # Planar distance only — vertical lift is handled by the apex
        # cap, not by stretching the time budget.
        dx = target[0] - origin[0]
        dy = target[1] - origin[1]
        distance = math.hypot(dx, dy)
        raw = distance / self._max_foot_speed
        if raw < self._min_swing_time:
            return self._min_swing_time
        if raw > self._max_swing_time:
            return self._max_swing_time
        return raw

    def _swing_point(self, swing: _LegSwing) -> Vec3:
        # Unified apex rule: lift the foot to the higher of
        # (origin_z, target_z + swing_clearance). A grounded leg
        # (origin_z = target_z) lifts the full clearance; an airborne
        # leg already above the apex threshold descends with no extra
        # bounce; a near-floor airborne leg gets a partial lift to the
        # same apex a grounded leg would use. Endpoint velocities are
        # zero so the Bezier decelerates fully at touchdown — landing
        # at the steady-state stance velocity would slam the foot into
        # the floor.
        origin = swing.origin
        target = swing.target
        required_apex_z = max(origin[2], target[2] + self._swing_clearance)
        effective_clearance = required_apex_z - max(origin[2], target[2])
        phase = swing.elapsed / swing.duration
        return swing_arc(
            phase_in_swing=phase,
            swing_origin=origin,
            target=target,
            swing_clearance=effective_clearance,
            swing_width=self._swing_width,
            identity_y_sign=identity_y_sign(target),
            swing_time=swing.duration,
            controller_dt=self._controller_dt,
            swing_origin_velocity=(0.0, 0.0, 0.0),
            swing_target_velocity=(0.0, 0.0, 0.0),
        )

    def _is_at_nominal(self, name: str) -> bool:
        nx, ny, nz = self._nominal[name]
        px, py, pz = self._positions[name]
        # Half-millimetre Manhattan threshold: well above the
        # rest-to-rest swing arc's numerical landing noise, well below
        # any motion the operator would notice.
        return abs(nx - px) + abs(ny - py) + abs(nz - pz) < 5e-4
