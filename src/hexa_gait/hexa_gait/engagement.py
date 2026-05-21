"""Stand -> Gait engagement and Paused -> Gait resume.

The standard tripod / wave / ripple strategies assume the engine has
already been ticking — they compute foot targets from PEP and AEP
points that sit symmetrically around the nominal stance. Coming out of
STAND, every foot is at NOMINAL, so on the first GAIT tick the
strategy would demand a position jump to PEP (swing legs) or AEP
(stance legs), and the body's stance feet would yank the body forward.

The same per-leg state machine handles two entry points:

- **engage mode** (``begin``) — STAND → GAIT. Runs one full master
  cycle 0 → 1; smoothstep envelope ramps body velocity from zero;
  hands off at master = 1.0 (≡ 0.0 in the modular clock).
- **resume mode** (``begin_resume``) — PAUSED → GAIT. Runs from the
  paused ``master_phase`` until every leg has completed its first
  post-resume swing (i.e. crossed into GAIT_LIKE). No smoothstep — the
  upstream ``BodyVelocityLimiter`` already ramps ``cmd_vel`` from zero,
  and the body velocity matches ``cmd_vel`` directly. The previously-
  airborne legs swing from their lowered positions (snapshot at pause
  time) up to the live AEP, merging with the strategy curve at
  touchdown. Previously-stance legs integrate stance until their phase
  wraps to 0, then swing the same way.

Each leg passes through three per-leg states:

- **INITIAL_STANCE** — leg is on the ground and has not yet started
  its first swing. Only initial-stance legs (offset ≥ swing_end) start
  here. The foot is integrated by ``-v_body·dt`` each tick, so it stays
  fixed in world while the body moves forward. The body-frame foot
  drifts backward from NOMINAL by whatever fraction of the smoothstep
  envelope has fired by the leg's lift-off master.
- **INITIAL_SWING** — leg is in its first swing of the engagement.
  Initial-swing legs (offset < swing_end) enter this state at master = 0
  from NOMINAL; initial-stance legs enter at master = 1 − offset from
  their integrated position. The swing arc retargets to the *live* AEP
  each tick (based on the current ``v_cmd``) so the touchdown matches
  steady-state stance velocity.
- **GAIT_LIKE** — leg has completed its first swing. Mirrors what the
  GAIT engine state does: swing legs follow the strategy's swing curve
  (with the current ``v_cmd``-derived stride); stance legs integrate
  the internal body velocity from their current body-frame position.
  Stance is therefore history-dependent rather than rebuilt from
  instantaneous stride. A leg that would do a second swing within one
  cycle (e.g. ripple's ``l_front`` between master 5/6 and 1.0) follows
  the strategy for that second swing too. Continuity at the engagement
  → GAIT boundary is exact: both sides use the same split, so the
  engine seeds its own ``StanceIntegrator`` from the engagement
  controller's last per-leg foot positions and integration continues
  uninterrupted across master = 1.0.

In engage mode, engagement ends at master = 1.0 (≡ 0.0 in the modular
clock). By that point every leg has finished its first swing from
NOMINAL and sits on its GAIT-expected curve, so the handoff carries no
position step.

In resume mode, the controller ends once every leg has crossed into
GAIT_LIKE (entered the strategy-driven branch). The engine reseats the
gait clock from ``exit_master`` so GAIT continues from the resumed
phase.

Body-velocity envelope (engage mode only)
=========================================

``v_body(t) = cmd_vel(t) · smoothstep(τ)`` with ``τ = master / W``,
where ``W`` is the **earliest first-touchdown master** across all legs:

    W = min over initial-swing legs of (swing_end − offset)

For tripod ``W = 0.5``; for ripple and wave ``W = 1/6``. ``W`` is the
horizon over which body velocity must reach ``cmd_vel`` so that every
post-touchdown leg sees the steady-state body velocity and integrated
stance matches the strategy's Bezier-driven stance. Past master = W the
envelope is pinned at 1.0 for the remainder of the cycle.

``cmd_vel`` is read every tick, so a varying input is shadowed
continuously — no snapshot. Mid-engagement ``cmd_vel → 0`` bails to
PAUSING via the engine and hands off to the ``PauseController``.
Resume mode disables the envelope (held at 1.0).

Composition with the upstream body-velocity filter
==================================================

``hexa_control`` runs a vectorial rate-cap ``BodyVelocityLimiter`` on
the output of ``scale_to_envelope``, so the ``cmd_vel`` reaching this
controller is already acceleration-bounded. The limiter resets to zero
on edges that leave the walking set, so ``begin()`` always sees a
zero-velocity upstream state.
"""

from __future__ import annotations

import math
from enum import Enum
from typing import Mapping

from .clock import LEG_NAMES
from .gaits.base import (
    LegContext,
    Strategy,
    StrideParams,
    derive_cycle_time,
    identity_y_sign,
    live_aep,
    per_leg_planar_velocity,
    stride_vector,
    swing_arc,
)
from .pause import LegOutput


__all__ = [
    "EngagementController",
    "EngagementState",
]

Vec3 = tuple[float, float, float]


class EngagementState(Enum):
    IDLE = "idle"
    ENGAGING = "engaging"
    DONE = "done"


def _smoothstep(tau: float) -> float:
    """``f(τ) = 3τ² − 2τ³`` clamped to [0, 1].

    Standard smoothstep / Hermite-3 polynomial. ``f(0) = f'(0) = f(1) =
    f'(1) − 1 = 0``: matches zero velocity at engagement start, full
    cmd_vel at the saturation horizon, and vanishing acceleration at
    both endpoints (so body acceleration is C1 across the STAND /
    ENGAGING / GAIT boundaries).
    """
    if tau <= 0.0:
        return 0.0
    if tau >= 1.0:
        return 1.0
    return tau * tau * (3.0 - 2.0 * tau)


class EngagementController:
    """One full cycle of engagement with continuous velocity tracking.

    Construct once at engine startup. Call ``begin(strategy,
    leg_contexts)`` whenever the engine enters ENGAGING, then
    ``update(dt, v_cmd_xy, omega_cmd)`` each tick until ``state is
    DONE``.
    """

    def __init__(
        self,
        nominal_stance: Mapping[str, Vec3],
        stride_length: float,
        min_cycle_time: float,
        max_cycle_time: float,
        duty_factor: float,
        swing_clearance: float,
        swing_width: float,
        controller_dt: float,
    ) -> None:
        missing = set(LEG_NAMES) - set(nominal_stance)
        if missing:
            raise ValueError(f"nominal_stance missing legs: {sorted(missing)}")

        self._nominal: dict[str, Vec3] = {n: tuple(nominal_stance[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._stride_length = stride_length
        self._min_cycle_time = min_cycle_time
        self._max_cycle_time = max_cycle_time
        self._duty_factor = duty_factor
        self._swing_end = 1.0 - duty_factor
        self._swing_clearance = swing_clearance
        self._swing_width = swing_width
        self._controller_dt = controller_dt

        self._state = EngagementState.IDLE
        # "engage" — STAND → GAIT (full master cycle, smoothstep envelope).
        # "resume" — PAUSED → GAIT (per-leg first-swing completion, no
        # envelope, seeded from paused master and last_targets).
        self._mode: str = "engage"

        # Per-strategy state, populated in begin() / begin_resume().
        self._strategy: Strategy | None = None
        self._leg_contexts: dict[str, LegContext] = {}
        self._is_initial_swing: dict[str, bool] = {n: False for n in LEG_NAMES}
        self._first_lift_off_master: dict[str, float] = {n: 0.0 for n in LEG_NAMES}
        self._first_touchdown_master: dict[str, float] = {n: 0.0 for n in LEG_NAMES}
        # Horizon at which the smoothstep envelope saturates. Set to the
        # earliest first touchdown across legs so every post-touchdown
        # leg sees v_body = v_cmd. Unused in resume mode.
        self._smoothstep_window = 1.0

        # Live engagement state, evolved by update(). In engage mode
        # ``_master`` is clamped to ``[0, 1]`` inclusive; the upper end is
        # the engine's GAIT handoff point (handed off as 0.0 via the
        # ``exit_master`` property, since the clock is modular). In resume
        # mode the clamp is removed — ``_master`` can exceed 1.0 while
        # the slowest leg finishes its first post-resume swing.
        self._master = 0.0
        self._v_body_x = 0.0
        self._v_body_y = 0.0
        self._omega = 0.0
        self._foot_position: dict[str, Vec3] = dict(self._nominal)
        self._lift_off_position: dict[str, Vec3] = dict(self._nominal)
        self._has_lifted_off: dict[str, bool] = {n: False for n in LEG_NAMES}
        # Resume-mode end condition: True when a leg has crossed into its
        # GAIT_LIKE branch at least once since ``begin_resume`` was called.
        # Unused in engage mode (which terminates on master >= 1.0).
        self._has_completed_first_swing: dict[str, bool] = {
            n: False for n in LEG_NAMES
        }

    @property
    def state(self) -> EngagementState:
        return self._state

    @property
    def exit_master(self) -> float:
        """Master phase to seed the engine clock with on GAIT handoff.

        Modular wraparound: in engage mode ``_master`` clamps to 1.0 at
        DONE so ``_master % 1.0 == 0.0`` — GAIT picks up at the start
        of the next cycle. In resume mode ``_master`` sits somewhere in
        ``[master_phase, master_phase + 1]`` at DONE so the wraparound
        yields the resumed cycle position. Both branches collapse to
        the same expression because the wraparound contract is the same.
        """
        return self._master % 1.0

    @property
    def v_body(self) -> tuple[float, float, float]:
        """Current internal body velocity ``(v_x, v_y, omega_z)``.

        Exposed for diagnostics and tests; the engine ignores this and
        consumes the per-leg foot targets returned by ``update``.
        """
        return (self._v_body_x, self._v_body_y, self._omega)

    @property
    def smoothstep_window(self) -> float:
        """Master horizon over which the body-velocity smoothstep ramps.

        Equals the earliest first-touchdown master across all legs:
        ``min(swing_end − offset)`` over initial-swing legs. Exposed for
        tests; the engine itself does not need it.
        """
        return self._smoothstep_window

    def begin(
        self,
        strategy: Strategy,
        leg_contexts: Mapping[str, LegContext],
    ) -> None:
        """Arm the engagement from a STAND start.

        Resets internal velocity to zero, foot positions to NOMINAL,
        and snapshots only the strategy's per-leg first-swing windows.
        Velocity tracking is live — there is no cmd_vel snapshot.
        """
        missing = set(LEG_NAMES) - set(leg_contexts)
        if missing:
            raise ValueError(f"leg_contexts missing legs: {sorted(missing)}")
        if strategy.duty_factor != self._duty_factor:
            raise ValueError(
                f"strategy duty_factor ({strategy.duty_factor}) does not match"
                f" controller duty_factor ({self._duty_factor})"
            )

        self._mode = "engage"
        self._strategy = strategy
        self._leg_contexts = dict(leg_contexts)
        offsets = strategy.phase_offsets.offsets

        is_initial_swing: dict[str, bool] = {}
        first_lift_off: dict[str, float] = {}
        first_touchdown: dict[str, float] = {}
        # 1e-9 tolerance covers the float artefacts when the offset and
        # swing_end share a common irrational like 1/3: ripple's
        # ``r_middle`` (offset 1/3) and ``swing_end`` (1 − 2/3) differ
        # by one ULP. A leg sitting exactly at the boundary is at AEP
        # at master = 0 — that is, in stance, not in swing.
        boundary = self._swing_end - 1e-9
        for name in LEG_NAMES:
            o = offsets[name]
            if o < boundary:
                # Initial-swing: lift off at master = 0 from NOMINAL,
                # touch down at master = swing_end − offset.
                is_initial_swing[name] = True
                first_lift_off[name] = 0.0
                first_touchdown[name] = self._swing_end - o
            else:
                # Initial-stance: stays grounded until phase = 0 (lift
                # off at master = 1 − offset), then swings for one swing
                # window before touchdown.
                is_initial_swing[name] = False
                first_lift_off[name] = 1.0 - o
                first_touchdown[name] = (1.0 - o) + self._swing_end
        self._is_initial_swing = is_initial_swing
        self._first_lift_off_master = first_lift_off
        self._first_touchdown_master = first_touchdown

        # Smoothstep saturates at the earliest first touchdown so every
        # leg that enters GAIT_LIKE sees v_body = v_cmd. For tripod that
        # is master = 0.5; for ripple and wave it is master = 1/6 (only
        # one or two initial-swing legs, the largest offset being 1/6).
        self._smoothstep_window = min(first_touchdown.values())

        self._master = 0.0
        self._v_body_x = 0.0
        self._v_body_y = 0.0
        self._omega = 0.0
        self._foot_position = dict(self._nominal)
        self._lift_off_position = dict(self._nominal)
        # Initial-swing legs lift off from NOMINAL at master = 0, so
        # their lift-off snapshot is already correct. Initial-stance
        # legs snapshot when they cross INITIAL_STANCE -> INITIAL_SWING.
        self._has_lifted_off = {n: is_initial_swing[n] for n in LEG_NAMES}
        self._has_completed_first_swing = {n: False for n in LEG_NAMES}

        self._state = EngagementState.ENGAGING

    def begin_resume(
        self,
        strategy: Strategy,
        leg_contexts: Mapping[str, LegContext],
        last_targets: Mapping[str, Vec3],
        prev_swing_flags: Mapping[str, bool],
        master_phase: float,
    ) -> None:
        """Arm the engagement from a PAUSED start.

        Seeds the per-leg state machine to resume the gait from the
        paused ``master_phase``. Previously-airborne legs (``prev_swing_
        flags[n] == True``) are treated as INITIAL_SWING legs whose
        lift-off snapshot is the lowered foot position; their merge arc
        sweeps from there back up to the live AEP. Previously-stance
        legs are INITIAL_STANCE: they integrate stance from the paused
        position until their phase wraps to 0, then swing through one
        normal swing window. The smoothstep envelope is disabled —
        ``BodyVelocityLimiter`` in ``hexa_control`` already ramps
        ``cmd_vel`` from zero on the PAUSED → RESUMING edge.
        """
        missing = set(LEG_NAMES) - set(leg_contexts)
        if missing:
            raise ValueError(f"leg_contexts missing legs: {sorted(missing)}")
        missing = set(LEG_NAMES) - set(last_targets)
        if missing:
            raise ValueError(f"last_targets missing legs: {sorted(missing)}")
        if strategy.duty_factor != self._duty_factor:
            raise ValueError(
                f"strategy duty_factor ({strategy.duty_factor}) does not match"
                f" controller duty_factor ({self._duty_factor})"
            )
        if not (0.0 <= master_phase < 1.0):
            raise ValueError(f"master_phase must be in [0, 1); got {master_phase}")

        self._mode = "resume"
        self._strategy = strategy
        self._leg_contexts = dict(leg_contexts)
        offsets = strategy.phase_offsets.offsets

        is_initial_swing: dict[str, bool] = {}
        first_lift_off: dict[str, float] = {}
        first_touchdown: dict[str, float] = {}
        lift_off_position: dict[str, Vec3] = dict(self._nominal)
        has_lifted_off: dict[str, bool] = {}

        for name in LEG_NAMES:
            phase = (master_phase + offsets[name]) % 1.0
            if prev_swing_flags.get(name, False):
                # Was airborne: merge arc starts now from the lowered
                # position, touches down when the cycle reaches swing_end.
                is_initial_swing[name] = True
                first_lift_off[name] = master_phase
                first_touchdown[name] = master_phase + max(
                    0.0, self._swing_end - phase
                )
                lift_off_position[name] = tuple(last_targets[name])  # type: ignore[assignment]
                has_lifted_off[name] = True
            else:
                # Was stance: integrate stance until phase wraps to 0
                # (master += 1 - phase), then swing through one swing
                # window.
                is_initial_swing[name] = False
                first_lift_off[name] = master_phase + (1.0 - phase)
                first_touchdown[name] = first_lift_off[name] + self._swing_end
                has_lifted_off[name] = False

        self._is_initial_swing = is_initial_swing
        self._first_lift_off_master = first_lift_off
        self._first_touchdown_master = first_touchdown
        self._smoothstep_window = 1.0  # unused in resume mode

        self._master = master_phase
        self._v_body_x = 0.0
        self._v_body_y = 0.0
        self._omega = 0.0
        self._foot_position = {n: tuple(last_targets[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._lift_off_position = lift_off_position
        self._has_lifted_off = has_lifted_off
        self._has_completed_first_swing = {n: False for n in LEG_NAMES}

        self._state = EngagementState.ENGAGING

    def update(
        self,
        dt: float,
        v_cmd_xy: tuple[float, float],
        omega_cmd: float,
    ) -> dict[str, LegOutput]:
        if self._state is EngagementState.IDLE:
            return self._emit_nominal_stance()
        assert self._strategy is not None

        # 1) Compute per-leg planar velocity *from the commanded body
        # velocity*. ``cycle_time`` derivation must use the same input
        # GAIT uses so the clock advances coherently across the
        # engagement / GAIT boundary.
        cmd_leg_v = per_leg_planar_velocity(
            self._leg_contexts, v_cmd_xy, omega_cmd
        )
        max_cmd_leg_v = max(
            (math.hypot(vx, vy) for vx, vy in cmd_leg_v.values()),
            default=0.0,
        )
        cycle_time = derive_cycle_time(
            max_cmd_leg_v,
            self._stride_length,
            self._duty_factor,
            self._min_cycle_time,
            self._max_cycle_time,
        )
        stance_time = cycle_time * self._duty_factor

        # 2) Advance master phase using the commanded cycle_time. In
        # engage mode the clock clamps at master = 1.0 (the GAIT handoff
        # point). In resume mode the clock advances freely — the
        # controller terminates on per-leg "first swing complete" flags,
        # not a master horizon.
        if cycle_time > 0.0:
            advanced = self._master + dt / cycle_time
            if self._mode == "engage":
                self._master = min(advanced, 1.0)
            else:
                self._master = advanced

        # 3) Body velocity. In engage mode it follows the smoothstep
        # envelope of cmd_vel until master >= smoothstep_window, after
        # which the envelope holds at 1.0. The envelope's vanishing
        # derivative at τ = 1 keeps body acceleration C1 across the
        # saturation point and the STAND / ENGAGING / GAIT boundaries.
        # In resume mode the envelope is pinned to 1.0 — BodyVelocityLimiter
        # in hexa_control already smoothed the PAUSED → RESUMING edge.
        if (
            self._mode == "engage"
            and self._smoothstep_window > 0.0
            and self._master < self._smoothstep_window
        ):
            tau = self._master / self._smoothstep_window
            envelope = _smoothstep(tau)
        else:
            envelope = 1.0
        self._v_body_x = v_cmd_xy[0] * envelope
        self._v_body_y = v_cmd_xy[1] * envelope
        self._omega = omega_cmd * envelope

        # 4) Per-leg planar velocity at the *internal* body velocity.
        # Used by INITIAL_STANCE foot integration and by the swing
        # arc's target-velocity argument.
        body_leg_v = per_leg_planar_velocity(
            self._leg_contexts,
            (self._v_body_x, self._v_body_y),
            self._omega,
        )

        # 5) Per-leg output.
        offsets = self._strategy.phase_offsets.offsets
        out: dict[str, LegOutput] = {}
        for name in LEG_NAMES:
            phase = (self._master + offsets[name]) % 1.0
            first_lift_off = self._first_lift_off_master[name]
            first_touchdown = self._first_touchdown_master[name]

            if self._master >= first_touchdown:
                # GAIT_LIKE: swing legs follow the strategy's swing
                # curve; stance legs integrate the internal body
                # velocity from their current body-frame position. The
                # split mirrors what GAIT itself does, so the engagement
                # → GAIT handoff is consistent: stance is history-
                # dependent on both sides of master = 1.0. Under steady
                # cmd_vel (smoothstep saturated by first_touchdown by
                # construction) the integration reproduces the closed-
                # form stance Bezier; under varying cmd_vel it removes
                # the slip the closed form would otherwise inject.
                in_stance = phase >= self._swing_end
                if in_stance:
                    vx_body, vy_body = body_leg_v[name]
                    fp = self._foot_position[name]
                    self._foot_position[name] = (
                        fp[0] - vx_body * dt,
                        fp[1] - vy_body * dt,
                        fp[2],
                    )
                    foot = self._foot_position[name]
                else:
                    vx_cmd, vy_cmd = cmd_leg_v[name]
                    stride_vec = stride_vector(
                        vx_cmd, vy_cmd, stance_time, self._stride_length
                    )
                    stride = StrideParams(
                        stride_vector=stride_vec,
                        cycle_time=cycle_time,
                        duty_factor=self._duty_factor,
                        swing_clearance=self._swing_clearance,
                        swing_width=self._swing_width,
                        controller_dt=self._controller_dt,
                    )
                    foot = self._strategy.foot_target(
                        phase, stride, self._leg_contexts[name]
                    )
                    self._foot_position[name] = foot
                out[name] = LegOutput(foot_target=foot, phase=phase, stance=in_stance)
                self._has_completed_first_swing[name] = True
            elif self._master >= first_lift_off:
                # INITIAL_SWING: arc from the leg's lift-off snapshot to
                # the *live* AEP. Initial-stance legs snapshot here on
                # the first tick of swing so the swing starts from
                # wherever stance integration carried the foot.
                if not self._has_lifted_off[name]:
                    self._lift_off_position[name] = self._foot_position[name]
                    self._has_lifted_off[name] = True

                vx_cmd, vy_cmd = cmd_leg_v[name]
                stride_vec = stride_vector(
                    vx_cmd, vy_cmd, stance_time, self._stride_length
                )
                nominal = self._nominal[name]
                aep = live_aep(nominal, stride_vec)

                leg_swing_master = self._master - first_lift_off
                leg_swing_duration_master = first_touchdown - first_lift_off
                phase_in_swing = (
                    leg_swing_master / leg_swing_duration_master
                    if leg_swing_duration_master > 0.0
                    else 0.0
                )
                phase_in_swing = max(0.0, min(phase_in_swing, 1.0))
                leg_swing_time = leg_swing_duration_master * cycle_time

                # Swing target velocity is the *internal* body-frame leg
                # velocity at touchdown. With the smoothstep saturated
                # by ``first_touchdown`` this equals ``-cmd_vel``,
                # matching the steady-state stance velocity GAIT_LIKE
                # picks up with on the next tick.
                vx_body, vy_body = body_leg_v[name]
                foot = swing_arc(
                    phase_in_swing=phase_in_swing,
                    swing_origin=self._lift_off_position[name],
                    target=aep,
                    swing_clearance=self._swing_clearance,
                    swing_width=self._swing_width,
                    identity_y_sign=identity_y_sign(nominal),
                    swing_time=leg_swing_time,
                    controller_dt=self._controller_dt,
                    swing_origin_velocity=(0.0, 0.0, 0.0),
                    swing_target_velocity=(-vx_body, -vy_body, 0.0),
                )
                self._foot_position[name] = foot
                out[name] = LegOutput(
                    foot_target=foot, phase=phase, stance=False
                )
            else:
                # INITIAL_STANCE: integrate the *internal* body velocity.
                # The foot stays planted in world; in the body frame it
                # drifts backward by ∫ v_body·dt. Only initial-stance
                # legs reach this branch (initial-swing legs have
                # ``first_lift_off = 0``, so master ≥ first_lift_off on
                # the very first tick).
                vx_body, vy_body = body_leg_v[name]
                fp = self._foot_position[name]
                self._foot_position[name] = (
                    fp[0] - vx_body * dt,
                    fp[1] - vy_body * dt,
                    fp[2],
                )
                out[name] = LegOutput(
                    foot_target=self._foot_position[name],
                    phase=phase,
                    stance=True,
                )

        if self._mode == "engage":
            if self._master >= 1.0:
                self._state = EngagementState.DONE
        else:
            if all(self._has_completed_first_swing.values()):
                self._state = EngagementState.DONE

        return out

    def _emit_nominal_stance(self) -> dict[str, LegOutput]:
        return {
            n: LegOutput(foot_target=self._nominal[n], phase=0.0, stance=True)
            for n in LEG_NAMES
        }
