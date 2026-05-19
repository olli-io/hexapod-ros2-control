"""Stand -> Gait engagement: continuous-tracking first half-cycle.

The standard tripod / wave / ripple strategies assume the engine has
already been ticking — they compute foot targets from PEP and AEP
points that sit symmetrically around the nominal stance. Coming out of
STAND, every foot is at NOMINAL, so on the first GAIT tick the
strategy would demand a position jump to PEP (swing legs) or AEP
(stance legs), and the body's stance feet would yank the body forward.

The ENGAGING engine state runs one asymmetric first half-cycle through
this controller. The controller holds an internal body-velocity state
``(v_body, omega_body)`` that shadows live ``cmd_vel`` through a
smoothstep envelope. Specifically::

    v_body(t) = cmd_vel(t) · f(τ),      f(τ) = 3τ² − 2τ³
    τ = master_phase / β,               master_phase ∈ [0, β]

``f(τ)`` ramps body velocity from 0 to ``cmd_vel`` across the half
cycle. Its derivative vanishes at both endpoints, so body acceleration
is C1-continuous at the STAND → ENGAGING and ENGAGING → GAIT
boundaries. ``cmd_vel`` is read every tick, so a varying input is
shadowed continuously — no snapshot.

Foot trajectories are then driven by this internal velocity:

- **Stance feet** integrate ``−v_leg · dt`` each tick. This is the
  physical motion of a planted foot (foot fixed in world, body moves
  at ``+v_body``). Body-frame foot velocity equals ``−v_body`` by
  construction at every instant.
- **Swing feet** are re-planned every tick: the swing arc runs from
  the leg's lift-off snapshot to the *live* AEP (computed from the
  current ``v_leg`` and cycle_time), with ``swing_target_velocity =
  −v_leg``. Touchdown therefore matches steady-state stance velocity,
  eliminating the swing → stance body-frame velocity step at GAIT
  handover.

Geometric guarantee for constant ``cmd_vel``: ``∫₀^engage_time f(τ) dt
= ½·engage_time``, so the stance foot's integrated displacement is
``½·v_cmd·β·cycle_time = ½·stride``. The foot lands exactly at PEP at
``master = β``. For varying ``cmd_vel`` the integral tracks the
running command; the foot lands at PEP for the *current* command, with
a small residual offset proportional to how much cmd_vel slewed within
the half cycle. Mid-engagement ``cmd_vel → 0`` bails to STOPPING via
the engine and reuses the existing ``TransitionController``.

Composition with the upstream body-velocity rate limiter
========================================================

``hexa_control`` runs a stateful ``BodyVelocityLimiter`` between
``scale_to_envelope`` and the ``/gait/params`` publish, so the
``cmd_vel`` reaching this controller is itself rate-limited (bounded
``|Δ(v_x, v_y)|`` and ``|Δω_z|`` per tick). Engagement is unaffected
in design: it already reads ``cmd_vel`` live each tick and the
"varying cmd_vel → small residual touchdown offset" regime described
above is now the normal case from ``STAND``, not the exception. The
two shapers compose harmlessly — the limiter ramps ``cmd_vel`` toward
the user demand; this controller's smoothstep further shapes its
internal ``v_body`` against whatever the limiter is currently
publishing. The composed ramp still converges by ``master = β``. The
limiter resets to zero on every edge that leaves the walking set
(``{engaging, gait}``), so ``begin()`` always sees a zero-velocity
upstream state.
"""

from __future__ import annotations

import math
from enum import Enum
from typing import Mapping

from .clock import LEG_NAMES
from .gaits.base import LegContext, Strategy, identity_y_sign, swing_arc
from .transition import LegOutput


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
    cmd_vel at handoff, and vanishing acceleration at both endpoints
    (so body acceleration is C1 across the STAND / ENGAGING / GAIT
    boundaries).
    """
    if tau <= 0.0:
        return 0.0
    if tau >= 1.0:
        return 1.0
    return tau * tau * (3.0 - 2.0 * tau)


class EngagementController:
    """First half-cycle of the gait with continuous velocity tracking.

    Construct once at engine startup. Call ``begin(strategy,
    leg_contexts)`` whenever the engine enters ENGAGING, then
    ``update(dt, v_cmd_xy, omega_cmd)`` each tick until ``state is
    DONE``. The ``max_body_*_accel`` parameters are not used directly
    by the smoothstep envelope — they exist as soft caps consulted by
    callers that want to size cmd_vel slew externally. The peak body
    acceleration produced by this controller is
    ``1.5·cmd_vel/engage_time`` (smoothstep derivative max), bounded by
    the gait's own velocity / cycle_time ceiling.
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

        # Per-strategy state, populated in begin().
        self._strategy: Strategy | None = None
        self._leg_contexts: dict[str, LegContext] = {}
        self._initial_swing: dict[str, bool] = {n: False for n in LEG_NAMES}
        self._first_transition_master: dict[str, float] = {n: 0.0 for n in LEG_NAMES}
        self._exit_master = 0.0

        # Live engagement state, evolved by update().
        self._master = 0.0
        self._v_body_x = 0.0
        self._v_body_y = 0.0
        self._omega = 0.0
        self._foot_position: dict[str, Vec3] = dict(self._nominal)
        self._lift_off_position: dict[str, Vec3] = dict(self._nominal)
        self._has_lifted_off: dict[str, bool] = {n: False for n in LEG_NAMES}

    @property
    def state(self) -> EngagementState:
        return self._state

    @property
    def exit_master(self) -> float:
        """Master phase to seed the engine clock with on GAIT handoff."""
        return self._exit_master

    @property
    def v_body(self) -> tuple[float, float, float]:
        """Current internal body velocity ``(v_x, v_y, omega_z)``.

        Exposed for diagnostics and tests; the engine ignores this and
        consumes the per-leg foot targets returned by ``update``.
        """
        return (self._v_body_x, self._v_body_y, self._omega)

    def begin(
        self,
        strategy: Strategy,
        leg_contexts: Mapping[str, LegContext],
    ) -> None:
        """Arm the engagement from a STAND start.

        Resets internal velocity to zero, foot positions to NOMINAL,
        and snapshots only the strategy's per-leg roles (initial swing
        vs initial stance). Velocity tracking is live — there is no
        cmd_vel snapshot.
        """
        missing = set(LEG_NAMES) - set(leg_contexts)
        if missing:
            raise ValueError(f"leg_contexts missing legs: {sorted(missing)}")
        if strategy.duty_factor != self._duty_factor:
            raise ValueError(
                f"strategy duty_factor ({strategy.duty_factor}) does not match"
                f" controller duty_factor ({self._duty_factor})"
            )

        self._strategy = strategy
        self._leg_contexts = dict(leg_contexts)
        offsets = strategy.phase_offsets.offsets

        first: dict[str, float] = {}
        is_swing: dict[str, bool] = {}
        for name in LEG_NAMES:
            o = offsets[name]
            if o < self._swing_end:
                is_swing[name] = True
                first[name] = self._swing_end - o
            else:
                is_swing[name] = False
                first[name] = 1.0 - o
        self._initial_swing = is_swing
        self._first_transition_master = first

        # ``exit_master = β`` is the master phase covered by one half
        # cycle. At this point every leg has finished its first
        # transition: initial-swing legs are at AEP, initial-stance
        # legs are at PEP (both for the live cmd_vel).
        self._exit_master = self._duty_factor

        self._master = 0.0
        self._v_body_x = 0.0
        self._v_body_y = 0.0
        self._omega = 0.0
        self._foot_position = dict(self._nominal)
        self._lift_off_position = dict(self._nominal)
        # Initial-swing legs lift off from NOMINAL right at engagement
        # start; their lift-off snapshot is already correct. Initial-
        # stance legs snapshot when they cross stance -> swing (a non-
        # event during ENGAGING for tripod since that crossing
        # coincides with exit_master).
        self._has_lifted_off = {n: is_swing[n] for n in LEG_NAMES}

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
        cmd_leg_v: dict[str, tuple[float, float]] = {}
        max_cmd_leg_v = 0.0
        for name in LEG_NAMES:
            r_x, r_y, _ = self._leg_contexts[name].mount_xyz
            vx = v_cmd_xy[0] - omega_cmd * r_y
            vy = v_cmd_xy[1] + omega_cmd * r_x
            cmd_leg_v[name] = (vx, vy)
            speed = math.hypot(vx, vy)
            if speed > max_cmd_leg_v:
                max_cmd_leg_v = speed
        cycle_time = self._derive_cycle_time(max_cmd_leg_v)
        stance_time = cycle_time * self._duty_factor

        # 2) Advance master phase using the commanded cycle_time. The
        # clock is the same one GAIT will use; engagement just stops
        # advancing it past exit_master.
        if cycle_time > 0.0:
            self._master = min(
                self._master + dt / cycle_time, self._exit_master
            )

        # 3) Body velocity follows the smoothstep envelope of cmd_vel.
        # tau ∈ [0, 1] runs alongside master phase: tau = master / β.
        # The smoothstep ramps from 0 to 1 with vanishing derivatives
        # at both ends, so body acceleration is continuous at STAND /
        # ENGAGING / GAIT boundaries.
        tau = self._master / self._exit_master if self._exit_master > 0.0 else 1.0
        envelope = _smoothstep(tau)
        self._v_body_x = v_cmd_xy[0] * envelope
        self._v_body_y = v_cmd_xy[1] * envelope
        self._omega = omega_cmd * envelope

        # 4) Per-leg planar velocity at the *internal* body velocity.
        # Foot integration and the swing's target-velocity argument use
        # this — never the raw cmd value.
        body_leg_v: dict[str, tuple[float, float]] = {}
        for name in LEG_NAMES:
            r_x, r_y, _ = self._leg_contexts[name].mount_xyz
            vx = self._v_body_x - self._omega * r_y
            vy = self._v_body_y + self._omega * r_x
            body_leg_v[name] = (vx, vy)

        # 5) Per-leg foot output. Stance integrates; swing replans to
        # the live AEP each tick.
        offsets = self._strategy.phase_offsets.offsets
        out: dict[str, LegOutput] = {}
        for name in LEG_NAMES:
            leg_phase = (self._master + offsets[name]) % 1.0
            transition_m = self._first_transition_master[name]

            if self._initial_swing[name]:
                in_swing = self._master < transition_m
            else:
                # Initial-stance legs lift off at ``transition_m`` and
                # touch down one swing window later. Past that window
                # they must return to stance integration, otherwise the
                # foot stays clipped at the live AEP until exit_master
                # and the GAIT handoff sees a step-shaped position
                # jump. Tripod hides this: every initial-stance leg has
                # ``transition_m = exit_master`` so the post-swing
                # branch is unreachable. Ripple and wave place lift-offs
                # well before exit_master, so the upper bound matters.
                leg_swing_window_master = 1.0 - self._duty_factor
                in_swing = (
                    transition_m
                    <= self._master
                    < transition_m + leg_swing_window_master
                )

            if in_swing:
                if not self._has_lifted_off[name]:
                    # First tick of swing for an initial-stance leg:
                    # snapshot where the foot is now (= integrated PEP)
                    # so the swing trajectory starts cleanly from there.
                    self._lift_off_position[name] = self._foot_position[name]
                    self._has_lifted_off[name] = True

                # Live AEP: where the foot should touch down given the
                # current commanded leg velocity. Migrates outward as
                # cmd_vel ramps in.
                vx_cmd, vy_cmd = cmd_leg_v[name]
                stride_vec = self._stride_vector(vx_cmd, vy_cmd, stance_time)
                nominal = self._nominal[name]
                aep = (
                    nominal[0] + 0.5 * stride_vec[0],
                    nominal[1] + 0.5 * stride_vec[1],
                    nominal[2] + 0.5 * stride_vec[2],
                )

                # Per-leg swing window in master space.
                if self._initial_swing[name]:
                    leg_swing_master = self._master
                    leg_swing_duration_master = transition_m
                else:
                    leg_swing_master = self._master - transition_m
                    leg_swing_duration_master = max(
                        1.0 - self._duty_factor, 1e-9
                    )

                phase_in_swing = (
                    leg_swing_master / leg_swing_duration_master
                    if leg_swing_duration_master > 0.0
                    else 0.0
                )
                phase_in_swing = max(0.0, min(phase_in_swing, 1.0))
                leg_swing_time = leg_swing_duration_master * cycle_time

                # Swing target velocity is the *internal* body-frame
                # leg velocity at touchdown. With the smoothstep
                # envelope this equals ``cmd_vel`` at master = β and
                # ``envelope·cmd_vel`` mid-swing, matching the
                # steady-state stance velocity at touchdown.
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
                    foot_target=foot, phase=leg_phase, stance=False
                )
            else:
                # Stance: integrate the internal body velocity.
                vx_body, vy_body = body_leg_v[name]
                fp = self._foot_position[name]
                self._foot_position[name] = (
                    fp[0] - vx_body * dt,
                    fp[1] - vy_body * dt,
                    fp[2],
                )
                out[name] = LegOutput(
                    foot_target=self._foot_position[name],
                    phase=leg_phase,
                    stance=True,
                )

        if self._master >= self._exit_master:
            self._state = EngagementState.DONE

        return out

    def _derive_cycle_time(self, max_leg_v: float) -> float:
        """Same v -> cycle_time relation as ``Engine._derive_cycle_time``.

        Duplicated here so the engagement is self-contained; the engine
        keeps its own copy for GAIT ticks. Any future change must be
        mirrored in both places.
        """
        if max_leg_v <= 0.0:
            return self._max_cycle_time
        raw = self._stride_length / (max_leg_v * self._duty_factor)
        if raw < self._min_cycle_time:
            return self._min_cycle_time
        if raw > self._max_cycle_time:
            return self._max_cycle_time
        return raw

    def _stride_vector(
        self, v_x: float, v_y: float, stance_time: float
    ) -> Vec3:
        sx = v_x * stance_time
        sy = v_y * stance_time
        magnitude = math.hypot(sx, sy)
        if magnitude > self._stride_length and magnitude > 0.0:
            scale = self._stride_length / magnitude
            sx *= scale
            sy *= scale
        return (sx, sy, 0.0)

    def _emit_nominal_stance(self) -> dict[str, LegOutput]:
        return {
            n: LegOutput(foot_target=self._nominal[n], phase=0.0, stance=True)
            for n in LEG_NAMES
        }
