"""Gait engine — orchestrates clock, strategy, and transition controller.

The engine is the only stateful component in the gait chain. Strategies
stay pure; the transition controller's state is per-stop and isolated.
The engine itself routes between three modes based on the commanded
body velocity:

- **STAND**     — ``cmd_vel`` is zero. Emit the nominal stance.
- **GAIT**      — ``cmd_vel`` is non-zero. Advance the phase clock and
  evaluate the active strategy.
- **STOPPING**  — ``cmd_vel`` just went zero from a non-zero state. Run
  the ``TransitionController`` ladder to bring all six legs back to
  nominal. If a non-zero ``cmd_vel`` arrives mid-stop, complete the
  transition first, then restart the gait from ``master = 0``
  (per the velocity-mid-stop contract in ``src/hexa_gait/README.md``).

``cycle_time`` is not configured directly. The engine derives it each
GAIT tick from the commanded velocity, ``stride_length``, and
``duty_factor``: faster commands ⇒ shorter cycles at constant stride.
``min_cycle_time`` and ``max_cycle_time`` bound the derivation so the
gait saturates cleanly at the speed ceiling and stays brisk at the
slow end.

The nominal-stance helper ``nominal_stance_from_yaml`` reuses
``hexa_kinematics``'s FK and ``leg_to_body`` so the engine never
duplicates the trig that lives in ``body_transform.leg_to_body``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Mapping

from hexa_kinematics.body_transform import leg_to_body
from hexa_kinematics.joint_config import load_standing_pose
from hexa_kinematics.leg_ik import forward_kinematics
from hexa_kinematics.leg_specs import LEG_NAMES, load_leg_specs

from .clock import GaitClock
from .gaits.base import LegContext, Strategy, StrideParams
from .transition import LegOutput, TransitionController, TransitionState


__all__ = [
    "Engine",
    "EngineConfig",
    "EngineState",
    "LegOutput",
    "build_leg_contexts",
    "nominal_stance_from_yaml",
]

Vec3 = tuple[float, float, float]


class EngineState(Enum):
    STAND = "stand"
    GAIT = "gait"
    STOPPING = "stopping"


@dataclass(frozen=True)
class EngineConfig:
    """Engine-internal knobs, sourced entirely from
    ``hexa_gait/config/gait.yaml``. None of these are on the wire.

    ``stride_length`` and ``min_cycle_time`` / ``max_cycle_time`` define
    the velocity → cycle_time relationship the engine applies each
    GAIT tick.
    """

    stride_length: float
    min_cycle_time: float
    max_cycle_time: float
    duty_factor: float
    step_height: float
    swing_width: float
    controller_dt: float
    recenter_swing_time: float
    cmd_zero_tol: float


def nominal_stance_from_yaml(
    geometry_yaml: str | Path,
    standing_pose_yaml: str | Path,
) -> dict[str, Vec3]:
    """Body-frame foot position per leg at the YAML-defined standing pose.

    Mirrors the math previously inlined in
    ``hexa_bringup/tools/stub_stance_publisher.py:51-62``, but routed
    through ``hexa_kinematics.body_transform.leg_to_body`` so the trig
    lives in exactly one place.
    """
    legs = load_leg_specs(geometry_yaml)
    angles = load_standing_pose(standing_pose_yaml, geometry_yaml)
    return {n: leg_to_body(forward_kinematics(angles, legs[n]), legs[n]) for n in LEG_NAMES}


class Engine:
    """Per-tick gait engine.

    ``update(dt, v_body_xy, omega_z)`` returns one ``LegOutput`` per
    leg. Cold start is ``STAND`` with the last-emitted targets seeded
    to ``nominal_stance``, so the first tick matches the previous stub
    publisher's behaviour exactly. ``cycle_time`` is derived per-tick
    from the commanded velocity and the engine's configured
    ``stride_length`` / ``min_cycle_time`` / ``max_cycle_time``.
    """

    def __init__(
        self,
        config: EngineConfig,
        strategy: Strategy,
        nominal_stance: Mapping[str, Vec3],
        leg_contexts: Mapping[str, LegContext],
    ) -> None:
        missing = set(LEG_NAMES) - set(nominal_stance)
        if missing:
            raise ValueError(f"nominal_stance missing legs: {sorted(missing)}")
        missing = set(LEG_NAMES) - set(leg_contexts)
        if missing:
            raise ValueError(f"leg_contexts missing legs: {sorted(missing)}")

        self._config = config
        self._strategy = strategy
        self._nominal: dict[str, Vec3] = {n: tuple(nominal_stance[n]) for n in LEG_NAMES}  # type: ignore[misc]
        self._legs: dict[str, LegContext] = dict(leg_contexts)

        self._clock = GaitClock(strategy.phase_offsets)
        self._transition = TransitionController(
            nominal_stance=self._nominal,
            recenter_swing_time=config.recenter_swing_time,
            swing_clearance=config.step_height,
            swing_width=config.swing_width,
            controller_dt=config.controller_dt,
        )

        self._state = EngineState.STAND
        self._last_targets: dict[str, Vec3] = dict(self._nominal)
        self._last_stance: dict[str, bool] = {n: True for n in LEG_NAMES}

    @property
    def state(self) -> EngineState:
        return self._state

    def update(
        self,
        dt: float,
        v_body_xy: tuple[float, float],
        omega_z: float,
    ) -> dict[str, LegOutput]:
        cmd_zero = self._cmd_is_zero(v_body_xy, omega_z)

        if self._state is EngineState.STAND:
            if not cmd_zero:
                self._state = EngineState.GAIT
                self._clock.reset(0.0)
                return self._tick_gait(dt, v_body_xy, omega_z)
            return self._emit_stand()

        if self._state is EngineState.GAIT:
            if cmd_zero:
                self._state = EngineState.STOPPING
                # _last_stance is True when the leg is on the ground;
                # the controller wants the opposite (True = airborne),
                # so invert here.
                swing_flags = {n: not self._last_stance[n] for n in LEG_NAMES}
                self._transition.begin(self._last_targets, swing_flags)
                return self._tick_transition(dt)
            return self._tick_gait(dt, v_body_xy, omega_z)

        # STOPPING: run the transition ladder to completion. A non-zero
        # cmd arriving mid-stop is honoured only after the transition
        # finishes; the README is explicit about this.
        out = self._tick_transition(dt)
        if self._transition.state is TransitionState.STAND:
            self._state = EngineState.STAND
        return out

    def _cmd_is_zero(self, v_body_xy: tuple[float, float], omega_z: float) -> bool:
        tol = self._config.cmd_zero_tol
        return abs(v_body_xy[0]) < tol and abs(v_body_xy[1]) < tol and abs(omega_z) < tol

    def _emit_stand(self) -> dict[str, LegOutput]:
        return {
            n: LegOutput(foot_target=self._nominal[n], phase=0.0, stance=True)
            for n in LEG_NAMES
        }

    def _tick_gait(
        self,
        dt: float,
        v_body_xy: tuple[float, float],
        omega_z: float,
    ) -> dict[str, LegOutput]:
        duty_factor = self._config.duty_factor
        stride_length = self._config.stride_length

        # Per-leg planar velocity: linear command plus the tangential
        # contribution from body yaw rate. Computed once up front so
        # the max-leg speed can drive a single cycle_time choice.
        leg_velocities: dict[str, tuple[float, float]] = {}
        max_leg_v = 0.0
        for name in LEG_NAMES:
            r_x, r_y, _ = self._legs[name].mount_xyz
            v_x = v_body_xy[0] - omega_z * r_y
            v_y = v_body_xy[1] + omega_z * r_x
            leg_velocities[name] = (v_x, v_y)
            speed = math.hypot(v_x, v_y)
            if speed > max_leg_v:
                max_leg_v = speed

        cycle_time = self._derive_cycle_time(max_leg_v)
        stance_time = cycle_time * duty_factor

        self._clock.advance(dt, cycle_time)
        phases = self._clock.phases()

        out: dict[str, LegOutput] = {}
        for name in LEG_NAMES:
            leg = self._legs[name]
            v_x, v_y = leg_velocities[name]
            stride_vec = self._stride_vector(v_x, v_y, stance_time, stride_length)
            stride = StrideParams(
                stride_vector=stride_vec,
                cycle_time=cycle_time,
                duty_factor=duty_factor,
                swing_clearance=self._config.step_height,
                swing_width=self._config.swing_width,
                controller_dt=self._config.controller_dt,
            )
            target = self._strategy.foot_target(phases[name], stride, leg)
            stance = phases[name] >= (1.0 - duty_factor)
            out[name] = LegOutput(foot_target=target, phase=phases[name], stance=stance)

        self._last_targets = {n: out[n].foot_target for n in LEG_NAMES}
        self._last_stance = {n: out[n].stance for n in LEG_NAMES}
        return out

    def _derive_cycle_time(self, max_leg_v: float) -> float:
        """Pick cycle_time so the fastest leg's stride equals stride_length.

        Clamped to ``[min_cycle_time, max_cycle_time]``. At zero
        ``max_leg_v`` the raw quotient diverges, so we clamp to the
        slow end — the resulting stride is zero anyway because every
        ``v_leg`` is zero.
        """
        cfg = self._config
        if max_leg_v <= 0.0:
            return cfg.max_cycle_time
        raw = cfg.stride_length / (max_leg_v * cfg.duty_factor)
        if raw < cfg.min_cycle_time:
            return cfg.min_cycle_time
        if raw > cfg.max_cycle_time:
            return cfg.max_cycle_time
        return raw

    def _stride_vector(
        self,
        v_x: float,
        v_y: float,
        stance_time: float,
        stride_length: float,
    ) -> Vec3:
        """Per-leg stride displacement, magnitude-clamped to stride_length.

        The clamp matters only when ``max_leg_v`` exceeds the implied
        ceiling (``min_cycle_time`` has clipped ``cycle_time``); below
        saturation the raw stride is already ``≤ stride_length``.
        """
        sx = v_x * stance_time
        sy = v_y * stance_time
        magnitude = math.hypot(sx, sy)
        if magnitude > stride_length and magnitude > 0.0:
            scale = stride_length / magnitude
            sx *= scale
            sy *= scale
        return (sx, sy, 0.0)

    def _tick_transition(self, dt: float) -> dict[str, LegOutput]:
        # Map "in swing during transition" to "not stance" so that if
        # the engine drops back to STAND mid-transition it carries the
        # right grounded flags forward.
        out = self._transition.update(dt)
        self._last_targets = {n: out[n].foot_target for n in LEG_NAMES}
        self._last_stance = {n: out[n].stance for n in LEG_NAMES}
        return out


def build_leg_contexts(
    geometry_yaml: str | Path,
    standing_pose_yaml: str | Path,
) -> dict[str, LegContext]:
    """Build the per-leg ``LegContext`` map the engine needs at init.

    Couples the kinematics' ``LegSpec`` (mount geometry) with the
    YAML-derived nominal stance. Kept here rather than in ``leg_specs``
    because ``LegContext`` is a gait-engine concept.
    """
    legs = load_leg_specs(geometry_yaml)
    nominal = nominal_stance_from_yaml(geometry_yaml, standing_pose_yaml)
    return {
        n: LegContext(
            name=n,
            mount_xyz=legs[n].mount_xyz,
            mount_yaw=legs[n].mount_yaw,
            nominal_stance=nominal[n],
        )
        for n in LEG_NAMES
    }
