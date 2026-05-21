"""Vectorial rate-cap slew on the published body velocity.

Each ``step()`` advances the stored ``(v_x, v_y, ω_z)`` toward the
target by at most ``accel_linear · dt`` on the planar linear vector
and ``accel_angular · dt`` on the yaw scalar. The linear pair is
treated as a vector — diagonal direction reversals traverse the
magnitude-zero point at the same single bounded rate as axis-aligned
ones, with no per-axis coupling artefacts.

Rate-cap (constant max acceleration) rather than a first-order
time-constant low-pass because:

- the slew reaches the target in finite time, so the gait engine's
  ``cmd_zero_tol`` is hit cleanly on a release without any
  special-case snap-to-zero (which previously short-circuited the
  filter the moment any axis hit exactly ``0.0`` — a problem when
  the teleop deadband zeroed an axis mid-traversal of a stick flip,
  producing an unbounded one-tick stop);
- the worst-case body-frame acceleration is bounded symmetrically
  regardless of step size, direction, or whether the target was
  reached by a release, a direction flip, a ``scale_to_envelope`` cap
  change, or a Nav2 stop;
- the tuning unit (m/s² and rad/s²) matches what downstream
  literature and Nav2 dynamic-limiter parameters use.

``accel_linear`` and ``accel_angular`` are exposed as validated
properties so the control node can retune the linear cap on a
``/cmd_gait`` switch (the gait's per-leg velocity ceiling is
gait-specific, so an absolute acceleration cap that feels right on
one gait over- or under-shoots on another — see
``hexa_control/config/control.yaml``).

``snap_tol_*`` clamps any sub-tolerance dribble at the very end of a
ramp so the engine sees an exact zero, not a 1e-15 residue. Keep it
tight (≤ the engine's ``cmd_zero_tol``).
"""

from __future__ import annotations

import math


__all__ = ["BodyVelocityLimiter"]

Vec3 = tuple[float, float, float]


class BodyVelocityLimiter:
    def __init__(
        self,
        accel_linear: float,
        accel_angular: float,
        snap_tol_linear: float = 1.0e-4,
        snap_tol_angular: float = 1.0e-4,
    ) -> None:
        if snap_tol_linear < 0.0:
            raise ValueError(
                f"snap_tol_linear must be non-negative, got {snap_tol_linear}"
            )
        if snap_tol_angular < 0.0:
            raise ValueError(
                f"snap_tol_angular must be non-negative, got {snap_tol_angular}"
            )
        self._accel_linear = self._validate_positive("accel_linear", accel_linear)
        self._accel_angular = self._validate_positive("accel_angular", accel_angular)
        self._snap_tol_linear = snap_tol_linear
        self._snap_tol_angular = snap_tol_angular
        self._v_x = 0.0
        self._v_y = 0.0
        self._omega = 0.0

    @staticmethod
    def _validate_positive(name: str, value: float) -> float:
        if value <= 0.0:
            raise ValueError(f"{name} must be positive, got {value}")
        return value

    @property
    def accel_linear(self) -> float:
        return self._accel_linear

    @accel_linear.setter
    def accel_linear(self, value: float) -> None:
        self._accel_linear = self._validate_positive("accel_linear", value)

    @property
    def accel_angular(self) -> float:
        return self._accel_angular

    @accel_angular.setter
    def accel_angular(self, value: float) -> None:
        self._accel_angular = self._validate_positive("accel_angular", value)

    @property
    def state(self) -> Vec3:
        return (self._v_x, self._v_y, self._omega)

    def reset(self, value: Vec3 = (0.0, 0.0, 0.0)) -> None:
        self._v_x, self._v_y, self._omega = value

    def step(self, target: Vec3, dt: float) -> Vec3:
        if dt <= 0.0:
            return self.state

        tgt_vx, tgt_vy, tgt_omega = target

        # Linear pair: vectorial slew capped at accel_linear * dt.
        dx = tgt_vx - self._v_x
        dy = tgt_vy - self._v_y
        distance = math.hypot(dx, dy)
        max_step_lin = self._accel_linear * dt
        if distance <= max_step_lin:
            self._v_x = tgt_vx
            self._v_y = tgt_vy
        else:
            scale = max_step_lin / distance
            self._v_x += scale * dx
            self._v_y += scale * dy
        if math.hypot(self._v_x, self._v_y) < self._snap_tol_linear:
            self._v_x = 0.0
            self._v_y = 0.0

        # Angular: scalar slew capped at accel_angular * dt.
        d_omega = tgt_omega - self._omega
        max_step_ang = self._accel_angular * dt
        if abs(d_omega) <= max_step_ang:
            self._omega = tgt_omega
        else:
            self._omega += math.copysign(max_step_ang, d_omega)
        if abs(self._omega) < self._snap_tol_angular:
            self._omega = 0.0

        return self.state
