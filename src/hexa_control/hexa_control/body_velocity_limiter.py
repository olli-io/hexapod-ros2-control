"""Stateful body-velocity rate limiter.

Bounds the time derivative of the published ``(v_x, v_y, ω_z)`` so that
upstream sources — joystick releases, ``scale_to_envelope`` unwinding,
Nav2 stops, gait-cap changes on ``/cmd_gait`` — cannot produce step
jumps at the ``/gait/params`` boundary.

The limiter sits in ``hexa_control`` between ``scale_to_envelope`` and
the ``GaitParams`` publish. Every ``step()`` reads its stored previous
state and slews it toward the new target by at most ``max_*_accel·dt``.

Bounds:

- **Linear** — the ``(v_x, v_y)`` change is bounded as a *vector* by
  ``max_linear_accel·dt``. Magnitude is clamped, direction preserved,
  so diagonal stick inputs don't ramp √2× faster than axis-aligned
  ones (the hex is radially symmetric in the body plane).
- **Angular** — ``|Δω_z|`` is bounded by ``max_angular_accel·dt``.

The limiter is *not* a hard safety limit. During ramp-down it allows
brief excursions outside the static ``scale_to_envelope`` window —
that is the whole point of having it (smoothing the envelope-unwind
jump). See ``hexa_control/README.md`` for the downstream effect.
"""

from __future__ import annotations

import math


__all__ = ["BodyVelocityLimiter"]

Vec3 = tuple[float, float, float]


class BodyVelocityLimiter:
    def __init__(
        self, max_linear_accel: float, max_angular_accel: float
    ) -> None:
        if max_linear_accel <= 0.0:
            raise ValueError(
                f"max_linear_accel must be positive, got {max_linear_accel}"
            )
        if max_angular_accel <= 0.0:
            raise ValueError(
                f"max_angular_accel must be positive, got {max_angular_accel}"
            )
        self._max_linear_accel = max_linear_accel
        self._max_angular_accel = max_angular_accel
        self._v_x = 0.0
        self._v_y = 0.0
        self._omega = 0.0

    @property
    def state(self) -> Vec3:
        return (self._v_x, self._v_y, self._omega)

    def reset(self, value: Vec3 = (0.0, 0.0, 0.0)) -> None:
        self._v_x, self._v_y, self._omega = value

    def step(self, target: Vec3, dt: float) -> Vec3:
        if dt <= 0.0:
            return self.state

        tgt_vx, tgt_vy, tgt_omega = target

        dvx = tgt_vx - self._v_x
        dvy = tgt_vy - self._v_y
        d_mag = math.hypot(dvx, dvy)
        max_d_linear = self._max_linear_accel * dt
        if d_mag > max_d_linear:
            scale = max_d_linear / d_mag
            dvx *= scale
            dvy *= scale
        self._v_x += dvx
        self._v_y += dvy

        d_omega = tgt_omega - self._omega
        max_d_omega = self._max_angular_accel * dt
        if d_omega > max_d_omega:
            d_omega = max_d_omega
        elif d_omega < -max_d_omega:
            d_omega = -max_d_omega
        self._omega += d_omega

        return self.state
