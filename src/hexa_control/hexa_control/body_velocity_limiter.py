"""First-order low-pass on the published body velocity.

Each ``step()`` slews the stored ``(v_x, v_y, ω_z)`` toward the target
with ``α = 1 − exp(−dt/τ); v ← v + α·(target − v)``. Separate τ for
the linear pair and the angular axis. Tau (not an accel cap) so the
filter has no derivative discontinuity, self-scales with command size,
and composes on the same axis as the engagement controller's
smoothstep.

Not a hard safety limit — its job is to absorb the per-tick step that
``scale_to_envelope`` can produce on yaw-release / gait-cap change /
Nav2 stop. Upstream caps bound the target; α ∈ (0, 1) bounds the
output to a convex combination of state and target.
"""

from __future__ import annotations

import math


__all__ = ["BodyVelocityLimiter"]

Vec3 = tuple[float, float, float]


class BodyVelocityLimiter:
    def __init__(
        self,
        tau_linear: float,
        tau_angular: float,
        snap_tol_linear: float = 1.0e-3,
        snap_tol_angular: float = 1.0e-3,
    ) -> None:
        if tau_linear <= 0.0:
            raise ValueError(
                f"tau_linear must be positive, got {tau_linear}"
            )
        if tau_angular <= 0.0:
            raise ValueError(
                f"tau_angular must be positive, got {tau_angular}"
            )
        if snap_tol_linear < 0.0:
            raise ValueError(
                f"snap_tol_linear must be non-negative, got {snap_tol_linear}"
            )
        if snap_tol_angular < 0.0:
            raise ValueError(
                f"snap_tol_angular must be non-negative, got {snap_tol_angular}"
            )
        self._tau_linear = tau_linear
        self._tau_angular = tau_angular
        self._snap_tol_linear = snap_tol_linear
        self._snap_tol_angular = snap_tol_angular
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

        # Zero-target axes snap immediately. Asymptotic decay on a
        # release-from-walking leaves the filter state above the gait
        # engine's cmd_zero_tol for hundreds of milliseconds; during
        # that tail GAIT keeps ticking with a shrinking stride and
        # master phase keeps advancing, so legs visibly settle and
        # *new* lift-offs can trigger before cmd_zero ever fires.
        # Smoothing only matters when stepping cmd_vel down to a lower
        # non-zero value — a full release doesn't benefit from being
        # rounded off.
        if tgt_vx == 0.0:
            self._v_x = 0.0
        else:
            alpha_lin = 1.0 - math.exp(-dt / self._tau_linear)
            self._v_x += alpha_lin * (tgt_vx - self._v_x)
            if abs(self._v_x) < self._snap_tol_linear:
                self._v_x = 0.0

        if tgt_vy == 0.0:
            self._v_y = 0.0
        else:
            alpha_lin = 1.0 - math.exp(-dt / self._tau_linear)
            self._v_y += alpha_lin * (tgt_vy - self._v_y)
            if abs(self._v_y) < self._snap_tol_linear:
                self._v_y = 0.0

        if tgt_omega == 0.0:
            self._omega = 0.0
        else:
            alpha_ang = 1.0 - math.exp(-dt / self._tau_angular)
            self._omega += alpha_ang * (tgt_omega - self._omega)
            if abs(self._omega) < self._snap_tol_angular:
                self._omega = 0.0

        return self.state
