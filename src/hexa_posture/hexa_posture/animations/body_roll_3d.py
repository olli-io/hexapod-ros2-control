"""3D body roll — combines ``VerticalBodyRoll`` and
``HorizontalBodyRoll`` with a quarter-cycle phase offset between
them, so the body's translation traces a circle in the (y, z)
plane and the pitch/yaw rotation traces a circle in the
(pitch, yaw) plane, phase-locked to the tripod gait.

Same per-axis curves as the 1D rolls; the horizontal pair is
phase-shifted by ``horizontal_phase_offset`` cycles relative to
the vertical pair. With the default 0.25 the horizontal cosine
becomes a sine, which is what closes the (y, z) and (pitch, yaw)
loops into circles.

Math (with ``φ = master_phase`` in ``[0, 1)`` and
``φ_h = φ + horizontal_phase_offset``):

* ``z     = -z_amplitude     * cos(2π φ)``
* ``pitch = -pitch_amplitude * sin(2π (φ + pitch_phase_offset))``
* ``y     = -y_amplitude     * cos(2π φ_h)``
* ``yaw   =  yaw_amplitude   * sin(2π (φ_h + yaw_phase_offset))``

Pure function — reads only the context, returns a ``BodyPose``.
"""

from dataclasses import dataclass
import math

from ..pose import IDENTITY, BodyPose
from .base import AnimationContext


@dataclass(frozen=True)
class BodyRoll3D:
    z_amplitude: float = 0.02
    """Half-range of the vertical heave (m)."""

    pitch_amplitude: float = 0.1745
    """Half-range of the pitch oscillation (rad). Default ≈ 10°."""

    y_amplitude: float = 0.02
    """Half-range of the lateral sway (m)."""

    yaw_amplitude: float = 0.1745
    """Half-range of the yaw oscillation (rad). Default ≈ 10°."""

    horizontal_phase_offset: float = 0.25
    """Phase shift of the horizontal pair (y, yaw) relative to the
    vertical pair (z, pitch), in cycles. 0.25 traces a circle;
    0.5 collapses it to a diagonal line."""

    pitch_phase_offset: float = 0.0
    """Extra phase shift of the pitch sine, in cycles."""

    yaw_phase_offset: float = 0.0
    """Extra phase shift of the yaw sine, in cycles."""

    def __call__(self, ctx: AnimationContext) -> BodyPose:
        if not ctx.walking or ctx.master_phase is None:
            return IDENTITY
        if ctx.gait_name != "tripod":
            return IDENTITY
        phi = ctx.master_phase
        phi_h = phi + self.horizontal_phase_offset
        z = -self.z_amplitude * math.cos(2.0 * math.pi * phi)
        pitch = -self.pitch_amplitude * math.sin(
            2.0 * math.pi * (phi + self.pitch_phase_offset)
        )
        y = -self.y_amplitude * math.cos(2.0 * math.pi * phi_h)
        yaw = self.yaw_amplitude * math.sin(
            2.0 * math.pi * (phi_h + self.yaw_phase_offset)
        )
        return BodyPose(y=y, z=z, pitch=pitch, yaw=yaw)
