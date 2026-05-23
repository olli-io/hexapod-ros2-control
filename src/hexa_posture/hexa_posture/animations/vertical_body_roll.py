"""Vertical body roll — heaves the body in z while pitching about
the lateral axis, phase-locked to the tripod gait. The whole motion
lives in the vertical (sagittal) plane, hence "vertical".

**Tripod-only.** Gated on ``ctx.gait_name == "tripod"`` and on a
non-None ``ctx.master_phase``; returns IDENTITY otherwise. The
animation rides the gait clock, so it also silences itself when
``ctx.walking`` is False — at zero ``/cmd_vel`` the master phase is
frozen and a static offset on z/pitch is not what the user wants.

Math (with ``φ = master_phase`` in ``[0, 1)``):

* ``z     = -z_amplitude * cos(2π φ)`` — cycle-rate cosine, one
  up/down heave per gait cycle (trough at φ = 0, peak at φ = 0.5).
* ``pitch =  pitch_amplitude * sin(2π (φ + pitch_phase_offset))`` —
  cycle-rate sine. Convention: +pitch about +y dips the front of
  the body (see joy_mapping docstring).

Pure function — reads only the context, returns a ``BodyPose``.
"""

from dataclasses import dataclass
import math

from ..pose import IDENTITY, BodyPose
from .base import AnimationContext


@dataclass(frozen=True)
class VerticalBodyRoll:
    z_amplitude: float = 0.02
    """Half-range of the vertical heave (m). Body z oscillates in
    ``[-z_amplitude, +z_amplitude]``."""

    pitch_amplitude: float = 0.1745
    """Half-range of the pitch oscillation (rad). Default ≈ 10°.
    Composes additively with the user pose, so stay well inside
    ``PoseLimits.pitch`` (0.30 rad)."""

    pitch_phase_offset: float = 0.0
    """Phase shift of the pitch sine, in cycles. Tweak if the
    fore/aft tilt is misaligned with the gait timing."""

    def __call__(self, ctx: AnimationContext) -> BodyPose:
        if not ctx.walking or ctx.master_phase is None:
            return IDENTITY
        if ctx.gait_name != "tripod":
            return IDENTITY
        phi = ctx.master_phase
        z = -self.z_amplitude * math.cos(2.0 * math.pi * phi)
        pitch = -self.pitch_amplitude * math.sin(
            2.0 * math.pi * (phi + self.pitch_phase_offset)
        )
        return BodyPose(z=z, pitch=pitch)
