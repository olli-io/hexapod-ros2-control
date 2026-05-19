"""Shared helpers for metachronal gaits (ripple, wave) and tripod.

``METACHRONAL_OFFSETS`` is the Wilson posterior→anterior sequence with a
contralateral half-cycle offset. Ripple and wave both use it; only the
duty factor differs between the two.

``phased_foot_target`` is the pure (phase, stride, leg) → body-frame
target shared across all three gaits. PEP / AEP are symmetric about the
nominal stance position; the swing window is ``[0, 1 - β)`` and the
stance window is ``[1 - β, 1)``. Stance is a quartic Bezier from AEP
toward PEP at constant tip velocity; swing is the two-curve
``swing_arc`` from ``base``.
"""

from __future__ import annotations

import numpy as np

from ..clock import PhaseOffsets
from ..trajectory import generate_stance_control_nodes, quartic_bezier
from .base import LegContext, StrideParams, identity_y_sign, swing_arc


__all__ = ["METACHRONAL_OFFSETS", "phased_foot_target"]


# Wilson's posterior → anterior wave on the right side (rear → middle →
# front at offsets 0, 1/3, 2/3) with the contralateral side a half cycle
# out of phase (left side starts at 1/2 and follows the same posterior →
# anterior ordering).
METACHRONAL_OFFSETS = PhaseOffsets(
    offsets={
        "r_rear": 0.0,
        "r_middle": 1.0 / 3.0,
        "r_front": 2.0 / 3.0,
        "l_rear": 1.0 / 2.0,
        "l_middle": 5.0 / 6.0,
        "l_front": 1.0 / 6.0,
    }
)


def phased_foot_target(
    phase: float, stride: StrideParams, leg: LegContext
) -> tuple[float, float, float]:
    """Shared (phase, stride, leg) → foot-target body-frame helper.

    Phase semantics (see ``docs/leg-phases.md``):

    - ``phase = 0`` — lift-off (PEP).
    - ``phase = 1 - duty_factor`` — touchdown (AEP). Stance begins.
    - ``phase = 1`` — back at PEP.

    Stance is a quartic Bezier from AEP toward PEP with constant tip
    velocity opposite the body motion. Swing is the two-curve
    ``swing_arc`` driven by stride / nominal / swing time. β is read
    from ``stride.duty_factor``; the engine fills it from the active
    strategy each tick.
    """
    nominal = np.array(leg.nominal_stance, dtype=np.float64)
    stride_vec = np.array(stride.stride_vector, dtype=np.float64)

    pep = nominal - 0.5 * stride_vec
    aep = nominal + 0.5 * stride_vec

    swing_end = 1.0 - stride.duty_factor
    if phase < swing_end:
        phase_in_swing = phase / swing_end if swing_end > 0.0 else 0.0
        swing_time = stride.cycle_time * (1.0 - stride.duty_factor)
        return swing_arc(
            phase_in_swing=phase_in_swing,
            swing_origin=(float(pep[0]), float(pep[1]), float(pep[2])),
            target=(float(aep[0]), float(aep[1]), float(aep[2])),
            swing_clearance=stride.swing_clearance,
            swing_width=stride.swing_width,
            identity_y_sign=identity_y_sign(leg.nominal_stance),
            swing_time=swing_time,
            controller_dt=stride.controller_dt,
        )

    stance_phase = (phase - swing_end) / stride.duty_factor
    stance_nodes = generate_stance_control_nodes(
        stance_origin=aep, stride_vector=stride_vec
    )
    point = quartic_bezier(stance_nodes, stance_phase)
    return (float(point[0]), float(point[1]), float(point[2]))
