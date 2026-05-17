"""Tripod gait: alternating 3+3, ``duty_factor = 0.5``.

The six legs split into two tripods that alternate between swing and
stance. Tripod A (``l_front``, ``r_middle``, ``l_rear``) lifts off at
``master = 0.0``; tripod B (``r_front``, ``l_middle``, ``r_rear``) lifts
off at ``master = 0.5``. With a 50% duty factor the support polygon
always has exactly three feet on the ground.
"""

from __future__ import annotations

import numpy as np

from ..clock import PhaseOffsets
from ..trajectory import generate_stance_control_nodes, quartic_bezier
from .base import LegContext, StrideParams, identity_y_sign, swing_arc


__all__ = ["TRIPOD_OFFSETS", "Tripod"]


TRIPOD_OFFSETS = PhaseOffsets(
    offsets={
        "l_front": 0.0,
        "r_middle": 0.0,
        "l_rear": 0.0,
        "r_front": 0.5,
        "l_middle": 0.5,
        "r_rear": 0.5,
    }
)


class Tripod:
    """Alternating-tripod strategy.

    Phase semantics, in line with ``docs/leg-phases.md``:

    - ``phase = 0`` — lift-off (PEP). Foot begins the swing.
    - ``phase = 1 - duty_factor`` — touchdown (AEP). Swing ends, stance
      begins. For ``duty_factor = 0.5`` this is ``phase = 0.5``.
    - ``phase = 1`` — back at PEP, next cycle.

    SWING covers ``[0, 1 - duty_factor)``, STANCE covers
    ``[1 - duty_factor, 1)``.
    """

    phase_offsets = TRIPOD_OFFSETS
    duty_factor = 0.5

    def foot_target(
        self, phase: float, stride: StrideParams, leg: LegContext
    ) -> tuple[float, float, float]:
        nominal = np.array(leg.nominal_stance, dtype=np.float64)
        stride_vec = np.array(stride.stride_vector, dtype=np.float64)

        # PEP / AEP are symmetric about the nominal stance position.
        pep = nominal - 0.5 * stride_vec
        aep = nominal + 0.5 * stride_vec

        swing_end = 1.0 - stride.duty_factor
        if phase < swing_end:
            # Swing phase, normalized to [0, 1) for the two-curve arc.
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

        # Stance phase: evenly-spaced quartic Bezier from AEP toward PEP
        # (constant tip velocity opposite the body motion).
        stance_phase = (phase - swing_end) / stride.duty_factor
        stance_nodes = generate_stance_control_nodes(
            stance_origin=aep, stride_vector=stride_vec
        )
        point = quartic_bezier(stance_nodes, stance_phase)
        return (float(point[0]), float(point[1]), float(point[2]))
