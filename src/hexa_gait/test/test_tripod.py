import math

import pytest

from hexa_gait.gaits.base import LegContext, StrideParams
from hexa_gait.gaits.tripod import Tripod


def _leg(nominal=(0.2, 0.1, -0.1)):
    return LegContext(
        name="l_front",
        mount_xyz=(0.083, 0.0575, 0.0),
        mount_yaw=math.radians(30),
        nominal_stance=nominal,
    )


def _stride(stride=(0.0, 0.0, 0.0)):
    return StrideParams(
        stride_vector=stride,
        cycle_time=1.2,
        duty_factor=0.5,
        swing_clearance=0.03,
        swing_width=0.0,
        controller_dt=0.02,
    )


def test_tripod_zero_stride_holds_nominal_xy_at_all_phases():
    leg = _leg()
    stride = _stride()
    tripod = Tripod()
    # With zero stride, PEP = AEP = nominal, so the foot's XY stays at
    # nominal_xy regardless of phase. Z still lifts during swing (the
    # trajectory's mid_z is nominal_z + swing_clearance even when the
    # stride collapses to a point).
    for phase in (0.0, 0.1, 0.25, 0.49, 0.5, 0.75, 0.99):
        target = tripod.foot_target(phase, stride, leg)
        assert target[0] == pytest.approx(leg.nominal_stance[0], abs=1e-9)
        assert target[1] == pytest.approx(leg.nominal_stance[1], abs=1e-9)
    # Stance phase keeps Z at nominal too (no lift).
    for phase in (0.5, 0.7, 0.99):
        assert (
            tripod.foot_target(phase, stride, leg)[2]
            == pytest.approx(leg.nominal_stance[2], abs=1e-9)
        )


def test_tripod_phase_zero_emits_pep():
    leg = _leg()
    stride_vec = (0.18, 0.0, 0.0)
    stride = _stride(stride=stride_vec)
    tripod = Tripod()
    pep = (
        leg.nominal_stance[0] - 0.5 * stride_vec[0],
        leg.nominal_stance[1] - 0.5 * stride_vec[1],
        leg.nominal_stance[2] - 0.5 * stride_vec[2],
    )
    target = tripod.foot_target(0.0, stride, leg)
    assert target == pytest.approx(pep, abs=1e-9)


def test_tripod_touchdown_phase_emits_aep():
    leg = _leg()
    stride_vec = (0.18, 0.0, 0.0)
    stride = _stride(stride=stride_vec)
    tripod = Tripod()
    aep = (
        leg.nominal_stance[0] + 0.5 * stride_vec[0],
        leg.nominal_stance[1] + 0.5 * stride_vec[1],
        leg.nominal_stance[2] + 0.5 * stride_vec[2],
    )
    # Phase = 0.5 is the start of stance (AEP); the stance curve starts here.
    target = tripod.foot_target(0.5, stride, leg)
    assert target == pytest.approx(aep, abs=1e-9)


def test_tripod_swing_lifts_above_nominal_z():
    leg = _leg()
    stride = _stride(stride=(0.18, 0.0, 0.0))
    tripod = Tripod()
    # Mid-swing the foot should be at the apex height (above PEP/AEP z).
    target = tripod.foot_target(0.25, stride, leg)
    assert target[2] > leg.nominal_stance[2] + 1e-6


def test_tripod_stance_stays_at_ground():
    leg = _leg()
    stride = _stride(stride=(0.18, 0.0, 0.0))
    tripod = Tripod()
    # Stance covers [0.5, 1.0); z should equal nominal_z throughout.
    for phase in (0.5, 0.7, 0.9, 0.99):
        target = tripod.foot_target(phase, stride, leg)
        assert target[2] == pytest.approx(leg.nominal_stance[2], abs=1e-9)
