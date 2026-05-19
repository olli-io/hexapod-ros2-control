import math

import pytest

from hexa_gait.gaits._common import METACHRONAL_OFFSETS
from hexa_gait.gaits.base import LegContext, StrideParams
from hexa_gait.gaits.wave import Wave


def _leg(nominal=(0.2, 0.1, -0.1)):
    return LegContext(
        name="l_front",
        mount_xyz=(0.083, 0.0575, 0.0),
        mount_yaw=math.radians(30),
        nominal_stance=nominal,
    )


def _stride(stride=(0.0, 0.0, 0.0), duty_factor=Wave.duty_factor):
    return StrideParams(
        stride_vector=stride,
        cycle_time=2.0,
        duty_factor=duty_factor,
        swing_clearance=0.03,
        swing_width=0.0,
        controller_dt=0.02,
    )


def test_wave_duty_factor_five_sixths():
    assert Wave.duty_factor == pytest.approx(5.0 / 6.0)


def test_wave_shares_metachronal_offsets_with_ripple():
    assert Wave.phase_offsets is METACHRONAL_OFFSETS


def test_wave_zero_stride_holds_nominal_xy_at_all_phases():
    leg = _leg()
    stride = _stride()
    wave = Wave()
    for phase in (0.0, 0.05, 0.1, 0.2, 0.5, 0.9, 0.99):
        target = wave.foot_target(phase, stride, leg)
        assert target[0] == pytest.approx(leg.nominal_stance[0], abs=1e-9)
        assert target[1] == pytest.approx(leg.nominal_stance[1], abs=1e-9)


def test_wave_phase_zero_emits_pep():
    leg = _leg()
    stride_vec = (0.18, 0.0, 0.0)
    stride = _stride(stride=stride_vec)
    wave = Wave()
    pep = (
        leg.nominal_stance[0] - 0.5 * stride_vec[0],
        leg.nominal_stance[1] - 0.5 * stride_vec[1],
        leg.nominal_stance[2] - 0.5 * stride_vec[2],
    )
    target = wave.foot_target(0.0, stride, leg)
    assert target == pytest.approx(pep, abs=1e-9)


def test_wave_touchdown_phase_emits_aep():
    leg = _leg()
    stride_vec = (0.18, 0.0, 0.0)
    stride = _stride(stride=stride_vec)
    wave = Wave()
    aep = (
        leg.nominal_stance[0] + 0.5 * stride_vec[0],
        leg.nominal_stance[1] + 0.5 * stride_vec[1],
        leg.nominal_stance[2] + 0.5 * stride_vec[2],
    )
    swing_end = 1.0 - Wave.duty_factor  # 1/6
    target = wave.foot_target(swing_end, stride, leg)
    assert target == pytest.approx(aep, abs=1e-9)


def test_wave_swing_lifts_above_nominal_z():
    leg = _leg()
    stride = _stride(stride=(0.18, 0.0, 0.0))
    wave = Wave()
    swing_mid = 0.5 * (1.0 - Wave.duty_factor)
    target = wave.foot_target(swing_mid, stride, leg)
    assert target[2] > leg.nominal_stance[2] + 1e-6


def test_wave_stance_stays_at_ground():
    leg = _leg()
    stride = _stride(stride=(0.18, 0.0, 0.0))
    wave = Wave()
    swing_end = 1.0 - Wave.duty_factor  # 1/6 ≈ 0.1667
    # Stance covers most of [0, 1) for wave.
    for phase in (swing_end, 0.25, 0.5, 0.75, 0.99):
        target = wave.foot_target(phase, stride, leg)
        assert target[2] == pytest.approx(leg.nominal_stance[2], abs=1e-9)
