import math

import pytest

from hexa_gait.gaits._common import METACHRONAL_OFFSETS
from hexa_gait.gaits.base import LegContext, StrideParams
from hexa_gait.gaits.crawl import Crawl


def _leg(nominal=(0.2, 0.1, -0.1)):
    return LegContext(
        name="l_front",
        mount_xyz=(0.083, 0.0575, 0.0),
        mount_yaw=math.radians(30),
        nominal_stance=nominal,
    )


def _stride(stride=(0.0, 0.0, 0.0), duty_factor=Crawl.duty_factor):
    return StrideParams(
        stride_vector=stride,
        cycle_time=1.5,
        duty_factor=duty_factor,
        swing_clearance=0.03,
        swing_width=0.0,
        controller_dt=0.02,
    )


def test_crawl_duty_factor_two_thirds():
    assert Crawl.duty_factor == pytest.approx(2.0 / 3.0)


def test_crawl_uses_metachronal_offsets():
    # Crawl and Ripple share METACHRONAL_OFFSETS; the difference is duty
    # factor only. Offsets are the mirror of lift-off times (lift-off
    # at master = (1 - offset) mod 1), so the realized wave is rear →
    # middle → front per side with the left side half a cycle later.
    assert Crawl.phase_offsets is METACHRONAL_OFFSETS
    o = METACHRONAL_OFFSETS.offsets
    assert math.isclose(o["r_rear"], 0.0)
    assert math.isclose(o["r_middle"], 2.0 / 3.0)
    assert math.isclose(o["r_front"], 1.0 / 3.0)
    assert math.isclose(o["l_rear"], 0.5)
    assert math.isclose(o["l_middle"], 1.0 / 6.0)
    assert math.isclose(o["l_front"], 5.0 / 6.0)


def test_crawl_zero_stride_holds_nominal_xy_at_all_phases():
    leg = _leg()
    stride = _stride()
    crawl = Crawl()
    for phase in (0.0, 0.1, 0.25, 0.3, 0.5, 0.7, 0.99):
        target = crawl.foot_target(phase, stride, leg)
        assert target[0] == pytest.approx(leg.nominal_stance[0], abs=1e-9)
        assert target[1] == pytest.approx(leg.nominal_stance[1], abs=1e-9)


def test_crawl_phase_zero_emits_pep():
    leg = _leg()
    stride_vec = (0.18, 0.0, 0.0)
    stride = _stride(stride=stride_vec)
    crawl = Crawl()
    pep = (
        leg.nominal_stance[0] - 0.5 * stride_vec[0],
        leg.nominal_stance[1] - 0.5 * stride_vec[1],
        leg.nominal_stance[2] - 0.5 * stride_vec[2],
    )
    target = crawl.foot_target(0.0, stride, leg)
    assert target == pytest.approx(pep, abs=1e-9)


def test_crawl_touchdown_phase_emits_aep():
    leg = _leg()
    stride_vec = (0.18, 0.0, 0.0)
    stride = _stride(stride=stride_vec)
    crawl = Crawl()
    aep = (
        leg.nominal_stance[0] + 0.5 * stride_vec[0],
        leg.nominal_stance[1] + 0.5 * stride_vec[1],
        leg.nominal_stance[2] + 0.5 * stride_vec[2],
    )
    # phase = 1 - β = 1/3 is touchdown for crawl.
    swing_end = 1.0 - Crawl.duty_factor
    target = crawl.foot_target(swing_end, stride, leg)
    assert target == pytest.approx(aep, abs=1e-9)


def test_crawl_swing_lifts_above_nominal_z():
    leg = _leg()
    stride = _stride(stride=(0.18, 0.0, 0.0))
    crawl = Crawl()
    # Mid-swing is at half of [0, 1 − β).
    swing_mid = 0.5 * (1.0 - Crawl.duty_factor)
    target = crawl.foot_target(swing_mid, stride, leg)
    assert target[2] > leg.nominal_stance[2] + 1e-6


def test_crawl_stance_stays_at_ground():
    leg = _leg()
    stride = _stride(stride=(0.18, 0.0, 0.0))
    crawl = Crawl()
    swing_end = 1.0 - Crawl.duty_factor
    # Stance covers [swing_end, 1.0)
    for phase in (swing_end, swing_end + 0.1, 0.6, 0.99):
        target = crawl.foot_target(phase, stride, leg)
        assert target[2] == pytest.approx(leg.nominal_stance[2], abs=1e-9)
