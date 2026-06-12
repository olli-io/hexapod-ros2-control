import math

import pytest

from hexa_gait.gaits.base import LegContext, StrideParams
from hexa_gait.gaits.surf import SURF_OFFSETS, Surf


def _leg(nominal=(0.2, 0.1, -0.1)):
    return LegContext(
        name="l_front",
        mount_xyz=(0.083, 0.0575, 0.0),
        mount_yaw=math.radians(30),
        nominal_stance=nominal,
    )


def _stride(stride=(0.0, 0.0, 0.0), duty_factor=Surf.duty_factor):
    return StrideParams(
        stride_vector=stride,
        cycle_time=0.9,
        duty_factor=duty_factor,
        swing_clearance=0.03,
        swing_width=0.0,
        controller_dt=0.02,
    )


def test_surf_duty_factor_five_eighths():
    assert Surf.duty_factor == pytest.approx(5.0 / 8.0)


def test_surf_lift_offs_cluster_by_tripod():
    # Lift-off happens at master = (1 - offset) mod 1. The two natural
    # tripods lift as staggered groups half a cycle apart; the 1/10
    # within-group stagger keeps mixed airborne triples impossible
    # (margin cliff at 1/8 = β − 1/2 — see the module docstring).
    assert Surf.phase_offsets is SURF_OFFSETS
    lift = {n: (1.0 - o) % 1.0 for n, o in SURF_OFFSETS.offsets.items()}
    assert lift["r_front"] == pytest.approx(4.0 / 5.0)
    assert lift["l_middle"] == pytest.approx(9.0 / 10.0)
    assert lift["r_rear"] == pytest.approx(0.0)
    # Mirror group: each contralateral leg half a cycle later, same
    # internal order.
    for first, mirror in (
        ("r_front", "l_front"),
        ("l_middle", "r_middle"),
        ("r_rear", "l_rear"),
    ):
        assert (lift[mirror] - lift[first]) % 1.0 == pytest.approx(0.5)


def test_surf_airborne_set_never_mixes_tripods_beyond_a_pair():
    # At every master phase the airborne set is a subset of one tripod,
    # plus at most one trailing leg of the other at the group seams.
    tripod_a = {"r_front", "l_middle", "r_rear"}
    tripod_b = {"l_front", "r_middle", "l_rear"}
    swing_end = 1.0 - Surf.duty_factor
    for i in range(4800):
        master = (i + 0.5) / 4800
        airborne = {
            n
            for n, o in SURF_OFFSETS.offsets.items()
            if (master + o) % 1.0 < swing_end
        }
        in_a = len(airborne & tripod_a)
        in_b = len(airborne & tripod_b)
        assert min(in_a, in_b) <= 1, (master, airborne)


def test_surf_zero_stride_holds_nominal_xy_at_all_phases():
    leg = _leg()
    stride = _stride()
    surf = Surf()
    for phase in (0.0, 0.1, 0.25, 0.5, 0.7, 0.99):
        target = surf.foot_target(phase, stride, leg)
        assert target[0] == pytest.approx(leg.nominal_stance[0], abs=1e-9)
        assert target[1] == pytest.approx(leg.nominal_stance[1], abs=1e-9)


def test_surf_phase_zero_emits_pep():
    leg = _leg()
    stride_vec = (0.18, 0.0, 0.0)
    stride = _stride(stride=stride_vec)
    surf = Surf()
    pep = (
        leg.nominal_stance[0] - 0.5 * stride_vec[0],
        leg.nominal_stance[1] - 0.5 * stride_vec[1],
        leg.nominal_stance[2] - 0.5 * stride_vec[2],
    )
    target = surf.foot_target(0.0, stride, leg)
    assert target == pytest.approx(pep, abs=1e-9)


def test_surf_touchdown_phase_emits_aep():
    leg = _leg()
    stride_vec = (0.18, 0.0, 0.0)
    stride = _stride(stride=stride_vec)
    surf = Surf()
    aep = (
        leg.nominal_stance[0] + 0.5 * stride_vec[0],
        leg.nominal_stance[1] + 0.5 * stride_vec[1],
        leg.nominal_stance[2] + 0.5 * stride_vec[2],
    )
    # phase = 1 - β = 3/8 is touchdown for surf.
    swing_end = 1.0 - Surf.duty_factor
    target = surf.foot_target(swing_end, stride, leg)
    assert target == pytest.approx(aep, abs=1e-9)


def test_surf_stance_stays_at_ground():
    leg = _leg()
    stride = _stride(stride=(0.18, 0.0, 0.0))
    surf = Surf()
    swing_end = 1.0 - Surf.duty_factor
    for phase in (swing_end, swing_end + 0.1, 0.7, 0.99):
        target = surf.foot_target(phase, stride, leg)
        assert target[2] == pytest.approx(leg.nominal_stance[2], abs=1e-9)
