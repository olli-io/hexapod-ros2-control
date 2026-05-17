import math

import pytest

from hexa_gait.clock import LEG_NAMES, GaitClock, PhaseOffsets
from hexa_gait.gaits.tripod import TRIPOD_OFFSETS


def _zero_offsets():
    return PhaseOffsets(offsets={n: 0.0 for n in LEG_NAMES})


def test_advance_wraps_after_one_cycle():
    clock = GaitClock(_zero_offsets())
    clock.advance(0.5, cycle_time=1.0)
    assert clock.master == pytest.approx(0.5)
    clock.advance(0.5, cycle_time=1.0)
    # 1.0 wraps to 0.0.
    assert clock.master == pytest.approx(0.0, abs=1e-12)


def test_reset_seeds_master():
    clock = GaitClock(_zero_offsets())
    clock.reset(0.3)
    assert clock.master == pytest.approx(0.3)


def test_reset_rejects_out_of_range():
    clock = GaitClock(_zero_offsets())
    with pytest.raises(ValueError):
        clock.reset(1.0)
    with pytest.raises(ValueError):
        clock.reset(-0.1)


def test_phases_apply_offsets_modulo_one():
    clock = GaitClock(TRIPOD_OFFSETS)
    phases = clock.phases()
    # Tripod A (offset 0.0) starts at 0.0.
    assert phases["l_front"] == pytest.approx(0.0)
    assert phases["r_middle"] == pytest.approx(0.0)
    assert phases["l_rear"] == pytest.approx(0.0)
    # Tripod B (offset 0.5) starts at 0.5.
    assert phases["r_front"] == pytest.approx(0.5)
    assert phases["l_middle"] == pytest.approx(0.5)
    assert phases["r_rear"] == pytest.approx(0.5)

    clock.advance(0.4, cycle_time=1.0)
    phases = clock.phases()
    assert phases["l_front"] == pytest.approx(0.4)
    # Tripod B wraps around.
    assert phases["r_front"] == pytest.approx(0.9)


def test_advance_rejects_non_positive_cycle_time():
    clock = GaitClock(_zero_offsets())
    with pytest.raises(ValueError):
        clock.advance(0.1, cycle_time=0.0)
    with pytest.raises(ValueError):
        clock.advance(0.1, cycle_time=-1.0)


def test_phase_offsets_validates_membership():
    with pytest.raises(ValueError):
        PhaseOffsets(offsets={"l_front": 0.0})


def test_phase_offsets_validates_range():
    bad = {n: 0.0 for n in LEG_NAMES}
    bad["l_front"] = 1.0  # not in [0, 1)
    with pytest.raises(ValueError):
        PhaseOffsets(offsets=bad)
