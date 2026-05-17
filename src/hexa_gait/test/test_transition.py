import pytest

from hexa_gait.clock import LEG_NAMES
from hexa_gait.transition import TransitionController, TransitionState


def _flat_stance() -> dict[str, tuple[float, float, float]]:
    # Simple symmetric six-leg layout sufficient for ladder testing.
    return {
        "l_front": (0.15, 0.10, -0.10),
        "r_front": (0.15, -0.10, -0.10),
        "l_middle": (0.0, 0.12, -0.10),
        "r_middle": (0.0, -0.12, -0.10),
        "l_rear": (-0.15, 0.10, -0.10),
        "r_rear": (-0.15, -0.10, -0.10),
    }


def _recenter_order() -> tuple[str, ...]:
    return ("l_front", "r_front", "l_middle", "r_middle", "l_rear", "r_rear")


def _controller(**overrides):
    args = dict(
        nominal_stance=_flat_stance(),
        force_touchdown_speed=0.05,
        recenter_swing_time=0.4,
        recenter_order=_recenter_order(),
        swing_clearance=0.03,
        swing_width=0.0,
        controller_dt=0.02,
    )
    args.update(overrides)
    return TransitionController(**args)


def test_begin_skips_force_touchdown_when_all_grounded():
    ctrl = _controller()
    nominal = _flat_stance()
    ctrl.begin(last_targets=nominal, swing_flags={n: False for n in LEG_NAMES})
    assert ctrl.state is TransitionState.RECENTER


def test_force_touchdown_drives_airborne_legs_to_ground():
    ctrl = _controller(force_touchdown_speed=1.0)  # 1 m/s for a fast test
    nominal = _flat_stance()
    targets = dict(nominal)
    # Lift one leg 0.05 m above the ground.
    targets["l_front"] = (0.15, 0.10, -0.05)
    flags = {n: False for n in LEG_NAMES}
    flags["l_front"] = True
    ctrl.begin(last_targets=targets, swing_flags=flags)
    assert ctrl.state is TransitionState.FORCE_TOUCHDOWN

    # 0.06 s at 1 m/s drops the leg by 0.06 m — past the ground; the
    # controller should clamp to ground_z and transition to RECENTER.
    out = ctrl.update(dt=0.06)
    assert ctrl.state is TransitionState.RECENTER
    assert out["l_front"].foot_target[2] == pytest.approx(-0.10, abs=1e-9)
    assert out["l_front"].stance is True


def test_recenter_visits_each_leg_in_order_with_one_airborne():
    ctrl = _controller(recenter_swing_time=0.1)
    nominal = _flat_stance()
    # Start every leg slightly offset from nominal so RECENTER has work
    # to do; all legs grounded so FORCE_TOUCHDOWN is skipped.
    offset = {
        n: (nominal[n][0] + 0.01, nominal[n][1] + 0.01, nominal[n][2])
        for n in LEG_NAMES
    }
    ctrl.begin(last_targets=offset, swing_flags={n: False for n in LEG_NAMES})

    visited: list[str] = []
    # Run for up to one cycle per leg (plus a margin); each leg should
    # appear airborne for ~recenter_swing_time, then return to stance.
    for _ in range(80):
        out = ctrl.update(dt=0.02)
        airborne = [n for n in LEG_NAMES if not out[n].stance]
        assert len(airborne) <= 1, f"two legs airborne at once: {airborne}"
        if airborne and (not visited or visited[-1] != airborne[0]):
            visited.append(airborne[0])
        if ctrl.state is TransitionState.STAND:
            break

    assert visited == list(_recenter_order())
    assert ctrl.state is TransitionState.STAND


def test_stand_emits_nominal_for_all_legs():
    ctrl = _controller()
    nominal = _flat_stance()
    ctrl.begin(last_targets=nominal, swing_flags={n: False for n in LEG_NAMES})
    # 6 legs * 0.4s recenter_swing_time = 2.4s; 200 iters * 0.02s = 4s
    # is comfortably enough to reach STAND.
    for _ in range(200):
        ctrl.update(dt=0.02)
        if ctrl.state is TransitionState.STAND:
            break
    assert ctrl.state is TransitionState.STAND
    out = ctrl.update(dt=0.02)
    for name in LEG_NAMES:
        assert out[name].foot_target == nominal[name]
        assert out[name].stance is True
        assert out[name].phase == 0.0


def test_recenter_order_validation():
    with pytest.raises(ValueError):
        _controller(recenter_order=("l_front",) * 6)
