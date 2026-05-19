"""Unit tests for the reseat ladder.

Two layers:

1. ``reseat_nominal_stance`` — pure geometric function. At Δz=0 it
   reproduces the YAML standing nominal stance; positive Δz pulls the
   feet inward radially (femur drops, so the leg's horizontal reach
   shrinks); negative Δz pushes them outward. The body-frame z stays
   pinned to the default value regardless of Δz — the lift lives in
   pose.z, the gait nominal only tracks XY.
2. ``ReseatController`` — pair-sequenced ladder. Confirms the pair
   order, the per-pair completion timing, and the per-pair targets.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from hexa_gait.clock import LEG_NAMES
from hexa_gait.engine import reseat_geometry_from_yaml
from hexa_gait.initialize import PAIR_ORDER
from hexa_gait.reseat import (
    ReseatController,
    ReseatGeometry,
    default_geometry_from_pose,
    reseat_nominal_stance,
)
from hexa_kinematics.body_transform import leg_to_body
from hexa_kinematics.joint_config import load_standing_pose
from hexa_kinematics.leg_geometry import LegSpec
from hexa_kinematics.leg_ik import forward_kinematics
from hexa_kinematics.leg_specs import load_leg_specs


# --- reseat_nominal_stance -------------------------------------------------


def _leg_specs() -> dict[str, LegSpec]:
    geom = (
        Path(__file__).resolve().parents[2]
        / "hexa_description"
        / "config"
        / "geometry.yaml"
    )
    return load_leg_specs(geom)


def _yaml_geometry() -> ReseatGeometry:
    desc = Path(__file__).resolve().parents[2] / "hexa_description" / "config"
    return reseat_geometry_from_yaml(desc / "geometry.yaml", desc / "standing_pose.yaml")


def _yaml_nominal() -> dict[str, tuple[float, float, float]]:
    desc = Path(__file__).resolve().parents[2] / "hexa_description" / "config"
    legs = _leg_specs()
    angles = load_standing_pose(desc / "standing_pose.yaml", desc / "geometry.yaml")
    return {n: leg_to_body(forward_kinematics(angles, legs[n]), legs[n]) for n in LEG_NAMES}


def test_at_zero_height_returns_yaml_nominal_stance():
    # Sanity check: at Δz = 0 the reseat function reproduces the
    # YAML-derived nominal stance to numerical precision.
    out = reseat_nominal_stance(0.0, _yaml_geometry(), _leg_specs())
    nominal = _yaml_nominal()
    for name in LEG_NAMES:
        assert out[name] == pytest.approx(nominal[name], abs=1e-9)


def test_positive_height_grows_radial_distance():
    # Body lifted ⇒ feet need to be deeper relative to body ⇒ with
    # the tibia held at its default near-vertical lean, the femur
    # drops toward horizontal, which extends the foot's radial reach
    # outward in the leg frame. The body-frame foot therefore sits
    # *further* from the hip's X/Y projection — the legs splay out.
    legs = _leg_specs()
    geom = _yaml_geometry()
    nominal_default = reseat_nominal_stance(0.0, geom, legs)
    nominal_lifted = reseat_nominal_stance(0.03, geom, legs)
    for name in LEG_NAMES:
        mx, my, _ = legs[name].mount_xyz
        r_default = math.hypot(
            nominal_default[name][0] - mx, nominal_default[name][1] - my
        )
        r_lifted = math.hypot(
            nominal_lifted[name][0] - mx, nominal_lifted[name][1] - my
        )
        assert r_lifted > r_default


def test_negative_height_shrinks_radial_distance():
    # Inverse of the positive case: body lowered ⇒ feet closer to
    # body in z ⇒ femur rises (more above horizontal) ⇒ horizontal
    # reach shrinks ⇒ legs tuck in.
    legs = _leg_specs()
    geom = _yaml_geometry()
    nominal_default = reseat_nominal_stance(0.0, geom, legs)
    nominal_dropped = reseat_nominal_stance(-0.03, geom, legs)
    for name in LEG_NAMES:
        mx, my, _ = legs[name].mount_xyz
        r_default = math.hypot(
            nominal_default[name][0] - mx, nominal_default[name][1] - my
        )
        r_dropped = math.hypot(
            nominal_dropped[name][0] - mx, nominal_dropped[name][1] - my
        )
        assert r_dropped < r_default


def test_body_frame_z_stays_at_default_at_any_height():
    # Critical invariant — the body lift lives entirely in pose.z, NOT
    # in the gait's nominal_stance. The kinematics chain's
    # apply_body_pose subtracts pose.z; the gait nominal must match the
    # at-zero-height value so the net leg-frame foot depth ends up at
    # default_foot_depth + Δz.
    legs = _leg_specs()
    geom = _yaml_geometry()
    base_z = reseat_nominal_stance(0.0, geom, legs)[LEG_NAMES[0]][2]
    for dz in (-0.03, -0.01, 0.0, 0.01, 0.03):
        nominal = reseat_nominal_stance(dz, geom, legs)
        for name in LEG_NAMES:
            assert nominal[name][2] == pytest.approx(base_z, abs=1e-12)


def test_infeasible_height_raises():
    # A target so far from default that arcsin would saturate.
    # default_foot_depth ≈ 0.085 m; the maximum is roughly
    # tibia_len*cos(θ_t) + femur_len ≈ 0.13 + 0.08 ≈ 0.21 m. So Δz of
    # 0.20 m exceeds the limit.
    legs = _leg_specs()
    geom = _yaml_geometry()
    with pytest.raises(ValueError, match="geometrically feasible"):
        reseat_nominal_stance(0.20, geom, legs)


def test_default_geometry_from_pose_matches_yaml_helper():
    # The two ways to build the geometry — direct via the leaf
    # function, or through the engine-level YAML helper — must agree.
    desc = Path(__file__).resolve().parents[2] / "hexa_description" / "config"
    legs = _leg_specs()
    angles = load_standing_pose(desc / "standing_pose.yaml", desc / "geometry.yaml")
    direct = default_geometry_from_pose(angles, legs[LEG_NAMES[0]])
    yaml_helper = reseat_geometry_from_yaml(
        desc / "geometry.yaml", desc / "standing_pose.yaml"
    )
    assert direct.coxa_len == yaml_helper.coxa_len
    assert direct.femur_len == yaml_helper.femur_len
    assert direct.tibia_len == yaml_helper.tibia_len
    assert direct.tibia_from_vertical == yaml_helper.tibia_from_vertical
    assert direct.default_foot_depth == yaml_helper.default_foot_depth


# --- ReseatController ------------------------------------------------------


def _stance(z: float = -0.10) -> dict[str, tuple[float, float, float]]:
    # Symmetric six-leg layout (matches test_engine.py geometry); the
    # exact numbers don't matter — only that the controller ticks
    # through pairs correctly.
    return {
        "l_front": (0.15, 0.10, z),
        "r_front": (0.15, -0.10, z),
        "l_middle": (0.0, 0.12, z),
        "r_middle": (0.0, -0.12, z),
        "l_rear": (-0.15, 0.10, z),
        "r_rear": (-0.15, -0.10, z),
    }


def _shifted_stance(dx: float) -> dict[str, tuple[float, float, float]]:
    """Stance with each foot translated radially outward by dx in body frame."""
    out: dict[str, tuple[float, float, float]] = {}
    for name, xyz in _stance().items():
        # Push each foot away from the body centre along its (x, y).
        nx, ny, nz = xyz
        r = math.hypot(nx, ny)
        if r > 0.0:
            scale = (r + dx) / r
            out[name] = (nx * scale, ny * scale, nz)
        else:
            out[name] = xyz
    return out


def _controller(**overrides) -> ReseatController:
    args = dict(
        current_stance=_stance(),
        target_stance=_shifted_stance(0.02),
        pair_swing_time=0.1,
        swing_clearance=0.02,
        swing_width=0.0,
        controller_dt=0.02,
    )
    args.update(overrides)
    return ReseatController(**args)


def test_controller_starts_not_done():
    ctrl = _controller()
    assert ctrl.done is False


def test_first_tick_only_first_pair_swings():
    ctrl = _controller()
    current = _stance()
    out = ctrl.update(dt=0.02)
    active = PAIR_ORDER[0]  # ("l_middle", "r_middle")
    for name in active:
        assert out[name].foot_target != current[name]
        assert out[name].stance is False
    for name in LEG_NAMES:
        if name in active:
            continue
        assert out[name].foot_target == current[name]
        assert out[name].stance is True


def test_pairs_complete_in_order_and_snap_to_targets():
    target = _shifted_stance(0.02)
    ctrl = _controller(target_stance=target)
    dt = 0.02

    def _drain(active: tuple[str, str]) -> dict[str, tuple[float, float, float]]:
        for _ in range(20):
            out = ctrl.update(dt=dt)
            done = all(out[n].foot_target == target[n] for n in active)
            if done:
                return {n: out[n].foot_target for n in LEG_NAMES}
        raise AssertionError(f"pair {active} did not complete")

    # Pair 1: middle pair lands on target; others still at current stance.
    snap = _drain(PAIR_ORDER[0])
    current = _stance()
    for name in PAIR_ORDER[0]:
        assert snap[name] == pytest.approx(target[name], abs=1e-9)
    for name in PAIR_ORDER[1] + PAIR_ORDER[2]:
        assert snap[name] == current[name]

    # Pair 2: first diagonal lands; other diagonal still at current.
    snap = _drain(PAIR_ORDER[1])
    for name in PAIR_ORDER[0] + PAIR_ORDER[1]:
        assert snap[name] == pytest.approx(target[name], abs=1e-9)
    for name in PAIR_ORDER[2]:
        assert snap[name] == current[name]

    # Pair 3: other diagonal lands; ladder done.
    snap = _drain(PAIR_ORDER[2])
    for name in LEG_NAMES:
        assert snap[name] == pytest.approx(target[name], abs=1e-9)
    assert ctrl.done is True


def test_done_state_emits_target_forever():
    ctrl = _controller()
    target = _shifted_stance(0.02)
    for _ in range(500):
        ctrl.update(dt=0.02)
        if ctrl.done:
            break
    assert ctrl.done is True
    out = ctrl.update(dt=0.02)
    for name in LEG_NAMES:
        assert out[name].foot_target == target[name]
        assert out[name].stance is True


def test_missing_legs_raises():
    incomplete = dict(_stance())
    incomplete.pop("l_rear")
    with pytest.raises(ValueError, match="current_stance missing legs"):
        _controller(current_stance=incomplete)
    incomplete = dict(_shifted_stance(0.02))
    incomplete.pop("r_front")
    with pytest.raises(ValueError, match="target_stance missing legs"):
        _controller(target_stance=incomplete)


def test_nonpositive_pair_swing_time_raises():
    with pytest.raises(ValueError, match="pair_swing_time"):
        _controller(pair_swing_time=0.0)
