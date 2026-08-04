import math
from pathlib import Path

import pytest
import yaml

from hexa_common.limits import (
    VelocityCaps,
    load_velocity_caps,
    outer_stance_radius,
    scale_to_envelope,
    standing_stance_xy,
)


def _write_geometry(tmp_path: Path, **standing_overrides) -> Path:
    # Only the two reference mounts matter here; symmetry fills in the rest.
    # Values match geometry.yaml so the derived stance radii are realistic.
    geo = {
        "mounts": {
            "l_front": {"x": 0.083, "y": 0.0575, "yaw_deg": 30},
            "l_middle": {"x": 0.0, "y": 0.082, "yaw_deg": 90},
        }
    }
    path = tmp_path / "geometry.yaml"
    path.write_text(yaml.safe_dump(geo))
    return path


def _standing_pose(tip_reach=0.135, body_height=0.04, coxa_deg=0) -> dict:
    """A default_standing_pose block with all three groups alike."""
    return dict(
        body_height=body_height,
        **{
            group: dict(tip_reach=tip_reach, coxa_deg=coxa_deg)
            for group in ("front", "middle", "rear")
        },
    )


def _write_yaml(tmp_path: Path, **overrides) -> Path:
    # Duty factors are sourced from the gait descriptors in
    # ``hexa_common.gait_catalog``, not YAML. The YAML only carries the
    # gait-agnostic knobs. default_standing_pose is here because the angular cap
    # is derived from the stance it describes, not from a knob.
    base = dict(
        stride_length=0.12,
        min_swing_time=0.30,
        max_swing_time=1.0,
        step_height=0.035,
        swing_width=0.0,
        controller_dt=0.02,
        cmd_zero_tol=1.0e-4,
        yaw_bias=0.75,
        default_standing_pose=_standing_pose(),
        # No margin by default, so these cases pin the plain duty-factor
        # arithmetic; test_linear_max_drops_with_swing_phase_margin covers the
        # margined form.
        swing_phase_margin=0.0,
    )
    base.update(overrides)
    path = tmp_path / "gait.yaml"
    # gait.yaml is a ros2 params file; load_velocity_caps unwraps this.
    path.write_text(yaml.safe_dump({"gait_node": {"ros__parameters": base}}))
    return path


def _caps(tmp_path: Path, **overrides) -> VelocityCaps:
    return load_velocity_caps(
        _write_yaml(tmp_path, **overrides), _write_geometry(tmp_path)
    )


# Standing foot positions the fixtures above solve to, and the resulting outer
# radius. Recomputed rather than hard-coded where the test is about the
# derivation itself; hard-coded here as the expected value.
_R_OUTER = math.hypot(0.083 + 0.135 * math.cos(math.radians(30)),
                      0.0575 + 0.135 * math.sin(math.radians(30)))


def test_linear_max_tripod_derived_from_stride_swing_time_and_duty(tmp_path):
    # tripod linear_max = 0.12 * (1 − 0.5) / (0.30 * 0.5) = 0.40 m/s.
    caps = _caps(tmp_path)
    assert isinstance(caps, VelocityCaps)
    assert math.isclose(caps.linear_max("tripod"), 0.40)


def test_linear_max_per_gait_strictly_decreasing_with_duty(tmp_path):
    # Slower gait (higher β) gives a lower linear cap because the
    # swing window shrinks while the stance window grows.
    caps = _caps(tmp_path)
    # crawl = 0.12 * (1/3) / (0.30 * 2/3) = 0.20 m/s
    # ripple   = 0.12 * (1/6) / (0.30 * 5/6) = 0.08 m/s
    assert math.isclose(caps.linear_max("crawl"), 0.20, rel_tol=1e-9)
    assert math.isclose(caps.linear_max("ripple"), 0.08, rel_tol=1e-9)
    assert (
        caps.linear_max("tripod")
        > caps.linear_max("crawl")
        > caps.linear_max("ripple")
    )


def test_linear_max_drops_with_swing_phase_margin(tmp_path):
    # The margin hands part of the swing window back to stance, so the same
    # stride takes longer to cover and the cap falls. tripod swing_end becomes
    # 0.5 * 0.88 = 0.44, so linear_max = 0.12 * 0.44 / (0.30 * 0.56) = 0.3143.
    caps = _caps(tmp_path, swing_phase_margin=0.12)
    assert math.isclose(caps.linear_max("tripod"), 0.12 * 0.44 / (0.30 * 0.56))
    # A zero margin must reproduce the unmargined caps exactly, so the formula
    # stays backward compatible with a params file that predates the knob.
    plain = _caps(tmp_path, swing_phase_margin=0.0)
    for gait in ("tripod", "crawl", "ripple"):
        assert caps.linear_max(gait) < plain.linear_max(gait)
    assert math.isclose(plain.linear_max("tripod"), 0.40)


def test_linear_max_unknown_gait_raises(tmp_path):
    # Per-gait caps fail fast on typos rather than silently falling
    # back — the control layer must agree with the catalog names.
    caps = _caps(tmp_path)
    with pytest.raises(KeyError):
        caps.linear_max("gallop")


def test_linear_max_scales_with_stride_length(tmp_path):
    # Double stride_length → double linear_max for every gait.
    caps = _caps(tmp_path, stride_length=0.24)
    assert math.isclose(caps.linear_max("tripod"), 0.80)
    assert math.isclose(caps.linear_max("ripple"), 0.16)


def test_linear_max_scales_inversely_with_min_swing_time(tmp_path):
    # Slower min_swing_time → lower linear_max.
    caps = _caps(tmp_path, min_swing_time=0.60)
    assert math.isclose(caps.linear_max("tripod"), 0.20)


# ── stance geometry the angular cap is derived from ──────────────────────────


def test_standing_stance_places_each_foot_tip_reach_out_from_its_mount(tmp_path):
    # Closed-form mirror of standing_pose_from + leg_to_body: the tip sits its
    # group's tip_reach out along the leg's own direction, offset by the mount.
    stance = standing_stance_xy(_write_geometry(tmp_path), _write_yaml(tmp_path))
    assert set(stance) == {
        "l_front", "l_middle", "l_rear", "r_front", "r_middle", "r_rear",
    }
    # Middle legs point straight out (+y on the left), so the radius is simply
    # the mount offset plus the reach.
    assert math.isclose(stance["l_middle"][0], 0.0, abs_tol=1e-12)
    assert math.isclose(stance["l_middle"][1], 0.082 + 0.135)
    # Front-left: mount (0.083, 0.0575) plus 0.135 at 30 degrees.
    assert math.isclose(stance["l_front"][0], 0.083 + 0.135 * math.cos(math.radians(30)))
    assert math.isclose(stance["l_front"][1], 0.0575 + 0.135 * math.sin(math.radians(30)))
    # Left/right and fore/aft symmetry.
    assert math.isclose(stance["r_front"][0], stance["l_front"][0])
    assert math.isclose(stance["r_front"][1], -stance["l_front"][1])
    assert math.isclose(stance["l_rear"][0], -stance["l_front"][0])
    assert math.isclose(stance["l_rear"][1], stance["l_front"][1])


def test_each_group_reaches_out_its_own_distance(tmp_path):
    # The three pairs are configured separately; left/right still mirror.
    pose = _standing_pose()
    pose["front"]["tip_reach"] = 0.120
    pose["middle"]["tip_reach"] = 0.140
    pose["rear"]["tip_reach"] = 0.160
    stance = standing_stance_xy(
        _write_geometry(tmp_path),
        _write_yaml(tmp_path, default_standing_pose=pose),
    )
    assert math.isclose(stance["l_middle"][1], 0.082 + 0.140)
    assert math.isclose(
        stance["l_front"][0], 0.083 + 0.120 * math.cos(math.radians(30))
    )
    assert math.isclose(
        stance["l_rear"][0], -(0.083 + 0.160 * math.cos(math.radians(30)))
    )
    for group in ("front", "middle", "rear"):
        assert math.isclose(stance[f"r_{group}"][0], stance[f"l_{group}"][0])
        assert math.isclose(stance[f"r_{group}"][1], -stance[f"l_{group}"][1])


def test_positive_coxa_deg_splays_outward_on_every_leg(tmp_path):
    # The sign rule: a positive value is the left leg's, negated for rear legs
    # and again for right ones, so the same number widens front and rear alike.
    pose = _standing_pose(coxa_deg=15)
    splayed = standing_stance_xy(
        _write_geometry(tmp_path),
        _write_yaml(tmp_path, default_standing_pose=pose),
    )
    straight = standing_stance_xy(
        _write_geometry(tmp_path), _write_yaml(tmp_path)
    )
    # Corner feet move further from the fore/aft centreline, not across it.
    for leg in ("l_front", "l_rear"):
        assert splayed[leg][1] > straight[leg][1] > 0.0
    for leg in ("r_front", "r_rear"):
        assert splayed[leg][1] < straight[leg][1] < 0.0
    # ...and the footprint stays mirror-symmetric front-to-back.
    assert math.isclose(splayed["l_rear"][0], -splayed["l_front"][0])
    assert math.isclose(splayed["l_rear"][1], splayed["l_front"][1])


def test_outer_stance_radius_is_the_corner_legs(tmp_path):
    # With this geometry the corner feet (0.2358) reach further than the middle
    # ones (0.217), so they set the lever arm.
    r = outer_stance_radius(_write_geometry(tmp_path), _write_yaml(tmp_path))
    assert math.isclose(r, _R_OUTER)
    assert r > 0.082 + 0.135  # beats the middle legs


def test_outer_stance_radius_grows_with_tip_reach(tmp_path):
    wide = outer_stance_radius(
        _write_geometry(tmp_path),
        _write_yaml(tmp_path, default_standing_pose=_standing_pose(tip_reach=0.20)),
    )
    assert wide > _R_OUTER


def test_angular_max_is_linear_max_over_the_outer_stance_radius(tmp_path):
    # The whole point of dropping angular_z_max: the cap is derived, not tuned.
    caps = _caps(tmp_path)
    for gait in ("tripod", "crawl", "ripple"):
        assert math.isclose(
            caps.angular_max(gait), caps.linear_max(gait) / _R_OUTER, rel_tol=1e-12
        )
    # tripod: 0.40 / 0.2358 = 1.6965 rad/s.
    assert math.isclose(caps.angular_max("tripod"), 0.40 / _R_OUTER)


def test_angular_max_falls_when_the_stance_widens(tmp_path):
    # A wider stance is a longer lever arm, so the same foot speed buys less
    # yaw. This is the only way to slow turning now that the knob is gone.
    narrow = _caps(tmp_path)
    wide = _caps(
        tmp_path,
        default_standing_pose=_standing_pose(tip_reach=0.20),
    )
    assert wide.angular_max("tripod") < narrow.angular_max("tripod")
    # Widening the stance must not touch the linear cap.
    assert math.isclose(wide.linear_max("tripod"), narrow.linear_max("tripod"))


def test_angular_max_tracks_the_gait_ordering(tmp_path):
    caps = _caps(tmp_path)
    assert (
        caps.angular_max("tripod")
        > caps.angular_max("crawl")
        > caps.angular_max("ripple")
    )


def test_angular_max_unknown_gait_raises(tmp_path):
    caps = _caps(tmp_path)
    with pytest.raises(KeyError):
        caps.angular_max("gallop")


def test_missing_default_standing_pose_raises(tmp_path):
    # The angular cap has no fallback: without a stance there is no lever arm.
    raw = {
        "stride_length": 0.12,
        "min_swing_time": 0.30,
        "yaw_bias": 0.75,
    }
    path = tmp_path / "gait.yaml"
    path.write_text(yaml.safe_dump({"gait_node": {"ros__parameters": raw}}))
    with pytest.raises(KeyError):
        load_velocity_caps(path, _write_geometry(tmp_path))


def test_missing_mounts_raises(tmp_path):
    geo = tmp_path / "geometry.yaml"
    geo.write_text(yaml.safe_dump({"leg": {"coxa_length": 0.042}}))
    with pytest.raises(KeyError):
        load_velocity_caps(_write_yaml(tmp_path), geo)


def test_yaw_bias_anchors_at_tripod_and_eases_with_duty(tmp_path):
    # yaw_bias is per-gait, easing back toward neutral as β grows:
    #   yaw_bias_eff(β) = 0.5 + (yaw_bias_yaml − 0.5) · (1.5 − β)
    # The YAML value anchors at tripod (β=0.5); slower gaits sit closer
    # to neutral because their smaller linear_max can't absorb an
    # aggressive cut on top of the gait's intrinsic slowness.
    caps = _caps(tmp_path, yaw_bias=0.6)
    assert math.isclose(caps.yaw_bias("tripod"), 0.60, rel_tol=1e-9)
    # crawl β=2/3 → 0.5 + 0.1 · (1.5 − 2/3) = 0.5833
    assert math.isclose(caps.yaw_bias("crawl"), 0.5 + 0.1 * (1.5 - 2.0 / 3.0), rel_tol=1e-9)
    # ripple   β=5/6 → 0.5 + 0.1 · (1.5 − 5/6) = 0.5667
    assert math.isclose(caps.yaw_bias("ripple"), 0.5 + 0.1 * (1.5 - 5.0 / 6.0), rel_tol=1e-9)
    # Strict monotone: deviation shrinks as duty grows.
    dev = lambda name: caps.yaw_bias(name) - 0.5
    assert dev("tripod") > dev("crawl") > dev("ripple") > 0.0


def test_yaw_bias_uniform_when_yaml_is_neutral(tmp_path):
    # yaw_bias_yaml = 0.5 ⇒ no deviation ⇒ every gait stays at 0.5.
    caps = _caps(tmp_path, yaw_bias=0.5)
    for name in ("tripod", "crawl", "ripple"):
        assert math.isclose(caps.yaw_bias(name), 0.5, rel_tol=1e-9)


def test_yaw_bias_unknown_gait_raises(tmp_path):
    caps = _caps(tmp_path)
    with pytest.raises(KeyError):
        caps.yaw_bias("gallop")


def test_missing_yaw_bias_raises(tmp_path):
    raw = {
        "stride_length": 0.12,
        "min_swing_time": 0.30,
    }
    path = tmp_path / "gait.yaml"
    path.write_text(yaml.safe_dump({"gait_node": {"ros__parameters": raw}}))
    with pytest.raises(KeyError):
        load_velocity_caps(path, _write_geometry(tmp_path))


def test_accepts_string_path(tmp_path):
    caps = load_velocity_caps(
        str(_write_yaml(tmp_path)), str(_write_geometry(tmp_path))
    )
    assert math.isclose(caps.linear_max("tripod"), 0.40)


# ── scale_to_envelope ───────────────────────────────────────────────────────

# Standing foot positions matching the fixture geometry's expansion. These are
# the feet, not the mounts: the foot is the lever arm a yaw rate acts through,
# because that is where the gait engine lays the stride down.
_C_X = 0.083 + 0.135 * math.cos(math.radians(30))
_C_Y = 0.0575 + 0.135 * math.sin(math.radians(30))
_M_Y = 0.082 + 0.135
_STANCE: dict[str, tuple[float, float, float]] = {
    "l_front":  (_C_X, _C_Y, 0.0),
    "l_middle": (0.0, _M_Y, 0.0),
    "l_rear":   (-_C_X, _C_Y, 0.0),
    "r_front":  (_C_X, -_C_Y, 0.0),
    "r_middle": (0.0, -_M_Y, 0.0),
    "r_rear":   (-_C_X, -_C_Y, 0.0),
}
_LINEAR_MAX = 0.40
_YAW_BIAS = 0.75
_UNIFORM_BIAS = 0.5


def _max_leg_speed(v_x: float, v_y: float, omega_z: float) -> float:
    return max(
        math.hypot(v_x - omega_z * r_y, v_y + omega_z * r_x)
        for r_x, r_y, _ in _STANCE.values()
    )


def test_scale_passthrough_when_within_envelope():
    # Modest forward + modest yaw — every leg under 0.40 m/s.
    out = scale_to_envelope(0.1, 0.0, 0.5, _STANCE, _LINEAR_MAX, _YAW_BIAS)
    assert math.isclose(out[0], 0.1)
    assert math.isclose(out[1], 0.0)
    assert math.isclose(out[2], 0.5)


def test_scale_zero_command_stays_zero():
    out = scale_to_envelope(0.0, 0.0, 0.0, _STANCE, _LINEAR_MAX, _YAW_BIAS)
    assert out == (0.0, 0.0, 0.0)


def test_scale_pure_linear_at_cap_unchanged():
    # v_x = linear_max, no yaw: max leg speed equals the cap exactly,
    # so the cut must be a no-op.
    out = scale_to_envelope(0.40, 0.0, 0.0, _STANCE, _LINEAR_MAX, _YAW_BIAS)
    assert math.isclose(out[0], 0.40)
    assert math.isclose(out[1], 0.0)
    assert math.isclose(out[2], 0.0)


def test_scale_uniform_bias_preserves_ratio_at_full_forward_plus_full_yaw():
    # yaw_bias = 0.5 ⇒ ρ = 1 ⇒ uniform scaling — the cut falls equally
    # on v_x and omega_z, so the commanded translation:yaw ratio
    # survives. Regression guard for the unbiased baseline.
    v_x, v_y, omega_z = scale_to_envelope(
        0.40, 0.0, 1.0, _STANCE, _LINEAR_MAX, _UNIFORM_BIAS
    )
    assert math.isclose(_max_leg_speed(v_x, v_y, omega_z), _LINEAR_MAX, rel_tol=1e-9)
    # 0.40 / 1.0 = 0.40 ratio preserved.
    assert math.isclose(v_x / omega_z, 0.40, rel_tol=1e-9)
    assert math.isclose(v_y, 0.0)


def test_scale_biased_cut_favours_yaw_at_full_forward_plus_full_yaw():
    # yaw_bias = 0.75 ⇒ ρ = 3: at the cut, translation absorbs three
    # times the cut fraction omega does. The resulting v_x sits well
    # below uniform, omega sits well above, and the binding leg is at
    # the per-leg cap exactly.
    v_x_u, _, omega_u = scale_to_envelope(
        0.40, 0.0, 1.0, _STANCE, _LINEAR_MAX, _UNIFORM_BIAS
    )
    v_x_b, v_y_b, omega_b = scale_to_envelope(
        0.40, 0.0, 1.0, _STANCE, _LINEAR_MAX, _YAW_BIAS
    )

    assert v_x_b < v_x_u
    assert omega_b > omega_u
    assert math.isclose(v_y_b, 0.0)

    # ρ = 0.75 / 0.25 = 3 ⇒ (1 - s_v) / (1 - s_w) = 3.
    s_v = v_x_b / 0.40
    s_w = omega_b / 1.0
    assert math.isclose((1.0 - s_v) / (1.0 - s_w), 3.0, rel_tol=1e-9)

    # Binding leg lands on the per-leg cap, no overshoot.
    assert math.isclose(
        _max_leg_speed(v_x_b, v_y_b, omega_b), _LINEAR_MAX, rel_tol=1e-9
    )


def test_scale_bounds_pure_yaw_without_a_separate_angular_clamp():
    # There is no angular_max input any more. Capping every foot's speed is
    # what bounds omega, and it lands exactly on linear_max / r_outer — the
    # value VelocityCaps.angular_max reports.
    r_outer = max(math.hypot(r_x, r_y) for r_x, r_y, _ in _STANCE.values())
    for commanded in (2.0, 6.0, 50.0):
        _, _, omega = scale_to_envelope(
            0.0, 0.0, commanded, _STANCE, _LINEAR_MAX, _YAW_BIAS
        )
        assert math.isclose(omega, _LINEAR_MAX / r_outer, rel_tol=1e-9)


def test_scale_bounds_pure_yaw_symmetrically(tmp_path):
    r_outer = max(math.hypot(r_x, r_y) for r_x, r_y, _ in _STANCE.values())
    _, _, omega = scale_to_envelope(
        0.0, 0.0, -50.0, _STANCE, _LINEAR_MAX, _YAW_BIAS
    )
    assert math.isclose(omega, -_LINEAR_MAX / r_outer, rel_tol=1e-9)


def test_scale_yaw_ceiling_agrees_with_the_derived_cap(tmp_path):
    # Ties the two halves together: what load_velocity_caps advertises as the
    # angular cap is exactly what the envelope cut lets through.
    caps = _caps(tmp_path)
    for gait in ("tripod", "crawl", "ripple"):
        _, _, omega = scale_to_envelope(
            0.0, 0.0, 100.0, _STANCE, caps.linear_max(gait), caps.yaw_bias(gait)
        )
        assert math.isclose(omega, caps.angular_max(gait), rel_tol=1e-6)


def test_scale_biased_cut_favours_yaw_for_lateral_plus_yaw():
    # v_y exercises the r_x-coupled term — the cut split has to handle
    # the lateral direction the same way as forward.
    v_x_b, v_y_b, omega_b = scale_to_envelope(
        0.0, 0.40, 1.0, _STANCE, _LINEAR_MAX, _YAW_BIAS
    )
    assert math.isclose(
        _max_leg_speed(v_x_b, v_y_b, omega_b), _LINEAR_MAX, rel_tol=1e-9
    )
    assert math.isclose(v_x_b, 0.0)
    s_v = v_y_b / 0.40
    s_w = omega_b / 1.0
    assert math.isclose((1.0 - s_v) / (1.0 - s_w), 3.0, rel_tol=1e-9)


def test_scale_yaw_only_violation_zeros_translation():
    # Slow-gait corner: the commanded yaw alone already breaks the per-leg
    # envelope. The bias-toward-yaw contract pins translation at zero and
    # scales omega to fit.
    tiny_linear = 0.05
    r_outer = max(math.hypot(r_x, r_y) for r_x, r_y, _ in _STANCE.values())
    assert 1.0 * r_outer > tiny_linear  # premise: yaw alone overshoots
    v_x, v_y, omega = scale_to_envelope(
        0.10, 0.0, 1.0, _STANCE, tiny_linear, _YAW_BIAS
    )
    assert math.isclose(v_x, 0.0)
    assert math.isclose(v_y, 0.0)
    assert math.isclose(omega, tiny_linear / r_outer, rel_tol=1e-9)


def test_scale_uses_per_gait_linear_max(tmp_path):
    # The whole point of the refactor: passing a smaller linear_max
    # (e.g. ripple's cap) cuts the command down accordingly.
    caps = _caps(tmp_path)
    v_x, _, omega = scale_to_envelope(
        0.40,
        0.0,
        0.0,
        _STANCE,
        caps.linear_max("ripple"),
        caps.yaw_bias("ripple"),
    )
    # 0.40 was tripod's cap; ripple cap is 0.08, so the input gets scaled
    # to 0.08 (no yaw → max leg speed = |v_x|, bias is irrelevant).
    assert math.isclose(v_x, 0.08, rel_tol=1e-9)
    assert math.isclose(omega, 0.0)


def test_scale_accepts_two_component_stance_entries(tmp_path):
    # standing_stance_xy returns (x, y); scale_to_envelope must take it as-is.
    stance = standing_stance_xy(_write_geometry(tmp_path), _write_yaml(tmp_path))
    out = scale_to_envelope(0.1, 0.0, 0.5, stance, _LINEAR_MAX, _YAW_BIAS)
    assert math.isclose(out[0], 0.1)
    assert math.isclose(out[2], 0.5)
