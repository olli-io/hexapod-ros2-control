import math
from pathlib import Path

import pytest
import yaml

from hexa_gait.limits import VelocityCaps, load_velocity_caps, scale_to_envelope


def _write_yaml(tmp_path: Path, **overrides) -> Path:
    # Duty factors are sourced from the strategy classes in
    # ``hexa_gait.gaits``, not YAML. The YAML only carries the
    # gait-agnostic knobs.
    base = dict(
        stride_length=0.12,
        min_swing_time=0.30,
        max_cycle_time=2.0,
        step_height=0.035,
        swing_width=0.0,
        controller_dt=0.02,
        cmd_zero_tol=1.0e-4,
        max_foot_speed=0.333,
        max_swing_time=0.6,
        angular_z_max=1.0,
        yaw_bias=0.75,
    )
    base.update(overrides)
    path = tmp_path / "gait.yaml"
    path.write_text(yaml.safe_dump(base))
    return path


def test_linear_max_tripod_derived_from_stride_swing_time_and_duty(tmp_path):
    # tripod linear_max = 0.12 * (1 − 0.5) / (0.30 * 0.5) = 0.40 m/s.
    path = _write_yaml(tmp_path)
    caps = load_velocity_caps(path)
    assert isinstance(caps, VelocityCaps)
    assert math.isclose(caps.linear_max("tripod"), 0.40)


def test_linear_max_per_gait_strictly_decreasing_with_duty(tmp_path):
    # Slower gait (higher β) gives a lower linear cap because the
    # swing window shrinks while the stance window grows.
    path = _write_yaml(tmp_path)
    caps = load_velocity_caps(path)
    # ripple = 0.12 * (1/3) / (0.30 * 2/3) = 0.20 m/s
    # wave   = 0.12 * (1/6) / (0.30 * 5/6) = 0.08 m/s
    assert math.isclose(caps.linear_max("ripple"), 0.20, rel_tol=1e-9)
    assert math.isclose(caps.linear_max("wave"), 0.08, rel_tol=1e-9)
    assert (
        caps.linear_max("tripod")
        > caps.linear_max("ripple")
        > caps.linear_max("wave")
    )


def test_linear_max_unknown_gait_raises(tmp_path):
    # Per-gait caps fail fast on typos rather than silently falling
    # back — the control layer must agree with the registry names.
    path = _write_yaml(tmp_path)
    caps = load_velocity_caps(path)
    with pytest.raises(KeyError):
        caps.linear_max("gallop")


def test_linear_max_scales_with_stride_length(tmp_path):
    # Double stride_length → double linear_max for every gait.
    path = _write_yaml(tmp_path, stride_length=0.24)
    caps = load_velocity_caps(path)
    assert math.isclose(caps.linear_max("tripod"), 0.80)
    assert math.isclose(caps.linear_max("wave"), 0.16)


def test_linear_max_scales_inversely_with_min_swing_time(tmp_path):
    # Slower min_swing_time → lower linear_max.
    path = _write_yaml(tmp_path, min_swing_time=0.60)
    caps = load_velocity_caps(path)
    assert math.isclose(caps.linear_max("tripod"), 0.20)


def test_angular_max_passes_through_raw(tmp_path):
    # angular_z_max is an explicit knob, not derived from geometry.
    path = _write_yaml(tmp_path, angular_z_max=1.5)
    caps = load_velocity_caps(path)
    assert math.isclose(caps.angular_max, 1.5)


def test_yaw_bias_anchors_at_tripod_and_eases_with_duty(tmp_path):
    # yaw_bias is per-gait, easing back toward neutral as β grows:
    #   yaw_bias_eff(β) = 0.5 + (yaw_bias_yaml − 0.5) · (1.5 − β)
    # The YAML value anchors at tripod (β=0.5); slower gaits sit closer
    # to neutral because their smaller linear_max can't absorb an
    # aggressive cut on top of the gait's intrinsic slowness.
    path = _write_yaml(tmp_path, yaw_bias=0.6)
    caps = load_velocity_caps(path)
    assert math.isclose(caps.yaw_bias("tripod"), 0.60, rel_tol=1e-9)
    # ripple β=2/3 → 0.5 + 0.1 · (1.5 − 2/3) = 0.5833
    assert math.isclose(caps.yaw_bias("ripple"), 0.5 + 0.1 * (1.5 - 2.0 / 3.0), rel_tol=1e-9)
    # wave   β=5/6 → 0.5 + 0.1 · (1.5 − 5/6) = 0.5667
    assert math.isclose(caps.yaw_bias("wave"), 0.5 + 0.1 * (1.5 - 5.0 / 6.0), rel_tol=1e-9)
    # Strict monotone: deviation shrinks as duty grows.
    dev = lambda name: caps.yaw_bias(name) - 0.5
    assert dev("tripod") > dev("ripple") > dev("wave") > 0.0


def test_yaw_bias_uniform_when_yaml_is_neutral(tmp_path):
    # yaw_bias_yaml = 0.5 ⇒ no deviation ⇒ every gait stays at 0.5.
    path = _write_yaml(tmp_path, yaw_bias=0.5)
    caps = load_velocity_caps(path)
    for name in ("tripod", "ripple", "wave"):
        assert math.isclose(caps.yaw_bias(name), 0.5, rel_tol=1e-9)


def test_yaw_bias_unknown_gait_raises(tmp_path):
    path = _write_yaml(tmp_path)
    caps = load_velocity_caps(path)
    with pytest.raises(KeyError):
        caps.yaw_bias("gallop")


def test_missing_angular_z_max_raises(tmp_path):
    raw = {
        "stride_length": 0.12,
        "min_swing_time": 0.30,
        "yaw_bias": 0.75,
    }
    path = tmp_path / "gait.yaml"
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(KeyError):
        load_velocity_caps(path)


def test_missing_yaw_bias_raises(tmp_path):
    raw = {
        "stride_length": 0.12,
        "min_swing_time": 0.30,
        "angular_z_max": 1.0,
    }
    path = tmp_path / "gait.yaml"
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(KeyError):
        load_velocity_caps(path)


def test_accepts_string_path(tmp_path):
    path = _write_yaml(tmp_path)
    caps = load_velocity_caps(str(path))
    assert math.isclose(caps.linear_max("tripod"), 0.40)


# Mount positions matching geometry.yaml's expansion. Only the (r_x, r_y)
# components are read; r_z is ignored. Symmetric six-leg hexapod.
_MOUNTS: dict[str, tuple[float, float, float]] = {
    "l_front":  (0.083, 0.0575, 0.0),
    "l_middle": (0.0,   0.082,  0.0),
    "l_rear":   (-0.083, 0.0575, 0.0),
    "r_front":  (0.083, -0.0575, 0.0),
    "r_middle": (0.0,   -0.082,  0.0),
    "r_rear":   (-0.083, -0.0575, 0.0),
}
_LINEAR_MAX = 0.40
_ANGULAR_MAX = 1.0
_YAW_BIAS = 0.75
_UNIFORM_BIAS = 0.5


def test_scale_passthrough_when_within_envelope():
    # Modest forward + modest yaw — every leg under 0.40 m/s.
    out = scale_to_envelope(
        0.1, 0.0, 0.5, _MOUNTS, _LINEAR_MAX, _ANGULAR_MAX, _YAW_BIAS
    )
    assert math.isclose(out[0], 0.1)
    assert math.isclose(out[1], 0.0)
    assert math.isclose(out[2], 0.5)


def test_scale_zero_command_stays_zero():
    out = scale_to_envelope(
        0.0, 0.0, 0.0, _MOUNTS, _LINEAR_MAX, _ANGULAR_MAX, _YAW_BIAS
    )
    assert out == (0.0, 0.0, 0.0)


def test_scale_pure_linear_at_cap_unchanged():
    # v_x = linear_max, no yaw: max leg speed equals the cap exactly,
    # so the cut must be a no-op.
    out = scale_to_envelope(
        0.40, 0.0, 0.0, _MOUNTS, _LINEAR_MAX, _ANGULAR_MAX, _YAW_BIAS
    )
    assert math.isclose(out[0], 0.40)
    assert math.isclose(out[1], 0.0)
    assert math.isclose(out[2], 0.0)


def test_scale_uniform_bias_preserves_ratio_at_full_forward_plus_full_yaw():
    # yaw_bias = 0.5 ⇒ ρ = 1 ⇒ uniform scaling — the cut falls equally
    # on v_x and omega_z, so the commanded translation:yaw ratio
    # survives. Regression guard for the unbiased baseline.
    v_x, v_y, omega_z = scale_to_envelope(
        0.40, 0.0, 1.0, _MOUNTS, _LINEAR_MAX, _ANGULAR_MAX, _UNIFORM_BIAS
    )
    max_v = 0.0
    for r_x, r_y, _ in _MOUNTS.values():
        max_v = max(max_v, math.hypot(v_x - omega_z * r_y, v_y + omega_z * r_x))
    assert math.isclose(max_v, _LINEAR_MAX, rel_tol=1e-9)
    # 0.40 / 1.0 = 0.40 ratio preserved.
    assert math.isclose(v_x / omega_z, 0.40, rel_tol=1e-9)
    assert math.isclose(v_y, 0.0)


def test_scale_biased_cut_favours_yaw_at_full_forward_plus_full_yaw():
    # yaw_bias = 0.75 ⇒ ρ = 3: at the cut, translation absorbs three
    # times the cut fraction omega does. The resulting v_x sits well
    # below uniform, omega sits well above, and the binding leg is at
    # the per-leg cap exactly.
    v_x_u, _, omega_u = scale_to_envelope(
        0.40, 0.0, 1.0, _MOUNTS, _LINEAR_MAX, _ANGULAR_MAX, _UNIFORM_BIAS
    )
    v_x_b, v_y_b, omega_b = scale_to_envelope(
        0.40, 0.0, 1.0, _MOUNTS, _LINEAR_MAX, _ANGULAR_MAX, _YAW_BIAS
    )

    assert v_x_b < v_x_u
    assert omega_b > omega_u
    assert math.isclose(v_y_b, 0.0)

    # ρ = 0.75 / 0.25 = 3 ⇒ (1 - s_v) / (1 - s_w) = 3.
    s_v = v_x_b / 0.40
    s_w = omega_b / 1.0
    assert math.isclose((1.0 - s_v) / (1.0 - s_w), 3.0, rel_tol=1e-9)

    # Binding leg lands on the per-leg cap, no overshoot.
    max_v = 0.0
    for r_x, r_y, _ in _MOUNTS.values():
        max_v = max(
            max_v, math.hypot(v_x_b - omega_b * r_y, v_y_b + omega_b * r_x)
        )
    assert math.isclose(max_v, _LINEAR_MAX, rel_tol=1e-9)


def test_scale_clamps_omega_to_angular_max_first():
    # omega beyond cap is clamped before the cut. With v=0 and omega
    # clamped to 1.0, max leg speed is omega * r_outer ≈ 0.101, which
    # is well under linear_max, so no further cut happens.
    out = scale_to_envelope(
        0.0, 0.0, 5.0, _MOUNTS, _LINEAR_MAX, _ANGULAR_MAX, _YAW_BIAS
    )
    assert math.isclose(out[0], 0.0)
    assert math.isclose(out[1], 0.0)
    assert math.isclose(out[2], _ANGULAR_MAX)


def test_scale_clamps_negative_omega():
    out = scale_to_envelope(
        0.0, 0.0, -5.0, _MOUNTS, _LINEAR_MAX, _ANGULAR_MAX, _YAW_BIAS
    )
    assert math.isclose(out[2], -_ANGULAR_MAX)


def test_scale_biased_cut_favours_yaw_for_lateral_plus_yaw():
    # v_y exercises the r_x-coupled term — the cut split has to handle
    # the lateral direction the same way as forward.
    v_x_b, v_y_b, omega_b = scale_to_envelope(
        0.0, 0.40, 1.0, _MOUNTS, _LINEAR_MAX, _ANGULAR_MAX, _YAW_BIAS
    )
    max_v = 0.0
    for r_x, r_y, _ in _MOUNTS.values():
        max_v = max(
            max_v, math.hypot(v_x_b - omega_b * r_y, v_y_b + omega_b * r_x)
        )
    assert math.isclose(max_v, _LINEAR_MAX, rel_tol=1e-9)
    assert math.isclose(v_x_b, 0.0)
    s_v = v_y_b / 0.40
    s_w = omega_b / 1.0
    assert math.isclose((1.0 - s_v) / (1.0 - s_w), 3.0, rel_tol=1e-9)


def test_scale_yaw_only_violation_zeros_translation():
    # Slow-gait corner: angular_max * r_outer exceeds linear_max, so
    # omega alone breaks the per-leg envelope. The bias-toward-yaw
    # contract pins translation at zero and scales omega to fit.
    tiny_linear = 0.05  # angular_max * r_outer ≈ 0.101 > 0.05
    v_x, v_y, omega = scale_to_envelope(
        0.10, 0.0, 1.0, _MOUNTS, tiny_linear, _ANGULAR_MAX, _YAW_BIAS
    )
    assert math.isclose(v_x, 0.0)
    assert math.isclose(v_y, 0.0)
    # omega scales to tiny_linear / (angular_max * r_outer).
    r_outer = max(math.hypot(r_x, r_y) for r_x, r_y, _ in _MOUNTS.values())
    assert math.isclose(omega, tiny_linear / r_outer, rel_tol=1e-9)


def test_scale_uses_per_gait_linear_max(tmp_path):
    # The whole point of the refactor: passing a smaller linear_max
    # (e.g. wave's cap) cuts the command down accordingly.
    path = _write_yaml(tmp_path)
    caps = load_velocity_caps(path)
    v_x, _, omega = scale_to_envelope(
        0.40,
        0.0,
        0.0,
        _MOUNTS,
        caps.linear_max("wave"),
        caps.angular_max,
        caps.yaw_bias("wave"),
    )
    # 0.40 was tripod's cap; wave cap is 0.08, so the input gets scaled
    # to 0.08 (no yaw → max leg speed = |v_x|, bias is irrelevant).
    assert math.isclose(v_x, 0.08, rel_tol=1e-9)
    assert math.isclose(omega, 0.0)
