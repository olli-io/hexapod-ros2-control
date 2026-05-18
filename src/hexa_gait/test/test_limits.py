import math
from pathlib import Path

import pytest
import yaml

from hexa_gait.limits import VelocityCaps, load_velocity_caps, scale_to_envelope


def _write_yaml(tmp_path: Path, **overrides) -> Path:
    base = dict(
        stride_length=0.12,
        min_cycle_time=0.6,
        max_cycle_time=2.0,
        duty_factor=0.5,
        step_height=0.035,
        swing_width=0.0,
        controller_dt=0.02,
        cmd_zero_tol=1.0e-4,
        recenter_swing_time=0.4,
        angular_z_max=1.0,
    )
    base.update(overrides)
    path = tmp_path / "gait.yaml"
    path.write_text(yaml.safe_dump(base))
    return path


def test_linear_max_derived_from_stride_min_cycle_and_duty(tmp_path):
    # 0.12 / (0.6 * 0.5) = 0.40 m/s
    path = _write_yaml(tmp_path)
    caps = load_velocity_caps(path)
    assert isinstance(caps, VelocityCaps)
    assert math.isclose(caps.linear_max, 0.40)


def test_linear_max_scales_with_stride_length(tmp_path):
    # Double stride_length → double linear_max.
    path = _write_yaml(tmp_path, stride_length=0.24)
    caps = load_velocity_caps(path)
    assert math.isclose(caps.linear_max, 0.80)


def test_linear_max_scales_inversely_with_min_cycle_time(tmp_path):
    # Slower min_cycle_time → lower linear_max.
    path = _write_yaml(tmp_path, min_cycle_time=1.2)
    caps = load_velocity_caps(path)
    assert math.isclose(caps.linear_max, 0.20)


def test_angular_max_passes_through_raw(tmp_path):
    # angular_z_max is an explicit knob, not derived from geometry.
    path = _write_yaml(tmp_path, angular_z_max=1.5)
    caps = load_velocity_caps(path)
    assert math.isclose(caps.angular_max, 1.5)


def test_missing_angular_z_max_raises(tmp_path):
    raw = {
        "stride_length": 0.12,
        "min_cycle_time": 0.6,
        "duty_factor": 0.5,
    }
    path = tmp_path / "gait.yaml"
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(KeyError):
        load_velocity_caps(path)


def test_accepts_string_path(tmp_path):
    path = _write_yaml(tmp_path)
    caps = load_velocity_caps(str(path))
    assert math.isclose(caps.linear_max, 0.40)


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
_CAPS = VelocityCaps(linear_max=0.40, angular_max=1.0)


def test_scale_passthrough_when_within_envelope():
    # Modest forward + modest yaw — every leg under 0.40 m/s.
    out = scale_to_envelope(0.1, 0.0, 0.5, _MOUNTS, _CAPS)
    assert math.isclose(out[0], 0.1)
    assert math.isclose(out[1], 0.0)
    assert math.isclose(out[2], 0.5)


def test_scale_zero_command_stays_zero():
    out = scale_to_envelope(0.0, 0.0, 0.0, _MOUNTS, _CAPS)
    assert out == (0.0, 0.0, 0.0)


def test_scale_pure_linear_at_cap_unchanged():
    # v_x = linear_max, no yaw: max leg speed equals the cap exactly,
    # so the joint scale must be a no-op.
    out = scale_to_envelope(0.40, 0.0, 0.0, _MOUNTS, _CAPS)
    assert math.isclose(out[0], 0.40)
    assert math.isclose(out[1], 0.0)
    assert math.isclose(out[2], 0.0)


def test_scale_full_forward_plus_full_yaw_preserves_ratio():
    # The motivating case: stick fully forward + max yaw. Without joint
    # scaling the engine's per-leg stride clamp would silently shrink
    # the outer legs and eat the yaw. The fix is a single shared scale
    # so the commanded (v_x, omega_z) ratio survives.
    v_x, v_y, omega_z = scale_to_envelope(0.40, 0.0, 1.0, _MOUNTS, _CAPS)

    # Verify max-leg speed lands exactly at the cap (no headroom wasted,
    # no overshoot).
    max_v = 0.0
    for r_x, r_y, _ in _MOUNTS.values():
        max_v = max(max_v, math.hypot(v_x - omega_z * r_y, v_y + omega_z * r_x))
    assert math.isclose(max_v, _CAPS.linear_max, rel_tol=1e-9)

    # Translation:yaw ratio preserved (input ratio 0.40 / 1.0 == 0.40).
    assert math.isclose(v_x / omega_z, 0.40, rel_tol=1e-9)
    assert math.isclose(v_y, 0.0)


def test_scale_clamps_omega_to_angular_max_first():
    # omega beyond cap is clamped before joint scaling. With v=0 and
    # omega clamped to 1.0, max leg speed is omega * r_outer ≈ 0.101,
    # which is well under linear_max, so no further scaling happens.
    out = scale_to_envelope(0.0, 0.0, 5.0, _MOUNTS, _CAPS)
    assert math.isclose(out[0], 0.0)
    assert math.isclose(out[1], 0.0)
    assert math.isclose(out[2], _CAPS.angular_max)


def test_scale_clamps_negative_omega():
    out = scale_to_envelope(0.0, 0.0, -5.0, _MOUNTS, _CAPS)
    assert math.isclose(out[2], -_CAPS.angular_max)


def test_scale_lateral_plus_yaw_also_joint_scales():
    # v_y exercises the r_x-coupled term. Same joint-scale contract.
    v_x, v_y, omega_z = scale_to_envelope(0.0, 0.40, 1.0, _MOUNTS, _CAPS)
    max_v = 0.0
    for r_x, r_y, _ in _MOUNTS.values():
        max_v = max(max_v, math.hypot(v_x - omega_z * r_y, v_y + omega_z * r_x))
    assert math.isclose(max_v, _CAPS.linear_max, rel_tol=1e-9)
    # Original ratio v_y / omega_z = 0.40 preserved.
    assert math.isclose(v_y / omega_z, 0.40, rel_tol=1e-9)
