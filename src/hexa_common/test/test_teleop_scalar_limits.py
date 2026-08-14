"""The posture-mode scalar limit loader — degrees in, radians out."""

import math

import pytest
import yaml

from hexa_common import load_posture_scalar_limits


_ENVELOPE = {
    "pose_limit_x": 0.05,
    "pose_limit_y": 0.05,
    "pose_limit_roll": 0.30,
    "pose_limit_pitch": 0.30,
    "pose_limit_yaw": 0.50,
}

_POSTURE = {
    "x_max": 0.04,
    "y_max": 0.04,
    "roll_max_deg": 15.0,
    "pitch_max_deg": 15.0,
    "yaw_max_deg": 25.0,
    "yaw_tau_s": 0.10,
    "revert_tau_s": 0.25,
    "wiggle_pivot_forward_m": 0.06,
}


def _write(tmp_path, **posture_overrides):
    path = tmp_path / "tuning.yaml"
    path.write_text(yaml.safe_dump({
        "posture_node": {"ros__parameters": dict(_ENVELOPE)},
        "teleop_node": {
            "ros__parameters": {"posture": {**_POSTURE, **posture_overrides}}
        },
    }))
    return path


def test_angles_are_converted_to_radians(tmp_path):
    limits = load_posture_scalar_limits(_write(tmp_path))
    assert math.isclose(limits.roll_max, math.radians(15.0))
    assert math.isclose(limits.pitch_max, math.radians(15.0))
    assert math.isclose(limits.yaw_max, math.radians(25.0))


def test_metres_and_time_constants_pass_through(tmp_path):
    limits = load_posture_scalar_limits(_write(tmp_path))
    assert math.isclose(limits.x_max, 0.04)
    assert math.isclose(limits.y_max, 0.04)
    assert math.isclose(limits.yaw_tau, 0.10)
    assert math.isclose(limits.revert_tau, 0.25)
    assert math.isclose(limits.wiggle_pivot_forward_m, 0.06)


@pytest.mark.parametrize(
    ("key", "value", "envelope_key"),
    [
        ("x_max", 0.06, "pose_limit_x"),
        ("y_max", 0.06, "pose_limit_y"),
        ("roll_max_deg", 20.0, "pose_limit_roll"),
        ("pitch_max_deg", 20.0, "pose_limit_pitch"),
        ("yaw_max_deg", 30.0, "pose_limit_yaw"),
    ],
)
def test_a_limit_past_the_posture_envelope_is_rejected(
    tmp_path, key, value, envelope_key
):
    """Travel the posture stack would clamp away is a config error, not a shrug.

    The last part of that stick throw would move nothing.
    """
    with pytest.raises(ValueError, match=envelope_key):
        load_posture_scalar_limits(_write(tmp_path, **{key: value}))


def test_a_limit_exactly_on_the_envelope_is_allowed(tmp_path):
    """The stick reaching the clamp is fine; only reaching past it is not."""
    limits = load_posture_scalar_limits(_write(tmp_path, x_max=0.05))
    assert math.isclose(limits.x_max, 0.05)
