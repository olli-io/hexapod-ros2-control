import math
from pathlib import Path

import pytest

from hexa_kinematics.leg_specs import LEG_NAMES, load_leg_specs


@pytest.fixture
def geometry_yaml(tmp_path: Path) -> Path:
    # Minimal geometry.yaml that mirrors the structure expected by the
    # loader. Numbers are easy to verify by eye.
    text = """
leg:
  coxa_length: 0.04
  femur_length: 0.07
  tibia_length: 0.10
mounts:
  l_front:  {x: 0.08, y: 0.07, yaw_deg: 45}
  l_middle: {x: 0.00, y: 0.09, yaw_deg: 90}
"""
    path = tmp_path / "geometry.yaml"
    path.write_text(text)
    return path


def _close(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_loads_all_six_legs(geometry_yaml: Path):
    legs = load_leg_specs(geometry_yaml)
    assert set(legs.keys()) == set(LEG_NAMES)


def test_segment_lengths_propagate(geometry_yaml: Path):
    spec = load_leg_specs(geometry_yaml)["l_front"]
    assert math.isclose(spec.coxa_len, 0.04)
    assert math.isclose(spec.femur_len, 0.07)
    assert math.isclose(spec.tibia_len, 0.10)


# Fixture yaw values are whole degrees; convert via math.radians here
# so the assertions match exactly what the loader produces.
_FRONT_YAW = math.radians(45)
_MIDDLE_YAW = math.radians(90)


def test_left_legs_match_reference_mounts(geometry_yaml: Path):
    legs = load_leg_specs(geometry_yaml)
    assert _close(legs["l_front"].mount_xyz, (0.08, 0.07, 0.0))
    assert math.isclose(legs["l_front"].mount_yaw, _FRONT_YAW)
    assert _close(legs["l_middle"].mount_xyz, (0.0, 0.09, 0.0))
    assert math.isclose(legs["l_middle"].mount_yaw, _MIDDLE_YAW)


def test_rear_mirrors_front_about_y_axis(geometry_yaml: Path):
    legs = load_leg_specs(geometry_yaml)
    # rear: x → -x, yaw → pi - yaw
    assert _close(legs["l_rear"].mount_xyz, (-0.08, 0.07, 0.0))
    assert math.isclose(legs["l_rear"].mount_yaw, math.pi - _FRONT_YAW)


def test_right_mirrors_left_about_x_axis(geometry_yaml: Path):
    legs = load_leg_specs(geometry_yaml)
    # right: y → -y, yaw → -yaw  (applied after the front/rear mirror)
    assert _close(legs["r_front"].mount_xyz, (0.08, -0.07, 0.0))
    assert math.isclose(legs["r_front"].mount_yaw, -_FRONT_YAW)
    assert _close(legs["r_rear"].mount_xyz, (-0.08, -0.07, 0.0))
    assert math.isclose(legs["r_rear"].mount_yaw, -(math.pi - _FRONT_YAW))
    assert _close(legs["r_middle"].mount_xyz, (0.0, -0.09, 0.0))
    assert math.isclose(legs["r_middle"].mount_yaw, -_MIDDLE_YAW)
