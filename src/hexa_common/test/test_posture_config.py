"""The body-height envelope loader — absolute clearance in, offsets out."""

import math

import pytest

from hexa_common import load_body_height_offsets


_GAIT_YAML = """
gait_node:
  ros__parameters:
    default_standing_pose:
      body_height: 0.05
      front:
        tip_reach: 0.135
        coxa_deg: 0
      middle:
        tip_reach: 0.135
        coxa_deg: 0
      rear:
        tip_reach: 0.135
        coxa_deg: 0
"""

_POSTURE_YAML = """
posture_node:
  ros__parameters:
    body_height_max_m: 0.14
    body_height_min_m: 0.02
"""


def _write(tmp_path, gait=_GAIT_YAML, posture=_POSTURE_YAML):
    gait_path = tmp_path / "gait.yaml"
    posture_path = tmp_path / "posture.yaml"
    gait_path.write_text(gait)
    posture_path.write_text(posture)
    return gait_path, posture_path


def test_absolute_heights_become_offsets_from_the_stance(tmp_path):
    height_min, height_max = load_body_height_offsets(*_write(tmp_path))
    # 0.14 m of belly clearance is +0.09 m of lift off a 0.05 m stance.
    assert math.isclose(height_max, 0.09)
    assert math.isclose(height_min, -0.03)


def test_offsets_are_not_symmetric(tmp_path):
    """Guards the whole point of the change: up and down are independent."""
    height_min, height_max = load_body_height_offsets(*_write(tmp_path))
    assert not math.isclose(height_max, -height_min)


def test_moving_the_stance_does_not_move_the_ceiling(tmp_path):
    """The absolute ceiling stays put when the resting height changes.

    That is the difference from a delta-based envelope: raising the stance
    eats into the remaining lift rather than dragging the ceiling up with it.
    """
    raised = _GAIT_YAML.replace("body_height: 0.05", "body_height: 0.08")
    _, height_max = load_body_height_offsets(*_write(tmp_path, gait=raised))
    assert math.isclose(height_max, 0.14 - 0.08)


@pytest.mark.parametrize(
    "body_height", ["0.15", "0.01", "0.14", "0.02"]
)
def test_stance_outside_its_own_envelope_is_rejected(tmp_path, body_height):
    """A stance at or beyond either end would be clamped away from rest."""
    bad = _GAIT_YAML.replace("body_height: 0.05", f"body_height: {body_height}")
    with pytest.raises(ValueError, match="must bracket"):
        load_body_height_offsets(*_write(tmp_path, gait=bad))
