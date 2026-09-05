"""The body-height envelope loader — absolute clearance in, offsets out."""

import math

import pytest

from hexa_common import load_body_height_offsets


_GAIT_YAML = """
gait_node:
  ros__parameters:
    default_preset: normal
    presets:
      - id: normal
        leg_set: hexapod
        standing_pose:
          body_height: 0.05
          front: {tip_reach: 0.135, coxa_deg: 0}
          middle: {tip_reach: 0.135, coxa_deg: 0}
          rear: {tip_reach: 0.135, coxa_deg: 0}
        stride_length: 0.105
        stride_length_radial: 0.085
        min_swing_time: 0.6
        max_swing_time: 0.8
        step_height: 0.04
"""

# A second preset that stands somewhere else. The envelope has to bracket EVERY
# preset's nominal, not just the boot one's: a preset change re-plants onto that
# height, and a clamped nominal would put the body somewhere nobody asked for.
_TWO_PRESET_YAML = _GAIT_YAML.rstrip() + """
      - id: offroad
        leg_set: hexapod
        standing_pose:
          body_height: 0.095
          front: {tip_reach: 0.125, coxa_deg: 0}
          middle: {tip_reach: 0.125, coxa_deg: 0}
          rear: {tip_reach: 0.125, coxa_deg: 0}
        stride_length: 0.085
        stride_length_radial: 0.070
        min_swing_time: 0.75
        max_swing_time: 1.00
        step_height: 0.065
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


def test_the_nominal_is_the_boot_presets_stance(tmp_path):
    # Several presets stand at different heights; the offsets are measured from
    # the one the robot boots on, which is what PostureController rests at.
    lo, hi = load_body_height_offsets(*_write(tmp_path, gait=_TWO_PRESET_YAML))
    assert math.isclose(lo, 0.02 - 0.05)
    assert math.isclose(hi, 0.14 - 0.05)


def test_a_non_boot_preset_outside_the_envelope_is_rejected(tmp_path):
    # The check is over every preset, not just the boot one: tapping OFFROAD
    # would re-plant the body at a height the envelope clamps.
    bad = _TWO_PRESET_YAML.replace("body_height: 0.095", "body_height: 0.15")
    with pytest.raises(ValueError, match="presets.offroad"):
        load_body_height_offsets(*_write(tmp_path, gait=bad))
