"""Schema-drift smoke tests against the real hexa_description YAML.

Complements the inline-fixture tests in ``test_leg_specs.py``: those
verify mirror logic with self-documenting numbers; these load the
installed ``geometry.yaml`` and ``standing_pose.yaml`` and assert only
that the loaders accept the real schema. Renamed keys or missing
fields would break these even if the inline fixtures still pass.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

ament_index_python = pytest.importorskip(
    "ament_index_python",
    reason="real-config smoke tests need the colcon install layout",
)
from ament_index_python.packages import (  # noqa: E402
    PackageNotFoundError,
    get_package_share_directory,
)

from hexa_kinematics.joint_config import load_joint_limits, load_standing_pose  # noqa: E402
from hexa_kinematics.leg_specs import LEG_NAMES, load_leg_specs  # noqa: E402


@pytest.fixture(scope="module")
def description_config_dir() -> Path:
    try:
        share = get_package_share_directory("hexa_description")
    except PackageNotFoundError:
        pytest.skip("hexa_description is not installed in this environment")
    return Path(share) / "config"


def test_load_leg_specs_against_real_geometry(description_config_dir: Path):
    specs = load_leg_specs(description_config_dir / "geometry.yaml")
    assert set(specs.keys()) == set(LEG_NAMES)
    for name, spec in specs.items():
        assert spec.coxa_len > 0, name
        assert spec.femur_len > 0, name
        assert spec.tibia_len > 0, name
        assert all(math.isfinite(v) for v in spec.mount_xyz), name
        assert math.isfinite(spec.mount_yaw), name


def test_load_joint_limits_against_real_geometry(description_config_dir: Path):
    limits = load_joint_limits(description_config_dir / "geometry.yaml")
    assert set(limits.keys()) == {"coxa", "femur", "tibia"}
    for joint_type, lim in limits.items():
        assert lim.lower <= lim.center <= lim.upper, joint_type
        assert lim.effort > 0, joint_type
        assert lim.velocity > 0, joint_type


def test_load_standing_pose_against_real_yaml(description_config_dir: Path):
    angles = load_standing_pose(
        description_config_dir / "standing_pose.yaml",
        description_config_dir / "geometry.yaml",
    )
    assert len(angles) == 3
    assert all(math.isfinite(a) for a in angles)

    limits = load_joint_limits(description_config_dir / "geometry.yaml")
    for joint_type, theta in zip(("coxa", "femur", "tibia"), angles):
        lim = limits[joint_type]
        assert lim.lower <= theta <= lim.upper, joint_type
