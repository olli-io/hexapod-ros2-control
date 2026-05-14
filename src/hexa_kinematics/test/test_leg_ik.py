import math

import pytest

from hexa_kinematics import (
    LegSpec,
    UnreachableTarget,
    forward_kinematics,
    inverse_kinematics,
)


LEG = LegSpec(
    mount_xyz=(0.0, 0.0, 0.0),
    mount_yaw=0.0,
    coxa_len=0.05,
    femur_len=0.08,
    tibia_len=0.12,
)


def _close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_fk_zero_angles_extends_leg_along_x():
    foot = forward_kinematics((0.0, 0.0, 0.0), LEG)
    assert _close(foot, (LEG.coxa_len + LEG.femur_len + LEG.tibia_len, 0.0, 0.0))


def test_ik_extended_pose_returns_zero_angles():
    target = (LEG.coxa_len + LEG.femur_len + LEG.tibia_len, 0.0, 0.0)
    angles = inverse_kinematics(target, LEG)
    assert _close(angles, (0.0, 0.0, 0.0))


def test_ik_foot_straight_down_from_femur_joint():
    target = (LEG.coxa_len, 0.0, -(LEG.femur_len + LEG.tibia_len))
    angles = inverse_kinematics(target, LEG)
    assert _close(angles, (0.0, math.pi / 2, 0.0))


def test_ik_coxa_yaw_to_the_side():
    target = (0.0, LEG.coxa_len + LEG.femur_len + LEG.tibia_len, 0.0)
    angles = inverse_kinematics(target, LEG)
    assert _close(angles, (math.pi / 2, 0.0, 0.0))


@pytest.mark.parametrize(
    "angles",
    [
        (0.0, 0.0, 0.0),
        (0.0, 0.3, 0.4),
        (0.5, 0.2, 0.6),
        (-0.5, 0.4, 0.5),
        (1.2, -0.3, 1.0),
        (-1.0, 0.7, 0.3),
        (0.0, math.pi / 4, math.pi / 3),
    ],
)
def test_fk_ik_round_trip(angles):
    foot = forward_kinematics(angles, LEG)
    recovered = inverse_kinematics(foot, LEG)
    foot_again = forward_kinematics(recovered, LEG)
    assert _close(foot, foot_again, tol=1e-9)


def test_ik_raises_on_target_beyond_reach():
    too_far = (10.0, 0.0, 0.0)
    with pytest.raises(UnreachableTarget):
        inverse_kinematics(too_far, LEG)


def test_ik_raises_on_target_inside_inner_annulus():
    # Foot at the femur joint → d = 0, below |femur - tibia|.
    too_close = (LEG.coxa_len, 0.0, 0.0)
    with pytest.raises(UnreachableTarget):
        inverse_kinematics(too_close, LEG)
