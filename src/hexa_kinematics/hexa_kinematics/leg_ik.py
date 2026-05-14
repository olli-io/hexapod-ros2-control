"""Per-leg forward and inverse kinematics.

Both functions operate in the leg's coxa-mount frame (see
``leg_geometry``). ``inverse_kinematics`` returns the *knee-up* branch
— the standard hexapod spider stance, where the knee sits on the
upper-z side of the chord from femur joint to foot.
"""

import math

from .leg_geometry import JointAngles, LegSpec, Point3


class UnreachableTarget(ValueError):
    """Foot target lies outside the leg's reachable workspace."""


def forward_kinematics(angles: JointAngles, spec: LegSpec) -> Point3:
    """Foot position in the coxa-mount frame, given joint angles.

    Pure mathematical FK. See ``leg_geometry`` for frame and joint-angle
    conventions.
    """
    th_c, th_f, th_t = angles
    r = (
        spec.coxa_len
        + spec.femur_len * math.cos(th_f)
        + spec.tibia_len * math.cos(th_f + th_t)
    )
    z = -spec.femur_len * math.sin(th_f) - spec.tibia_len * math.sin(th_f + th_t)
    return (r * math.cos(th_c), r * math.sin(th_c), z)


def inverse_kinematics(target: Point3, spec: LegSpec) -> JointAngles:
    """Joint angles placing the foot at ``target`` in the coxa-mount frame.

    Returns the knee-up branch. Raises ``UnreachableTarget`` if ``target``
    lies outside the workspace annulus around the femur joint.

    This is the *unconstrained* mathematical IK — it does not honour servo
    joint limits. Callers must validate the returned angles against
    ``hexa_description`` joint limits before commanding hardware.
    """
    x, y, z = target
    # At (x, y) = (0, 0) the foot is on the coxa axis and θ_coxa is
    # undetermined; ``atan2(0, 0)`` returns 0. Degenerate but harmless.
    th_c = math.atan2(y, x)

    # r_prime < 0 means the foot lies between the coxa pivot and the body
    # centre — the math still produces a valid solution (femur folds back
    # under the body), but it will almost certainly violate servo limits.
    r_prime = math.hypot(x, y) - spec.coxa_len
    d = math.hypot(r_prime, z)

    f, t = spec.femur_len, spec.tibia_len
    if d > f + t + 1e-6 or d < abs(f - t) - 1e-6:
        raise UnreachableTarget(
            f"foot {target} is {d:.4f} m from the femur joint; "
            f"reach annulus is [{abs(f - t):.4f}, {f + t:.4f}] m"
        )

    # Floating-point safety: arguments may slip outside [-1, 1] right at
    # the workspace boundary.
    cos_beta = max(-1.0, min(1.0, (f * f + d * d - t * t) / (2.0 * f * d)))
    cos_gamma = max(-1.0, min(1.0, (f * f + t * t - d * d) / (2.0 * f * t)))

    alpha = math.atan2(-z, r_prime)
    beta = math.acos(cos_beta)
    gamma = math.acos(cos_gamma)

    # Knee-up branch: femur sits above the chord from femur joint to foot.
    th_f = alpha - beta
    th_t = math.pi - gamma
    return (th_c, th_f, th_t)
