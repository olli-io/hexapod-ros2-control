"""Velocity caps derived from the velocity envelope — single source of truth.

Downstream nodes (``hexa_teleop`` for stick scaling, ``hexa_control`` for
``/cmd_vel`` clamping) used to keep hand-aligned copies of the linear /
angular maxima. They drift in practice and hide the relationship to the
gait's actual physical reach. This module replaces those copies.

Linear cap is **per-gait** because the per-leg velocity ceiling depends
on the active strategy's duty factor (β):

    linear_max(gait) = stride_length * (1 − β) / (min_swing_time × β_gait)

That is exactly the per-leg velocity ceiling the gait saturates at:
beyond it the engine pins ``cycle_time`` at ``min_swing_time / (1 − β)``
and the per-leg stride magnitude is capped at ``stride_length``.
Tripod (β=0.5) sits at the high end of this curve; crawl (β=2/3) is
slower; ripple (β=5/6) is slowest. Clamping ``/cmd_vel`` at the active
gait's cap keeps the engine's stance integration bounded — anything
higher would push the engagement controller's stance feet past PEP and
hit joint limits.

Angular cap is **per-gait** too, and derived rather than tuned:

    angular_max(gait) = linear_max(gait) / r_outer

where ``r_outer`` is the planar radius of the outermost foot in the
standing pose. Turning in place makes each foot trace a circle of its
own standing radius, so the outermost foot is the one that hits
``linear_max`` first, and that is the whole story — there is no
separate angular ceiling to clamp against.

An explicit ``angular_z_max`` knob used to live in the envelope YAML.
It was set to 6 rad/s against a geometric ceiling near 0.6, so it never
limited anything; all it did was skew the two things that scale off it
(joystick full-scale deflection and the yaw acceleration ramp). A
derived value cannot drift like that. To change turn rate, change what
actually governs it: ``stride_length``, ``min_swing_time``, the gait's
duty factor, or the stance width (``standing_pose.tip_radius``).

``scale_to_envelope`` cuts ``(v_x, v_y, omega_z)`` so the implied
per-leg planar speed never exceeds ``linear_max``. This is the right
place for that scaling — at the ``/cmd_vel`` boundary — because the
engine's per-leg stride clamp would otherwise distort the command (the
outer legs clip while inner legs don't, which silently eats yaw at full
forward + full yaw).

The cut between translation and yaw is split in ratio
``yaw_bias : (1 − yaw_bias)`` — translation absorbs the larger
fraction, so at the extremes the resulting motion keeps more of the
commanded yaw. ``yaw_bias = 0.5`` recovers the unbiased (uniform)
behaviour where both components scale by the same factor. The trade is
explicit: the commanded translation:yaw ratio is not preserved when
the cut kicks in.

The effective bias is **per-gait**, easing back toward neutral as
duty factor grows:
``yaw_bias_eff(β) = 0.5 + (yaw_bias − 0.5) · (1.5 − β)``. The envelope
value anchors at tripod (β=0.5); slower gaits (crawl β=2/3 → 0.583,
ripple β=5/6 → 0.567) sit closer to neutral. The motivation is that the
slower a gait runs, the smaller its ``linear_max`` already is —
pushing more of the envelope cut onto translation on top of that is
too aggressive. Tripod, with the most headroom in both directions,
can afford the strongest yaw priority.

Pure-python; ``rclpy``-free so the helper is unit-testable standalone
and importable from teleop, control, and the gait engine without ROS
overhead. Duty factors come from the pure ``gait_catalog``; the numpy
strategy implementations are never imported.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

import yaml

from .gait_catalog import GAIT_DESCRIPTORS


@dataclass(frozen=True)
class VelocityCaps:
    """Per-gait linear caps, per-gait angular caps, and per-gait yaw bias.

    Callers look up by gait name via ``linear_max(name)`` /
    ``angular_max(name)`` / ``yaw_bias(name)``; all three dicts are keyed
    by the same strings the gait catalog uses (``GAIT_DESCRIPTORS``).
    Unknown names raise ``KeyError`` deliberately — a typo at the control
    layer should fail fast rather than silently fall back to the wrong cap.
    """

    linear_max_by_gait: Mapping[str, float]
    angular_max_by_gait: Mapping[str, float]
    yaw_bias_by_gait: Mapping[str, float]

    def linear_max(self, gait: str) -> float:
        return self.linear_max_by_gait[gait]

    def angular_max(self, gait: str) -> float:
        return self.angular_max_by_gait[gait]

    def yaw_bias(self, gait: str) -> float:
        return self.yaw_bias_by_gait[gait]


def standing_stance_xy(
    geometry_yaml: str | Path, envelope_yaml: str | Path
) -> dict[str, tuple[float, float]]:
    """Planar body-frame position of each standing foot.

    The lever arms a yaw command acts through. Mirrors two things the C++
    side does with IK: ``load_leg_specs``' symmetry expansion of the two
    reference mounts in ``geometry.yaml``, and ``standing_pose_from``'s
    placement of the tip ``tip_radius`` out from the coxa axis at the
    leg's splay angle. Solved in closed form here — the round trip through
    IK and FK lands on the same point, and this keeps the helper free of a
    kinematics dependency.
    """
    geo = yaml.safe_load(Path(geometry_yaml).read_text())
    tuning = yaml.safe_load(Path(envelope_yaml).read_text())
    standing = tuning["gait_node"]["ros__parameters"]["standing_pose"]
    tip_radius = float(standing["tip_radius"])
    corner_splay = math.radians(float(standing["corner_leg_coxa_deg"]))

    mounts = geo["mounts"]
    out: dict[str, tuple[float, float]] = {}
    for side in ("l", "r"):
        for name in ("front", "middle", "rear"):
            ref = mounts["l_middle" if name == "middle" else "l_front"]
            ref_yaw = math.radians(float(ref["yaw_deg"]))
            ref_x, ref_y = float(ref["x"]), float(ref["y"])

            # Rear mirrors fore/aft, right mirrors left/right.
            x = -ref_x if name == "rear" else ref_x
            yaw = (math.pi - ref_yaw) if name == "rear" else ref_yaw
            y = -ref_y if side == "r" else ref_y
            if side == "r":
                yaw = -yaw

            # Splay: middle legs point straight out; the corner legs mirror the
            # front-left leg's swivel, negated on the two diagonally opposite
            # ones (same rule as standing_pose_from / kInitialPose).
            if name == "middle":
                splay = 0.0
            elif (side, name) in (("l", "rear"), ("r", "front")):
                splay = -corner_splay
            else:
                splay = corner_splay

            out[f"{side}_{name}"] = (
                x + tip_radius * math.cos(yaw + splay),
                y + tip_radius * math.sin(yaw + splay),
            )
    return out


def outer_stance_radius(
    geometry_yaml: str | Path, envelope_yaml: str | Path
) -> float:
    """Planar radius of the outermost standing foot.

    ``linear_max / this`` is the fastest yaw the gait can lay down, since
    turning in place makes every foot trace a circle of its own radius and
    the outermost one saturates first.
    """
    radii = [
        math.hypot(x, y)
        for x, y in standing_stance_xy(geometry_yaml, envelope_yaml).values()
    ]
    r_outer = max(radii, default=0.0)
    if r_outer <= 0.0:
        raise ValueError(
            "outer stance radius is zero — every foot sits on the body axis, "
            "so the standing pose has no yaw authority; check tuning.yaml "
            "standing_pose"
        )
    return r_outer


def unit_stance_xy(
    geometry_yaml: str | Path, envelope_yaml: str | Path
) -> tuple[tuple[float, float], ...]:
    """Standing stance divided by the outermost foot's radius.

    The lever arms of :func:`standing_stance_xy`, made unitless. Teleop
    shapes stick input against the same per-leg foot-speed constraint the
    engine enforces, but in stick units rather than m/s; normalising the
    stance by ``r_outer`` is exactly what cancels the caps out of that
    constraint, because every gait's angular cap is its linear cap over
    that radius. One table therefore serves every gait.

    Returned as a tuple of pairs, not a name-keyed mapping: the consumer
    only ever takes a max across the legs, so which leg is which never
    comes up.
    """
    stance = standing_stance_xy(geometry_yaml, envelope_yaml)
    r_outer = outer_stance_radius(geometry_yaml, envelope_yaml)
    return tuple((x / r_outer, y / r_outer) for x, y in stance.values())


def load_velocity_caps(
    envelope_yaml: str | Path, geometry_yaml: str | Path
) -> VelocityCaps:
    """Build per-gait caps from the velocity-envelope YAML and the catalog.

    Duty factor is **not** in YAML — it lives on each gait descriptor in
    ``hexa_common.gait_catalog`` and is the single source of truth. We
    enumerate ``GAIT_DESCRIPTORS`` here so a new gait shows up in the caps
    map as soon as its descriptor is registered; the envelope YAML only
    contributes the gait-agnostic knobs (``stride_length``,
    ``min_swing_time``, ``yaw_bias``).

    ``envelope_yaml`` is hexa_description's ``tuning.yaml`` (or any ros2
    params file carrying a ``gait_node`` block); the caps are read from
    that block. ``geometry_yaml`` is its sibling ``geometry.yaml`` — needed
    for the leg mounts, since the angular cap is the linear one divided by
    the outermost foot's standing radius.
    """
    path = Path(envelope_yaml)
    with path.open() as f:
        raw = yaml.safe_load(f)
    # tuning.yaml is a ros2 params file; unwrap to the flat knob block the caps
    # are derived from (mirrors the gait node's ros params).
    raw = raw["gait_node"]["ros__parameters"]

    stride_length = float(raw["stride_length"])
    min_swing_time = float(raw["min_swing_time"])
    yaw_bias = float(raw["yaw_bias"])
    margin = float(raw.get("swing_phase_margin", 0.0))
    r_outer = outer_stance_radius(geometry_yaml, envelope_yaml)

    linear_max_by_gait: dict[str, float] = {}
    angular_max_by_gait: dict[str, float] = {}
    yaw_bias_by_gait: dict[str, float] = {}
    for name, descriptor in GAIT_DESCRIPTORS.items():
        duty = descriptor.duty_factor
        # The cap is stride_length covered in one stance, so it keys off the
        # realized swing/stance split (swing_end_phase in gaits/base.cpp), not
        # the nominal duty factor: the swing phase margin lengthens stance and
        # lowers top speed. Keep identical to gen_config.py's velocity_caps()
        # and pipeline_config_loader.cpp.
        swing_end = (1.0 - duty) * (1.0 - min(max(margin, 0.0), 0.4))
        linear_max = (
            stride_length * swing_end / (min_swing_time * (1.0 - swing_end))
        )
        linear_max_by_gait[name] = linear_max
        # Same lever arm for every gait; only the linear cap it divides differs.
        angular_max_by_gait[name] = linear_max / r_outer
        # yaw_bias stays keyed to the gait's nominal duty: it is a feel knob for
        # how a gait gives way under a saturating command, not a timing budget.
        yaw_bias_by_gait[name] = 0.5 + (yaw_bias - 0.5) * (1.5 - duty)
    return VelocityCaps(
        linear_max_by_gait=linear_max_by_gait,
        angular_max_by_gait=angular_max_by_gait,
        yaw_bias_by_gait=yaw_bias_by_gait,
    )


def scale_to_envelope(
    v_x: float,
    v_y: float,
    omega_z: float,
    stance_xy: Mapping[str, Sequence[float]],
    linear_max: float,
    yaw_bias: float,
) -> tuple[float, float, float]:
    """Cut the velocity triple to fit the gait envelope.

    Applied at the ``/cmd_vel`` boundary in two steps:

    1. The implied per-leg planar speed is computed across all six
       legs as ``|(v_x - omega_z·r_y, v_y + omega_z·r_x)|``. If the
       maximum is at or under ``linear_max`` the inputs pass through.
    2. Otherwise the reduction is split in ratio
       ``yaw_bias : (1 − yaw_bias)`` between translation and yaw —
       translation absorbs the larger share, so the resulting motion
       keeps more of the commanded yaw at the extremes.

    There is no separate angular clamp. Bounding every foot's speed to
    ``linear_max`` bounds ``omega_z`` to ``linear_max / r_outer`` on its
    own, which is exactly ``VelocityCaps.angular_max`` — the cap is a
    consequence of step 1, not an input to it.

    Concretely, parametrise the cut as ``s_v = 1 − ρ·t`` (applied to
    ``v_x``, ``v_y``) and ``s_w = 1 − t`` (applied to ``omega_z``) with
    ``ρ = yaw_bias / (1 − yaw_bias)``. The per-leg constraint
    ``|(s_v·v_x − s_w·omega_z·r_y, s_v·v_y + s_w·omega_z·r_x)| ≤
    linear_max`` becomes a quadratic in ``t`` per leg; the smallest
    positive root is the minimum cut that leg needs, and the binding
    cut is the maximum across legs. ``yaw_bias = 0.5`` makes ``ρ = 1``
    and recovers uniform scaling (direction preserved). Values above
    0.5 favour yaw; ``yaw_bias → 1`` approaches pure yaw priority.

    If the biased cut would drive ``s_v`` negative (the bias asks
    translation to scale past zero), translation is pinned at zero and
    ``omega_z`` is scaled alone to fit the per-leg cap. That happens
    whenever the commanded yaw alone already needs more than the whole
    envelope, i.e. ``|omega_z| · r_outer > linear_max``.

    ``stance_xy`` maps leg name to the leg's standing foot position in
    the body frame; only the first two components are read, so both
    ``(x, y)`` and ``(x, y, z)`` forms work. The foot,
    not the mount, is the lever arm a yaw rate acts through — it is where
    the gait engine lays the stride down. ``linear_max`` / ``yaw_bias``
    are passed in as scalars so the caller can look them up per active
    gait via ``VelocityCaps``.
    """
    cap_sq = linear_max * linear_max
    max_leg_v_sq = 0.0
    for r_x, r_y in (r[:2] for r in stance_xy.values()):
        vlx = v_x - omega_z * r_y
        vly = v_y + omega_z * r_x
        v_sq = vlx * vlx + vly * vly
        if v_sq > max_leg_v_sq:
            max_leg_v_sq = v_sq

    if max_leg_v_sq <= cap_sq:
        return v_x, v_y, omega_z

    rho = yaw_bias / (1.0 - yaw_bias)

    t_required = 0.0
    feasible = True
    for r_x, r_y in (r[:2] for r in stance_xy.values()):
        a0 = v_x - omega_z * r_y
        a1 = rho * v_x - omega_z * r_y
        b0 = v_y + omega_z * r_x
        b1 = rho * v_y + omega_z * r_x
        c = a0 * a0 + b0 * b0 - cap_sq
        if c <= 0.0:
            continue
        a = a1 * a1 + b1 * b1
        if a <= 0.0:
            feasible = False
            break
        b = -2.0 * (a0 * a1 + b0 * b1)
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            feasible = False
            break
        t_leg = (-b - math.sqrt(disc)) / (2.0 * a)
        if t_leg > t_required:
            t_required = t_leg

    if not feasible or rho * t_required >= 1.0:
        max_r = 0.0
        for r_x, r_y in (r[:2] for r in stance_xy.values()):
            r = math.hypot(r_x, r_y)
            if r > max_r:
                max_r = r
        omega_v_outer = abs(omega_z) * max_r
        if omega_v_outer > linear_max:
            return 0.0, 0.0, omega_z * (linear_max / omega_v_outer)
        return 0.0, 0.0, omega_z

    s_v = 1.0 - rho * t_required
    s_w = 1.0 - t_required
    return v_x * s_v, v_y * s_v, omega_z * s_w
