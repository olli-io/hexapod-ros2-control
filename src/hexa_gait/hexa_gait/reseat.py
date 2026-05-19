"""Reseat ladder: standing → standing at a different body height.

After the user lifts or lowers the chassis via the D-pad in posture
mode (``/body/pose.z`` non-zero), the gait engine's standing-pose joint
configuration drifts away from default. The tibia leans more,
the femur drops nearly horizontal, the stance footprint expands. Until
the user releases the D-pad and the height stabilises, that's harmless
— posture-mode IK is happy to extend the legs to wherever the foot
target lands.

Once the height has been stable for ``settle_delay`` seconds, the
``ReseatController`` walks each foot pair in turn to a new nominal
position that restores the YAML-defined standing pose joint angles at
the *current* body height. The pair order mirrors
``InitializeController.PLACE_FEET`` so the visual ladder matches the
cold-start sequence.

The body lift itself is owned by posture (``pose.z``); this controller
only repositions the feet. After the ladder completes, the engine
commits the new nominal stance and any subsequent walking happens at
the new posture.

Pure-Python module — no rclpy import. Both ``reseat_nominal_stance``
and ``ReseatController`` are unit-testable standalone, matching the
contract in CLAUDE.md.
"""

from __future__ import annotations

import math
from typing import Mapping

from hexa_kinematics.body_transform import leg_to_body
from hexa_kinematics.leg_geometry import LegSpec
from hexa_kinematics.leg_ik import forward_kinematics

from .clock import LEG_NAMES
from .gaits.base import identity_y_sign, swing_arc
from .initialize import PAIR_ORDER
from .transition import LegOutput


__all__ = [
    "PAIR_ORDER",
    "ReseatController",
    "ReseatGeometry",
    "default_geometry_from_pose",
    "reseat_nominal_stance",
]

Vec3 = tuple[float, float, float]


class ReseatGeometry:
    """Frozen snapshot of the YAML default standing pose's geometry.

    Built once at engine startup from ``standing_pose.yaml`` /
    ``geometry.yaml`` — captures the tibia-from-vertical angle and the
    foot depth below the coxa joint at the default standing pose, so
    every reseat target can be computed from a single ``target_height``
    scalar without re-reading the YAML.
    """

    __slots__ = (
        "coxa_len",
        "femur_len",
        "tibia_len",
        "tibia_from_vertical",
        "default_foot_depth",
    )

    def __init__(
        self,
        coxa_len: float,
        femur_len: float,
        tibia_len: float,
        tibia_from_vertical: float,
        default_foot_depth: float,
    ) -> None:
        self.coxa_len = coxa_len
        self.femur_len = femur_len
        self.tibia_len = tibia_len
        self.tibia_from_vertical = tibia_from_vertical
        self.default_foot_depth = default_foot_depth


def default_geometry_from_pose(
    standing_angles: tuple[float, float, float],
    leg_spec: LegSpec,
) -> ReseatGeometry:
    """Derive the reseat geometry from a standing-pose joint angle tuple.

    ``standing_angles`` is the IK-convention ``(theta_coxa, theta_femur,
    theta_tibia)`` returned by ``hexa_kinematics.joint_config.load_standing_pose``.
    ``leg_spec`` supplies the segment lengths (any leg works — they
    share lengths).

    Computes:
      * ``default_foot_depth`` — ``|foot_z|`` in the leg's coxa-mount
        frame at the default pose (FK).
      * ``tibia_from_vertical`` — the lean of the tibia from straight
        down, positive when leaning toward ``+x`` (radially outward).
    """
    th_c, th_f, th_t = standing_angles
    foot_leg = forward_kinematics((th_c, th_f, th_t), leg_spec)
    default_foot_depth = -foot_leg[2]
    # tibia direction in (r, z) is (cos(th_f + th_t), -sin(th_f + th_t)).
    # Angle from -z (straight down), positive toward +r:
    #   π/2 − (th_f + th_t).
    tibia_from_vertical = math.pi / 2.0 - (th_f + th_t)
    return ReseatGeometry(
        coxa_len=leg_spec.coxa_len,
        femur_len=leg_spec.femur_len,
        tibia_len=leg_spec.tibia_len,
        tibia_from_vertical=tibia_from_vertical,
        default_foot_depth=default_foot_depth,
    )


def reseat_nominal_stance(
    target_height_m: float,
    geometry: ReseatGeometry,
    leg_specs: Mapping[str, LegSpec],
) -> dict[str, Vec3]:
    """Body-frame nominal stance per leg at a target body height.

    For a body lifted by ``Δz = target_height_m`` from the default
    standing pose, place each foot so the standing joint configuration
    is restored *at that new height*:

      * coxa angle stays at the default (typically 0 — leg radial).
      * tibia keeps the default ``tibia_from_vertical`` lean.
      * femur drops (less above horizontal) when the body lifts, and
        rises (more above horizontal) when the body lowers, by exactly
        the amount needed to span the new foot depth.

    The returned ``z`` value compensates for the kinematics chain's
    ``apply_body_pose`` subtraction of ``pose.z``: net effect at the IK
    is a leg-frame foot at ``(r_new, 0, -(default_foot_depth + Δz))``,
    while the gait nominal in the *nominal* body frame stays at the
    same ``z = -default_foot_depth`` regardless of ``Δz``. Only the
    radial X/Y components shift per leg.

    Raises ``ValueError`` if ``target_height_m`` is outside the
    geometrically feasible range (where the arcsin argument leaves
    ``[-1, +1]`` — the femur would need to bend past ±90°).
    """
    d_new = geometry.default_foot_depth + target_height_m
    # arcsin argument: positive when (tibia projection on vertical) >
    # foot depth, i.e. femur tilts up (knee above femur joint).
    arg = (geometry.tibia_len * math.cos(geometry.tibia_from_vertical) - d_new) / geometry.femur_len
    if arg < -1.0 or arg > 1.0:
        raise ValueError(
            f"target_height_m={target_height_m:+.4f} m is outside the geometrically "
            f"feasible reseat range (arcsin arg {arg:+.4f} ∉ [-1, 1])"
        )
    alpha = math.asin(arg)
    r_new = (
        geometry.coxa_len
        + geometry.femur_len * math.cos(alpha)
        + geometry.tibia_len * math.sin(geometry.tibia_from_vertical)
    )

    out: dict[str, Vec3] = {}
    for name in LEG_NAMES:
        if name not in leg_specs:
            raise ValueError(f"leg_specs missing {name!r}")
        spec = leg_specs[name]
        body_xyz = leg_to_body((r_new, 0.0, -d_new), spec)
        # Add Δz so apply_body_pose's z-subtraction lands the foot in
        # the leg frame at -d_new. With mount_z = 0 this is equivalent
        # to nominal.z = -default_foot_depth — independent of Δz.
        out[name] = (body_xyz[0], body_xyz[1], body_xyz[2] + target_height_m)
    return out


class ReseatController:
    """PLACE_FEET-style ladder: three sequential pairs swing to new nominal.

    Mirrors ``InitializeController._tick_place_feet`` directly — same
    pair order, same swing-arc parameters, same rest-to-rest endpoint
    velocities. Difference: no LIFT_BODY phase. The body is already
    standing; the reseat only repositions the feet.

    Stateful per leg (each leg remembers its current foot position so
    non-active legs hold), which doesn't fit the pure strategy
    contract — same justification as ``InitializeController``.
    """

    def __init__(
        self,
        current_stance: Mapping[str, Vec3],
        target_stance: Mapping[str, Vec3],
        pair_swing_time: float,
        swing_clearance: float,
        swing_width: float,
        controller_dt: float,
    ) -> None:
        missing = set(LEG_NAMES) - set(current_stance)
        if missing:
            raise ValueError(f"current_stance missing legs: {sorted(missing)}")
        missing = set(LEG_NAMES) - set(target_stance)
        if missing:
            raise ValueError(f"target_stance missing legs: {sorted(missing)}")
        if pair_swing_time <= 0.0:
            raise ValueError(f"pair_swing_time must be positive; got {pair_swing_time}")

        self._current: dict[str, Vec3] = {
            n: tuple(current_stance[n]) for n in LEG_NAMES  # type: ignore[misc]
        }
        self._target: dict[str, Vec3] = {
            n: tuple(target_stance[n]) for n in LEG_NAMES  # type: ignore[misc]
        }
        self._pair_swing_time = pair_swing_time
        self._swing_clearance = swing_clearance
        self._swing_width = swing_width
        self._controller_dt = controller_dt

        self._positions: dict[str, Vec3] = dict(self._current)
        self._pair_idx = 0
        self._t_in_pair = 0.0
        self._done = False

    @property
    def done(self) -> bool:
        return self._done

    def update(self, dt: float) -> dict[str, LegOutput]:
        if self._done:
            return {
                n: LegOutput(foot_target=self._target[n], phase=0.0, stance=True)
                for n in LEG_NAMES
            }

        self._t_in_pair += dt
        phase = self._t_in_pair / self._pair_swing_time
        active = PAIR_ORDER[self._pair_idx]

        out: dict[str, LegOutput] = {}
        if phase >= 1.0:
            # Snap the active pair to their targets and advance.
            for name in active:
                self._positions[name] = self._target[name]
            self._pair_idx += 1
            self._t_in_pair = 0.0
            if self._pair_idx >= len(PAIR_ORDER):
                self._done = True
            for name in LEG_NAMES:
                out[name] = LegOutput(
                    foot_target=self._positions[name], phase=0.0, stance=True
                )
            return out

        # Mid-pair: active legs follow a rest-to-rest swing arc.
        # Endpoint velocities pinned to zero so each leg sets down
        # gently (same pattern as InitializeController.PLACE_FEET).
        for name in LEG_NAMES:
            if name in active:
                origin = self._current[name]
                target = self._target[name]
                point = swing_arc(
                    phase_in_swing=phase,
                    swing_origin=origin,
                    target=target,
                    swing_clearance=self._swing_clearance,
                    swing_width=self._swing_width,
                    identity_y_sign=identity_y_sign(target),
                    swing_time=self._pair_swing_time,
                    controller_dt=self._controller_dt,
                    swing_origin_velocity=(0.0, 0.0, 0.0),
                    swing_target_velocity=(0.0, 0.0, 0.0),
                )
                self._positions[name] = point
                out[name] = LegOutput(foot_target=point, phase=phase, stance=False)
            else:
                out[name] = LegOutput(
                    foot_target=self._positions[name], phase=0.0, stance=True
                )
        return out
