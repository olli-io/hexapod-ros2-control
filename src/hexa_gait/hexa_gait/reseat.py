"""Reseat ladder: arbitrary current foot positions → a target stance.

Used by two engine paths, with the same ladder mechanics in both:

* **Posture-height change.** After the user lifts or lowers the chassis
  via the D-pad (``/body/pose.z`` non-zero), the gait engine's
  standing-pose joint configuration drifts away from the YAML default.
  Once the height has been stable for ``settle_delay`` seconds, the
  engine kicks off a reseat with a target stance computed by
  ``reseat_nominal_stance`` — the body-frame foot positions that
  restore the default joint angles at the *current* body height.
* **Paused → standing cleanup.** After ``PauseController`` settles, the
  airborne legs sit at their (lowered) PEP XY rather than the nominal
  footprint. Holding the release past ``pause_to_reseat_delay`` runs a
  reseat back to the unchanged nominal stance so the robot looks
  visually settled.

In both cases the ladder consumes whatever foot positions the engine
hands it via ``current_stance`` — the previous nominal, post-pause
lowered XY, mid-engagement carry-over, anywhere. The pair order
mirrors ``InitializeController.PLACE_FEET`` so the visual ladder
matches the cold-start sequence.

Each pair swings both legs together for one fixed ``pair_swing_time``.
The two legs in a pair may have very different XY distances to cover
(post-pause lowering can leave one leg near nominal and its mirror
half a stride away), but they share ``phase`` and ``pair_swing_time``
so they arrive at their targets at the same instant. The swing arc
lifts straight up via ``swing_clearance`` and interpolates XY linearly
between origin and target — no body-Y lateral curve, so the geometry
is direction-agnostic.

The body lift itself is owned by posture (``pose.z``); this controller
only repositions the feet. After the ladder completes, the engine
commits the new nominal stance (height-change case) or just returns to
STAND (paused-cleanup case).

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
from .pause import LegOutput


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
    """Three sequential pairs swing from wherever they are to ``target_stance``.

    The caller supplies the actual current foot positions per leg — no
    assumption of a previous nominal footprint. Each pair's swing
    origin is snapshotted at the moment the pair becomes active (from
    the running per-leg position), so legs that have already been
    placed by an earlier pair launch their next move from the placed
    position, while legs waiting their turn hold their construction-
    time start.

    Pair mechanics:
      * Pair order matches ``InitializeController.PLACE_FEET``.
      * Both legs in a pair share the same ``pair_swing_time`` and
        ``phase`` and snap to their targets together when the window
        ends. Asymmetric XY travel distances produce simultaneous
        arrival.
      * The swing arc lifts vertically by ``swing_clearance`` and
        interpolates XY linearly along the (origin → target) chord —
        no body-Y lateral curve. The motion is direction-agnostic; it
        works for radially-outward reseats (post height-change) just
        as well as off-radial cleanups (post-pause).
      * Endpoint velocities are pinned to zero so each leg sets down
        gently — same rest-to-rest pattern as
        ``InitializeController.PLACE_FEET``.
      * After each pair snaps the ladder holds for ``pair_dwell_time``
        seconds (every leg still, stance=True, phase=0) before the
        next pair lifts. No dwell before the first pair or after the
        last — only between.

    Inactive legs hold their last reported position. ``done`` flips
    True once the final pair has snapped to its targets.

    Stateful per leg (each leg remembers its position so non-active
    legs hold), which doesn't fit the pure strategy contract — same
    justification as ``InitializeController``.
    """

    def __init__(
        self,
        current_stance: Mapping[str, Vec3],
        target_stance: Mapping[str, Vec3],
        pair_swing_time: float,
        pair_dwell_time: float,
        swing_clearance: float,
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
        if pair_dwell_time < 0.0:
            raise ValueError(f"pair_dwell_time must be non-negative; got {pair_dwell_time}")

        self._target: dict[str, Vec3] = {
            n: tuple(target_stance[n]) for n in LEG_NAMES  # type: ignore[misc]
        }
        self._pair_swing_time = pair_swing_time
        self._pair_dwell_time = pair_dwell_time
        self._swing_clearance = swing_clearance
        self._controller_dt = controller_dt

        # Running foot position per leg. Updated every tick for the
        # active pair (along their swing arc) and held constant for
        # all other legs. Seeded from ``current_stance``.
        self._positions: dict[str, Vec3] = {
            n: tuple(current_stance[n]) for n in LEG_NAMES  # type: ignore[misc]
        }
        # Snapshot of the active pair's swing origins. Captured when a
        # pair becomes active so the arc parameters stay constant for
        # the full pair window even though ``_positions`` is rewritten
        # every tick.
        self._pair_origin: dict[str, Vec3] = {}
        self._pair_idx = 0
        self._t_in_pair = 0.0
        # Countdown while the ladder is holding between two pair
        # swings. Zero outside the dwell window; set to
        # ``pair_dwell_time`` the instant a pair snaps to its targets
        # and counted down each tick. ``_seed_pair_origin`` for the
        # next pair runs when this drops to zero, so the just-settled
        # pair's final positions become the next-pair seeding origin
        # without race.
        self._dwell_remaining = 0.0
        self._done = False
        self._seed_pair_origin()

    @property
    def done(self) -> bool:
        return self._done

    def update(self, dt: float) -> dict[str, LegOutput]:
        if self._done:
            return {
                n: LegOutput(foot_target=self._target[n], phase=0.0, stance=True)
                for n in LEG_NAMES
            }

        if self._dwell_remaining > 0.0:
            # Held between two pair swings: every foot stays put. The
            # just-settled pair sits on its new target; the rest hold
            # their previous positions. Seed the next pair's origins
            # on the tick the dwell expires so the next ``update``
            # call starts the swing fresh.
            self._dwell_remaining -= dt
            if self._dwell_remaining <= 0.0:
                self._dwell_remaining = 0.0
                self._seed_pair_origin()
            return {
                n: LegOutput(foot_target=self._positions[n], phase=0.0, stance=True)
                for n in LEG_NAMES
            }

        self._t_in_pair += dt
        phase = self._t_in_pair / self._pair_swing_time
        active = PAIR_ORDER[self._pair_idx]

        out: dict[str, LegOutput] = {}
        if phase >= 1.0:
            # Snap both active legs to their targets simultaneously —
            # the shared pair window guarantees identical arrival even
            # if their XY distances differ. Advance the ladder.
            for name in active:
                self._positions[name] = self._target[name]
            self._pair_idx += 1
            self._t_in_pair = 0.0
            if self._pair_idx >= len(PAIR_ORDER):
                self._done = True
            elif self._pair_dwell_time > 0.0:
                # Hold before the next pair lifts. Seeding the next
                # pair's swing origin is deferred to dwell expiry.
                self._dwell_remaining = self._pair_dwell_time
            else:
                self._seed_pair_origin()
            for name in LEG_NAMES:
                out[name] = LegOutput(
                    foot_target=self._positions[name], phase=0.0, stance=True
                )
            return out

        # Mid-pair: both active legs follow a rest-to-rest swing arc
        # from their pair-start origin to their target. ``swing_width``
        # is hardcoded to zero so the curve is a vertical lift over a
        # linear XY chord — direction-agnostic.
        for name in LEG_NAMES:
            if name in active:
                origin = self._pair_origin[name]
                target = self._target[name]
                point = swing_arc(
                    phase_in_swing=phase,
                    swing_origin=origin,
                    target=target,
                    swing_clearance=self._swing_clearance,
                    swing_width=0.0,
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

    def _seed_pair_origin(self) -> None:
        if self._pair_idx >= len(PAIR_ORDER):
            return
        active = PAIR_ORDER[self._pair_idx]
        self._pair_origin = {name: self._positions[name] for name in active}
