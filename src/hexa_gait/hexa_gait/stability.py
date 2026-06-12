"""Static stability margin from stance-foot positions.

The README asserts that the support polygon always contains the CoM
because β and the phase offsets are chosen so it holds. This module
makes that claim checkable: ``support_polygon_margin`` returns the
signed distance (m) from the CoM's ground projection to the boundary
of the support polygon — positive when the CoM is inside, negative
when outside, with magnitude equal to the distance to the nearest
edge. The conventional quasi-static stability margin.

Assumptions match the gait engine's regime: flat ground, quasi-static
motion (no inertial terms — valid at the low speeds this engine
commands), and a CoM that projects onto the body origin unless the
caller says otherwise. Posture offsets shift the true CoM; callers
that know better pass ``com_xy``.

Pure Python, no rclpy — unit-testable standalone per the package
contract. Not wired into the runtime: it exists for tests and offline
gait evaluation.
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence


__all__ = ["support_polygon_margin"]


def _cross(o, a, b) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _convex_hull(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    """Andrew monotone chain; returns the hull in CCW order.

    Collinear boundary points are dropped. Degenerate inputs (all
    points coincident or collinear) return fewer than 3 vertices.
    """
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts
    lower: list[tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], p) <= 0.0:
            lower.pop()
        lower.append(p)
    upper: list[tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], p) <= 0.0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    return hull if len(hull) >= 3 else sorted(set(points))[:2]


def _point_segment_distance(p, a, b) -> float:
    ab_x, ab_y = b[0] - a[0], b[1] - a[1]
    ap_x, ap_y = p[0] - a[0], p[1] - a[1]
    denom = ab_x * ab_x + ab_y * ab_y
    if denom <= 0.0:
        return math.hypot(ap_x, ap_y)
    t = max(0.0, min(1.0, (ap_x * ab_x + ap_y * ab_y) / denom))
    return math.hypot(ap_x - t * ab_x, ap_y - t * ab_y)


def support_polygon_margin(
    stance_feet_xy: Iterable[tuple[float, float]],
    com_xy: tuple[float, float] = (0.0, 0.0),
) -> float:
    """Signed distance from the CoM projection to the support boundary.

    ``stance_feet_xy`` are the body-frame ground-plane positions of the
    feet currently in stance. Positive return: the CoM is statically
    supported, with that much margin to the nearest support-polygon
    edge. Negative: unsupported (the robot tips), with magnitude the
    distance back to the support set. Fewer than three non-collinear
    stance feet cannot enclose any point, so the result is always
    ``<= 0`` (negative distance to the degenerate hull).
    """
    feet = [(float(x), float(y)) for x, y in stance_feet_xy]
    if not feet:
        return -math.inf
    hull = _convex_hull(feet)
    if len(hull) < 3:
        if len(hull) == 1:
            return -math.hypot(com_xy[0] - hull[0][0], com_xy[1] - hull[0][1])
        return -_point_segment_distance(com_xy, hull[0], hull[1])

    margin = math.inf
    inside = True
    for i, a in enumerate(hull):
        b = hull[(i + 1) % len(hull)]
        edge_len = math.hypot(b[0] - a[0], b[1] - a[1])
        # Signed perpendicular distance; positive on the interior
        # (left) side of a CCW edge.
        signed = _cross(a, b, com_xy) / edge_len
        if signed < 0.0:
            inside = False
        margin = min(margin, signed)
    if inside:
        return margin
    # Outside: the min signed halfplane distance overestimates the
    # distance in corner regions; report the true distance to the hull
    # boundary, negated.
    return -min(
        _point_segment_distance(com_xy, hull[i], hull[(i + 1) % len(hull)])
        for i in range(len(hull))
    )
