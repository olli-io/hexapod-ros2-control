"""Quartic Bezier foot-tip trajectory.

Python port of ``docs/trajectory-generation/control_points.cpp`` (the
Syropod walk-controller curves). One step cycle of a single leg is
described by three 5-control-node quartic Bezier curves:

- primary swing  — lift-off to apex
- secondary swing — apex to touchdown
- stance         — touchdown back to lift-off

Nodes are placed for C0 (position), C1 (velocity), and where possible
C2 (acceleration) continuity at the joins, eliminating velocity/accel
discontinuities at touchdown and lift-off.

We **sample** ``B(t)`` directly each tick rather than integrating
``dB/dt``; the engine treats each strategy as a pure function of phase.
The C++ ``ground_contact`` branch and ``forceNormalTouchdown`` override
are omitted — flat sim only in v1.

All arrays are ``(5, 3)`` float64 numpy. Vectors are length-3.
"""

from __future__ import annotations

import numpy as np


__all__ = [
    "quartic_bezier",
    "quartic_bezier_dot",
    "generate_primary_swing_control_nodes",
    "generate_secondary_swing_control_nodes",
    "generate_stance_control_nodes",
]


def quartic_bezier(points: np.ndarray, t: float) -> np.ndarray:
    """Evaluate the quartic Bezier curve B(t) for ``t in [0, 1]``.

    ``points`` must be shape ``(5, 3)`` (P0..P4). Bernstein basis::

        B(t) = (1-t)^4 P0 + 4(1-t)^3 t P1 + 6(1-t)^2 t^2 P2
             + 4(1-t) t^3 P3 + t^4 P4
    """
    if points.shape != (5, 3):
        raise ValueError(f"points must be shape (5, 3); got {points.shape}")
    s = 1.0 - t
    b0 = s * s * s * s
    b1 = 4.0 * s * s * s * t
    b2 = 6.0 * s * s * t * t
    b3 = 4.0 * s * t * t * t
    b4 = t * t * t * t
    return (
        b0 * points[0]
        + b1 * points[1]
        + b2 * points[2]
        + b3 * points[3]
        + b4 * points[4]
    )


def quartic_bezier_dot(points: np.ndarray, t: float) -> np.ndarray:
    """Evaluate ``dB/dt`` of the quartic Bezier curve at ``t``.

    Useful for checking C1 continuity at curve joins in tests; the
    engine itself only needs B(t).
    """
    if points.shape != (5, 3):
        raise ValueError(f"points must be shape (5, 3); got {points.shape}")
    s = 1.0 - t
    # d/dt of the Bernstein basis collapses to 4 * (degree-3 Bernstein
    # over the differences of successive control points).
    d0 = points[1] - points[0]
    d1 = points[2] - points[1]
    d2 = points[3] - points[2]
    d3 = points[4] - points[3]
    return 4.0 * (
        s * s * s * d0
        + 3.0 * s * s * t * d1
        + 3.0 * s * t * t * d2
        + t * t * t * d3
    )


def _node_separation(
    velocity: np.ndarray, controller_dt: float, swing_delta_t: float
) -> np.ndarray:
    """Translation between successive Bezier control nodes that yields
    a tip velocity of ``velocity`` at a curve endpoint.

    Mirrors the C++ ``stance_node_seperation`` computation: each node
    sits ``0.25 * v * (dt / delta_t)`` further along the curve than the
    previous one, where ``delta_t`` is the Bezier-parameter step per
    controller tick.
    """
    return 0.25 * velocity * (controller_dt / swing_delta_t)


def generate_primary_swing_control_nodes(
    swing_origin: np.ndarray,
    swing_origin_velocity: np.ndarray,
    target: np.ndarray,
    swing_clearance: float,
    swing_width: float,
    identity_y_sign: int,
    controller_dt: float,
    swing_delta_t: float,
) -> np.ndarray:
    """Primary swing curve (lift-off -> apex).

    Returns the 5 control nodes ``(5, 3)`` for the first half of the
    swing trajectory. ``swing_origin_velocity`` carries the C1 join
    from stance; supply ``-stride_vector / (cycle_time * duty_factor)``
    for the analytical lift-off velocity.

    ``identity_y_sign`` is ``+1`` for left-side legs (whose neutral foot
    sits at positive y in the body frame) and ``-1`` for right-side.
    The lateral arch in the C++ source is shifted in the +/- y direction
    of the body frame; ``swing_width = 0.0`` disables the arch (flat
    tripod default).
    """
    mid = (swing_origin + target) / 2.0
    mid[2] = max(swing_origin[2], target[2]) + swing_clearance
    mid[1] += swing_width if identity_y_sign > 0 else -swing_width

    sep = _node_separation(swing_origin_velocity, controller_dt, swing_delta_t)

    nodes = np.empty((5, 3), dtype=np.float64)
    # C0 at stance->swing join.
    nodes[0] = swing_origin
    # C1 at stance->swing join.
    nodes[1] = swing_origin + sep
    # C2 at stance->swing join.
    nodes[2] = swing_origin + 2.0 * sep
    # C2 at primary->secondary swing join (symmetric apex).
    nodes[3] = (mid + nodes[2]) / 2.0
    nodes[3, 2] = mid[2]
    # Apex.
    nodes[4] = mid
    return nodes


def generate_secondary_swing_control_nodes(
    swing_1_nodes: np.ndarray,
    target: np.ndarray,
    stride_vector: np.ndarray,
    controller_dt: float,
    swing_delta_t: float,
    stance_delta_t: float,
) -> np.ndarray:
    """Secondary swing curve (apex -> touchdown).

    Returns the 5 control nodes ``(5, 3)`` for the second half of
    swing. Joins with C2 to the primary swing curve at the apex, and
    with C2 to the stance curve at touchdown via the analytical
    touchdown velocity ``-stride_vector * (stance_delta_t / dt)``.
    """
    final_velocity = -stride_vector * (stance_delta_t / controller_dt)
    sep = _node_separation(final_velocity, controller_dt, swing_delta_t)

    nodes = np.empty((5, 3), dtype=np.float64)
    nodes[0] = swing_1_nodes[4]
    # C1 at primary->secondary swing join (mirror about the apex).
    nodes[1] = swing_1_nodes[4] - (swing_1_nodes[3] - swing_1_nodes[4])
    # C2 at secondary swing->stance join.
    nodes[2] = target - 2.0 * sep
    # C1 at secondary swing->stance join.
    nodes[3] = target - sep
    # C0 at secondary swing->stance join.
    nodes[4] = target
    return nodes


def generate_stance_control_nodes(
    stance_origin: np.ndarray,
    stride_vector: np.ndarray,
    stride_scaler: float = 1.0,
) -> np.ndarray:
    """Stance curve (touchdown -> next lift-off).

    Returns the 5 control nodes ``(5, 3)``. Nodes are evenly spaced
    along ``-stride_vector * stride_scaler``, so ``dB/dt`` evaluates
    to a (nearly) constant tip velocity opposite the body motion —
    the foot pushes the body forward at a constant ground speed.

    ``stride_scaler`` defaults to 1.0; the C++ source uses values < 1.0
    to extend the first stance after rest. v1 does not need that knob.
    """
    sep = -stride_vector * stride_scaler * 0.25

    nodes = np.empty((5, 3), dtype=np.float64)
    for k in range(5):
        nodes[k] = stance_origin + k * sep
    return nodes
