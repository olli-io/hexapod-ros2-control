import numpy as np
import pytest

from hexa_gait.trajectory import (
    generate_primary_swing_control_nodes,
    generate_secondary_swing_control_nodes,
    generate_stance_control_nodes,
    quartic_bezier,
    quartic_bezier_dot,
)


def _nodes(p0, p1, p2, p3, p4):
    return np.array([p0, p1, p2, p3, p4], dtype=np.float64)


def test_quartic_bezier_endpoints():
    nodes = _nodes([0, 0, 0], [1, 0, 0], [2, 1, 0], [3, 2, 0], [4, 3, 0])
    np.testing.assert_allclose(quartic_bezier(nodes, 0.0), nodes[0])
    np.testing.assert_allclose(quartic_bezier(nodes, 1.0), nodes[4])


def test_quartic_bezier_evenly_spaced_is_linear():
    # Evenly spaced control points on the line P0 + k*d collapse the
    # quartic Bezier to a linear interpolation P0 + 4*t*d (see the
    # derivation in the trajectory module docstring).
    d = np.array([0.1, -0.05, 0.0])
    p0 = np.array([1.0, 2.0, 3.0])
    nodes = np.stack([p0 + k * d for k in range(5)])
    for t in (0.0, 0.25, 0.5, 0.75, 1.0):
        np.testing.assert_allclose(
            quartic_bezier(nodes, t),
            p0 + 4.0 * t * d,
            atol=1e-12,
        )


def test_quartic_bezier_shape_validation():
    with pytest.raises(ValueError):
        quartic_bezier(np.zeros((4, 3)), 0.0)
    with pytest.raises(ValueError):
        quartic_bezier_dot(np.zeros((4, 3)), 0.0)


def test_stance_curve_has_constant_velocity():
    # Evenly spaced stance nodes: B(t) is linear in t along -stride.
    stride = np.array([0.1, 0.0, 0.0])
    origin = np.array([0.5, 0.0, -0.1])
    nodes = generate_stance_control_nodes(stance_origin=origin, stride_vector=stride)
    # Bezier should reach -stride displacement at t=1.
    np.testing.assert_allclose(quartic_bezier(nodes, 0.0), origin)
    np.testing.assert_allclose(quartic_bezier(nodes, 1.0), origin - stride)
    # And dB/dt is constant magnitude (the stance velocity contract).
    v0 = quartic_bezier_dot(nodes, 0.0)
    v1 = quartic_bezier_dot(nodes, 0.5)
    v2 = quartic_bezier_dot(nodes, 1.0)
    np.testing.assert_allclose(v0, v1)
    np.testing.assert_allclose(v1, v2)


SWING_TIME = 0.6
STANCE_TIME = 0.6  # symmetric β = 0.5 chain.


def _swing_chain(stride):
    swing_origin = np.array([0.5, 0.0, -0.1])
    target = swing_origin + stride
    dt = 0.02
    swing_delta_t = dt / SWING_TIME
    stance_delta_t = dt / SWING_TIME
    primary = generate_primary_swing_control_nodes(
        swing_origin=swing_origin,
        swing_origin_velocity=-stride / SWING_TIME,
        target=target,
        swing_clearance=0.03,
        swing_width=0.0,
        identity_y_sign=1,
        controller_dt=dt,
        swing_delta_t=swing_delta_t,
    )
    secondary = generate_secondary_swing_control_nodes(
        swing_1_nodes=primary,
        target=target,
        stride_vector=stride,
        controller_dt=dt,
        swing_delta_t=swing_delta_t,
        stance_delta_t=stance_delta_t,
    )
    stance = generate_stance_control_nodes(
        stance_origin=target, stride_vector=stride
    )
    return swing_origin, target, primary, secondary, stance


def test_swing_curves_join_with_c0_continuity():
    stride = np.array([0.1, 0.0, 0.0])
    swing_origin, target, primary, secondary, stance = _swing_chain(stride)
    # Primary curve endpoints.
    np.testing.assert_allclose(quartic_bezier(primary, 0.0), swing_origin)
    # Primary -> secondary join (the apex).
    np.testing.assert_allclose(quartic_bezier(primary, 1.0), primary[4])
    np.testing.assert_allclose(quartic_bezier(secondary, 0.0), primary[4])
    # Secondary -> stance join (touchdown).
    np.testing.assert_allclose(quartic_bezier(secondary, 1.0), target)
    np.testing.assert_allclose(quartic_bezier(stance, 0.0), target)
    # Stance -> next primary join (lift-off again).
    np.testing.assert_allclose(quartic_bezier(stance, 1.0), target - stride)


def test_swing_curves_join_with_c1_continuity():
    # C1 means equal foot velocity *in real time* at each join. The
    # primary and secondary swing Beziers each cover swing_time / 2, the
    # stance Bezier covers stance_time. Parameter-space derivatives are
    # therefore comparable directly only at the primary -> secondary
    # apex (matching half-durations); the secondary -> stance join must
    # be checked in time space.
    stride = np.array([0.1, 0.0, 0.0])
    _, _, primary, secondary, stance = _swing_chain(stride)

    # Primary -> secondary apex (both cover swing_time / 2).
    np.testing.assert_allclose(
        quartic_bezier_dot(primary, 1.0),
        quartic_bezier_dot(secondary, 0.0),
        atol=1e-12,
    )

    # Secondary -> stance touchdown: convert each curve's dB/ds to dB/dt.
    dBdt_secondary = quartic_bezier_dot(secondary, 1.0) * (2.0 / SWING_TIME)
    dBdt_stance = quartic_bezier_dot(stance, 0.0) * (1.0 / STANCE_TIME)
    np.testing.assert_allclose(dBdt_secondary, dBdt_stance, atol=1e-12)


def test_swing_touchdown_velocity_matches_steady_state_stance():
    # Regression for the 2× node-separation bug: the swing's secondary
    # curve must touch down at exactly the steady-state stance velocity
    # (-stride / stance_time), not twice that.
    stride = np.array([0.1, 0.0, 0.0])
    _, _, _, secondary, _ = _swing_chain(stride)

    dBdt_secondary_touchdown = quartic_bezier_dot(secondary, 1.0) * (2.0 / SWING_TIME)
    expected = -stride / STANCE_TIME
    np.testing.assert_allclose(dBdt_secondary_touchdown, expected, atol=1e-12)


def test_primary_swing_liftoff_velocity_matches_requested():
    # Regression for the 2× node-separation bug on the lift-off side:
    # dB/dt at the start of the primary swing must equal the velocity
    # passed in (= -stride / swing_time for a steady-state join), not
    # twice that.
    stride = np.array([0.1, 0.0, 0.0])
    _, _, primary, _, _ = _swing_chain(stride)

    dBdt_primary_liftoff = quartic_bezier_dot(primary, 0.0) * (2.0 / SWING_TIME)
    expected = -stride / SWING_TIME
    np.testing.assert_allclose(dBdt_primary_liftoff, expected, atol=1e-12)


def test_swing_apex_clears_origin_by_swing_clearance():
    stride = np.array([0.1, 0.0, 0.0])
    swing_origin, target, primary, secondary, _ = _swing_chain(stride)
    apex = quartic_bezier(primary, 1.0)
    expected_z = max(swing_origin[2], target[2]) + 0.03
    assert apex[2] == pytest.approx(expected_z)
