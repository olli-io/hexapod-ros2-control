// gtest port of hexa_gait/test/test_trajectory.py — exercises the pure C++
// quartic-Bezier foot-tip trajectory builders (no ROS). Behavioural parity with
// the Python suite.
//
// Dropped: test_quartic_bezier_shape_validation. It asserts a ValueError on a
// wrong-shaped numpy array passed to quartic_bezier / quartic_bezier_dot. In C++
// the node set is a std::array<Vec3, 5> (BezierNodes), so a wrong size is a
// compile error rather than a runtime check — there is nothing to test.

#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "hexa_gait_cpp/trajectory.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {
namespace {

// Mirror of the Python _nodes(p0, p1, p2, p3, p4) helper: aggregate-init the
// BezierNodes from five explicit control points.
BezierNodes make_nodes(const Vec3& p0, const Vec3& p1, const Vec3& p2,
                       const Vec3& p3, const Vec3& p4) {
  return BezierNodes{p0, p1, p2, p3, p4};
}

// Per-component EXPECT_NEAR — the C++ analogue of np.testing.assert_allclose.
void expect_vec_near(const Vec3& a, const Vec3& b, double tol = 1e-9) {
  EXPECT_NEAR(a[0], b[0], tol);
  EXPECT_NEAR(a[1], b[1], tol);
  EXPECT_NEAR(a[2], b[2], tol);
}

TEST(Trajectory, QuarticBezierEndpoints) {
  const BezierNodes nodes =
      make_nodes(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(2, 1, 0), Vec3(3, 2, 0),
                 Vec3(4, 3, 0));
  expect_vec_near(quartic_bezier(nodes, 0.0), nodes[0]);
  expect_vec_near(quartic_bezier(nodes, 1.0), nodes[4]);
}

TEST(Trajectory, QuarticBezierEvenlySpacedIsLinear) {
  // Evenly spaced control points on the line P0 + k*d collapse the
  // quartic Bezier to a linear interpolation P0 + 4*t*d (see the
  // derivation in the trajectory module docstring).
  const Vec3 d(0.1, -0.05, 0.0);
  const Vec3 p0(1.0, 2.0, 3.0);
  const BezierNodes nodes = make_nodes(p0, p0 + 1.0 * d, p0 + 2.0 * d,
                                       p0 + 3.0 * d, p0 + 4.0 * d);
  for (double t : {0.0, 0.25, 0.5, 0.75, 1.0}) {
    expect_vec_near(quartic_bezier(nodes, t), p0 + 4.0 * t * d, 1e-12);
  }
}

// test_quartic_bezier_shape_validation dropped — see file header comment.

TEST(Trajectory, StanceCurveHasConstantVelocity) {
  // Evenly spaced stance nodes: B(t) is linear in t along -stride.
  const Vec3 stride(0.1, 0.0, 0.0);
  const Vec3 origin(0.5, 0.0, -0.1);
  const BezierNodes nodes = generate_stance_control_nodes(origin, stride);
  // Bezier should reach -stride displacement at t=1.
  expect_vec_near(quartic_bezier(nodes, 0.0), origin);
  expect_vec_near(quartic_bezier(nodes, 1.0), origin - stride);
  // And dB/dt is constant magnitude (the stance velocity contract).
  const Vec3 v0 = quartic_bezier_dot(nodes, 0.0);
  const Vec3 v1 = quartic_bezier_dot(nodes, 0.5);
  const Vec3 v2 = quartic_bezier_dot(nodes, 1.0);
  expect_vec_near(v0, v1);
  expect_vec_near(v1, v2);
}

constexpr double SWING_TIME = 0.6;
constexpr double STANCE_TIME = 0.6;  // symmetric β = 0.5 chain.

// Port of _swing_chain(stride): builds the primary / secondary swing and stance
// Bezier node sets for a single steady-state step. Positional args map 1:1 to
// the Python keyword args.
struct SwingChain {
  Vec3 swing_origin;
  Vec3 target;
  BezierNodes primary;
  BezierNodes secondary;
  BezierNodes stance;
};

SwingChain swing_chain(const Vec3& stride) {
  const Vec3 swing_origin(0.5, 0.0, -0.1);
  const Vec3 target = swing_origin + stride;
  const double dt = 0.02;
  const double swing_delta_t = dt / SWING_TIME;
  const double stance_delta_t = dt / SWING_TIME;
  const BezierNodes primary = generate_primary_swing_control_nodes(
      swing_origin, -stride / SWING_TIME, target, /*swing_clearance=*/0.03,
      /*swing_width=*/0.0, /*identity_y_sign=*/1, dt, swing_delta_t);
  const BezierNodes secondary = generate_secondary_swing_control_nodes(
      primary, target, stride, dt, swing_delta_t, stance_delta_t);
  const BezierNodes stance = generate_stance_control_nodes(target, stride);
  return {swing_origin, target, primary, secondary, stance};
}

TEST(Trajectory, SwingCurvesJoinWithC0Continuity) {
  const Vec3 stride(0.1, 0.0, 0.0);
  const SwingChain c = swing_chain(stride);
  // Primary curve endpoints.
  expect_vec_near(quartic_bezier(c.primary, 0.0), c.swing_origin);
  // Primary -> secondary join (the apex).
  expect_vec_near(quartic_bezier(c.primary, 1.0), c.primary[4]);
  expect_vec_near(quartic_bezier(c.secondary, 0.0), c.primary[4]);
  // Secondary -> stance join (touchdown).
  expect_vec_near(quartic_bezier(c.secondary, 1.0), c.target);
  expect_vec_near(quartic_bezier(c.stance, 0.0), c.target);
  // Stance -> next primary join (lift-off again).
  expect_vec_near(quartic_bezier(c.stance, 1.0), c.target - stride);
}

TEST(Trajectory, SwingCurvesJoinWithC1Continuity) {
  // C1 means equal foot velocity *in real time* at each join. The
  // primary and secondary swing Beziers each cover swing_time / 2, the
  // stance Bezier covers stance_time. Parameter-space derivatives are
  // therefore comparable directly only at the primary -> secondary
  // apex (matching half-durations); the secondary -> stance join must
  // be checked in time space.
  const Vec3 stride(0.1, 0.0, 0.0);
  const SwingChain c = swing_chain(stride);

  // Primary -> secondary apex (both cover swing_time / 2).
  expect_vec_near(quartic_bezier_dot(c.primary, 1.0),
                  quartic_bezier_dot(c.secondary, 0.0), 1e-12);

  // Secondary -> stance touchdown: convert each curve's dB/ds to dB/dt.
  const Vec3 dBdt_secondary =
      quartic_bezier_dot(c.secondary, 1.0) * (2.0 / SWING_TIME);
  const Vec3 dBdt_stance =
      quartic_bezier_dot(c.stance, 0.0) * (1.0 / STANCE_TIME);
  expect_vec_near(dBdt_secondary, dBdt_stance, 1e-12);
}

TEST(Trajectory, SwingTouchdownVelocityMatchesSteadyStateStance) {
  // Regression for the 2× node-separation bug: the swing's secondary
  // curve must touch down at exactly the steady-state stance velocity
  // (-stride / stance_time), not twice that.
  const Vec3 stride(0.1, 0.0, 0.0);
  const SwingChain c = swing_chain(stride);

  const Vec3 dBdt_secondary_touchdown =
      quartic_bezier_dot(c.secondary, 1.0) * (2.0 / SWING_TIME);
  const Vec3 expected = -stride / STANCE_TIME;
  expect_vec_near(dBdt_secondary_touchdown, expected, 1e-12);
}

TEST(Trajectory, PrimarySwingLiftoffVelocityMatchesRequested) {
  // Regression for the 2× node-separation bug on the lift-off side:
  // dB/dt at the start of the primary swing must equal the velocity
  // passed in (= -stride / swing_time for a steady-state join), not
  // twice that.
  const Vec3 stride(0.1, 0.0, 0.0);
  const SwingChain c = swing_chain(stride);

  const Vec3 dBdt_primary_liftoff =
      quartic_bezier_dot(c.primary, 0.0) * (2.0 / SWING_TIME);
  const Vec3 expected = -stride / SWING_TIME;
  expect_vec_near(dBdt_primary_liftoff, expected, 1e-12);
}

TEST(Trajectory, SwingApexClearsOriginBySwingClearance) {
  const Vec3 stride(0.1, 0.0, 0.0);
  const SwingChain c = swing_chain(stride);
  const Vec3 apex = quartic_bezier(c.primary, 1.0);
  const double expected_z = std::max(c.swing_origin[2], c.target[2]) + 0.03;
  EXPECT_NEAR(apex[2], expected_z, 1e-9);
}

}  // namespace
}  // namespace hexa_gait

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
