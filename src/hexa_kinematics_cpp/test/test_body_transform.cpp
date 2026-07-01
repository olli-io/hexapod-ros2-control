// Port of hexa_kinematics/test/test_body_transform.py.
#include <gtest/gtest.h>

#include <cmath>

#include "hexa_kinematics_cpp/body_transform.hpp"
#include "hexa_kinematics_cpp/leg_geometry.hpp"
#include "hexa_kinematics_cpp/leg_ik.hpp"

namespace {

using hexa_kinematics::body_to_leg;
using hexa_kinematics::forward_kinematics;
using hexa_kinematics::inverse_kinematics;
using hexa_kinematics::JointAngles;
using hexa_kinematics::leg_to_body;
using hexa_kinematics::LegSpec;
using hexa_kinematics::Point3;

LegSpec leg(const Point3& mount_xyz = Point3(0.0, 0.0, 0.0),
            double mount_yaw = 0.0) {
  return LegSpec{mount_xyz, mount_yaw, 0.05, 0.08, 0.12};
}

void expectClose(const Point3& a, const Point3& b, double tol = 1e-12) {
  EXPECT_NEAR(a[0], b[0], tol);
  EXPECT_NEAR(a[1], b[1], tol);
  EXPECT_NEAR(a[2], b[2], tol);
}

TEST(BodyTransform, RoundTripRecoversBodyPoint) {
  const LegSpec l = leg(Point3(0.1, 0.05, 0.02), 30.0 * M_PI / 180.0);
  const Point3 p_body(0.2, 0.1, -0.05);
  expectClose(leg_to_body(body_to_leg(p_body, l), l), p_body);
}

TEST(BodyTransform, MountPositionMapsToLegOrigin) {
  const LegSpec l = leg(Point3(0.1, 0.05, 0.02), 45.0 * M_PI / 180.0);
  expectClose(body_to_leg(l.mount_xyz, l), Point3(0.0, 0.0, 0.0));
}

TEST(BodyTransform, YawRotatesXyIntoLegFrame) {
  // Leg mounted at the origin with mount_yaw = 90deg — its +x axis aligns with
  // body +y. So a body point at (0, 1, 0) is at (1, 0, 0) in the leg frame, and
  // (1, 0, 0) is at (0, -1, 0).
  const LegSpec l = leg(Point3(0.0, 0.0, 0.0), M_PI / 2.0);
  expectClose(body_to_leg(Point3(0.0, 1.0, 0.0), l), Point3(1.0, 0.0, 0.0));
  expectClose(body_to_leg(Point3(1.0, 0.0, 0.0), l), Point3(0.0, -1.0, 0.0));
}

TEST(BodyTransform, ZIsUnchangedByYaw) {
  const LegSpec l = leg(Point3(0.0, 0.0, 0.0), 0.5);
  EXPECT_EQ(body_to_leg(Point3(0.0, 0.0, -0.07), l)[2], -0.07);
}

TEST(BodyTransform, BodyTargetRoundTripThroughIkAndFk) {
  // End-to-end stack: body -> leg -> IK -> FK -> leg -> body. Catches sign-flip
  // bugs that the per-module tests can't see in isolation.
  const LegSpec l = leg(Point3(0.10, 0.06, 0.0), -30.0 * M_PI / 180.0);
  const Point3 p_body(0.18, 0.02, -0.08);

  const Point3 p_leg_in = body_to_leg(p_body, l);
  const JointAngles angles = inverse_kinematics(p_leg_in, l);
  const Point3 p_leg_out = forward_kinematics(angles, l);
  const Point3 p_body_out = leg_to_body(p_leg_out, l);

  expectClose(p_leg_out, p_leg_in, 1e-9);
  expectClose(p_body_out, p_body, 1e-9);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
