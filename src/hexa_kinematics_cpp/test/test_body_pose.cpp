// Port of hexa_kinematics/test/test_body_pose.py.
#include <gtest/gtest.h>

#include <cmath>

#include "hexa_kinematics_cpp/body_transform.hpp"

namespace {

using hexa_kinematics::apply_body_pose;
using hexa_kinematics::BodyPose;
using hexa_kinematics::IDENTITY_BODY_POSE;
using hexa_kinematics::Point3;

void expectClose(const Point3& a, const Point3& b, double tol = 1e-12) {
  EXPECT_NEAR(a[0], b[0], tol);
  EXPECT_NEAR(a[1], b[1], tol);
  EXPECT_NEAR(a[2], b[2], tol);
}

TEST(BodyPose, IdentityPoseIsPassthrough) {
  expectClose(apply_body_pose(Point3(0.1, -0.2, 0.05), IDENTITY_BODY_POSE),
              Point3(0.1, -0.2, 0.05));
}

TEST(BodyPose, TranslationSubtractsFromTarget) {
  BodyPose pose;
  pose.x = 0.05;
  expectClose(apply_body_pose(Point3(0.20, 0.0, -0.10), pose),
              Point3(0.15, 0.0, -0.10));
}

TEST(BodyPose, YawRotatesXyOppositely) {
  BodyPose pose;
  pose.yaw = M_PI / 2.0;
  expectClose(apply_body_pose(Point3(1.0, 0.0, 0.0), pose),
              Point3(0.0, -1.0, 0.0));
}

TEST(BodyPose, PitchRotatesXzOppositely) {
  BodyPose pose;
  pose.pitch = M_PI / 2.0;
  expectClose(apply_body_pose(Point3(1.0, 0.0, 0.0), pose),
              Point3(0.0, 0.0, 1.0));
}

TEST(BodyPose, RollRotatesYzOppositely) {
  BodyPose pose;
  pose.roll = M_PI / 2.0;
  expectClose(apply_body_pose(Point3(0.0, 1.0, 0.0), pose),
              Point3(0.0, 0.0, -1.0));
}

TEST(BodyPose, PureTranslationPreservesRelativeGeometry) {
  BodyPose pose;
  pose.x = 0.03;
  pose.y = -0.01;
  pose.z = 0.02;
  const Point3 a = apply_body_pose(Point3(0.10, 0.05, -0.08), pose);
  const Point3 b = apply_body_pose(Point3(0.20, 0.05, -0.08), pose);
  expectClose(Point3(b[0] - a[0], b[1] - a[1], b[2] - a[2]),
              Point3(0.10, 0.0, 0.0));
}

TEST(BodyPose, RoundTripComposeWithInversePoseRecoversTarget) {
  // Applying pose P then pose (-P) recovers the original target only when
  // rotations are small enough that intrinsic XYZ commutes to first order. Use
  // a tiny pose to make the round-trip exact-ish.
  BodyPose pose;
  pose.x = 0.01;
  pose.y = -0.02;
  pose.z = 0.005;
  pose.roll = 0.02;
  pose.pitch = -0.01;
  pose.yaw = 0.03;
  BodyPose inv;
  inv.x = -pose.x;
  inv.y = -pose.y;
  inv.z = -pose.z;
  inv.roll = -pose.roll;
  inv.pitch = -pose.pitch;
  inv.yaw = -pose.yaw;
  const Point3 p(0.18, 0.04, -0.09);
  expectClose(apply_body_pose(apply_body_pose(p, pose), inv), p, 1e-3);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
