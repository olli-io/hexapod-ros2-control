// Port of hexa_kinematics/test/test_leg_ik.py.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "hexa_kinematics_cpp/leg_geometry.hpp"
#include "hexa_kinematics_cpp/leg_ik.hpp"

namespace {

using hexa_kinematics::JointAngles;
using hexa_kinematics::LegSpec;
using hexa_kinematics::Point3;

const LegSpec kLeg{Point3(0.0, 0.0, 0.0), 0.0, 0.05, 0.08, 0.12};

void expectClosePoint(const Point3& a, const Point3& b, double tol = 1e-9) {
  EXPECT_NEAR(a[0], b[0], tol);
  EXPECT_NEAR(a[1], b[1], tol);
  EXPECT_NEAR(a[2], b[2], tol);
}

void expectCloseAngles(const JointAngles& a, const JointAngles& b,
                       double tol = 1e-9) {
  EXPECT_NEAR(a[0], b[0], tol);
  EXPECT_NEAR(a[1], b[1], tol);
  EXPECT_NEAR(a[2], b[2], tol);
}

TEST(LegIk, FkZeroAnglesExtendsLegAlongX) {
  const Point3 foot = hexa_kinematics::forward_kinematics({0.0, 0.0, 0.0}, kLeg);
  expectClosePoint(
      foot, Point3(kLeg.coxa_len + kLeg.femur_len + kLeg.tibia_len, 0.0, 0.0));
}

TEST(LegIk, IkExtendedPoseReturnsZeroAngles) {
  const Point3 target(kLeg.coxa_len + kLeg.femur_len + kLeg.tibia_len, 0.0, 0.0);
  expectCloseAngles(hexa_kinematics::inverse_kinematics(target, kLeg),
                    {0.0, 0.0, 0.0});
}

TEST(LegIk, IkFootStraightDownFromFemurJoint) {
  const Point3 target(kLeg.coxa_len, 0.0,
                      -(kLeg.femur_len + kLeg.tibia_len));
  expectCloseAngles(hexa_kinematics::inverse_kinematics(target, kLeg),
                    {0.0, M_PI / 2.0, 0.0});
}

TEST(LegIk, IkCoxaYawToTheSide) {
  const Point3 target(0.0, kLeg.coxa_len + kLeg.femur_len + kLeg.tibia_len, 0.0);
  expectCloseAngles(hexa_kinematics::inverse_kinematics(target, kLeg),
                    {M_PI / 2.0, 0.0, 0.0});
}

TEST(LegIk, FkIkRoundTrip) {
  const std::vector<JointAngles> cases = {
      {0.0, 0.0, 0.0},
      {0.0, 0.3, 0.4},
      {0.5, 0.2, 0.6},
      {-0.5, 0.4, 0.5},
      {1.2, -0.3, 1.0},
      {-1.0, 0.7, 0.3},
      {0.0, M_PI / 4.0, M_PI / 3.0},
  };
  for (const auto& angles : cases) {
    const Point3 foot = hexa_kinematics::forward_kinematics(angles, kLeg);
    const JointAngles recovered =
        hexa_kinematics::inverse_kinematics(foot, kLeg);
    const Point3 foot_again =
        hexa_kinematics::forward_kinematics(recovered, kLeg);
    expectClosePoint(foot, foot_again, 1e-9);
  }
}

TEST(LegIk, IkRaisesOnTargetBeyondReach) {
  EXPECT_THROW(hexa_kinematics::inverse_kinematics(Point3(10.0, 0.0, 0.0), kLeg),
               hexa_kinematics::UnreachableTarget);
}

TEST(LegIk, IkRaisesOnTargetInsideInnerAnnulus) {
  // Foot at the femur joint -> d = 0, below |femur - tibia|.
  EXPECT_THROW(
      hexa_kinematics::inverse_kinematics(Point3(kLeg.coxa_len, 0.0, 0.0), kLeg),
      hexa_kinematics::UnreachableTarget);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
