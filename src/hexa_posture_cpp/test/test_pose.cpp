// Port of hexa_posture/test/test_pose.py.
#include <gtest/gtest.h>

#include "hexa_posture_cpp/pose.hpp"

namespace {

using hexa_posture::add;
using hexa_posture::BodyPose;
using hexa_posture::clamp;
using hexa_posture::IDENTITY;
using hexa_posture::PoseLimits;
using hexa_posture::scale;

TEST(Pose, IdentityIsAllZeros) { EXPECT_EQ(IDENTITY, BodyPose{}); }

TEST(Pose, AddIsComponentWise) {
  const BodyPose a{0.01, 0.02, 0.03, 0.1, 0.2, 0.3};
  const BodyPose b{-0.005, 0.0, 0.01, -0.05, 0.0, 0.1};
  const BodyPose s = add(a, b);
  EXPECT_NEAR(s.x, 0.005, 1e-12);
  EXPECT_NEAR(s.y, 0.02, 1e-12);
  EXPECT_NEAR(s.z, 0.04, 1e-12);
  EXPECT_NEAR(s.roll, 0.05, 1e-12);
  EXPECT_NEAR(s.pitch, 0.2, 1e-12);
  EXPECT_NEAR(s.yaw, 0.4, 1e-12);
}

TEST(Pose, ScaleIsUniform) {
  const BodyPose p{0.04, -0.02, 0.01, 0.1, -0.1, 0.2};
  const BodyPose s = scale(p, 0.5);
  EXPECT_NEAR(s.x, 0.02, 1e-12);
  EXPECT_NEAR(s.y, -0.01, 1e-12);
  EXPECT_NEAR(s.z, 0.005, 1e-12);
  EXPECT_NEAR(s.roll, 0.05, 1e-12);
  EXPECT_NEAR(s.pitch, -0.05, 1e-12);
  EXPECT_NEAR(s.yaw, 0.1, 1e-12);
}

TEST(Pose, ClampCapsEachAxisSymmetrically) {
  const PoseLimits limits{0.05, 0.05, 0.04, 0.3, 0.3, 0.5};
  const BodyPose runaway{10.0, -10.0, 1.0, 5.0, -5.0, 2.0};
  const BodyPose c = clamp(runaway, limits);
  EXPECT_EQ(c, (BodyPose{0.05, -0.05, 0.04, 0.3, -0.3, 0.5}));
}

TEST(Pose, ClampPassesThroughInEnvelopeValues) {
  const PoseLimits limits;
  const BodyPose inside{0.01, -0.02, 0.005, 0.05, -0.05, 0.1};
  EXPECT_EQ(clamp(inside, limits), inside);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
