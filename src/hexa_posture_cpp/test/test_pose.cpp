// Port of hexa_posture/test/test_pose.py.
#include <gtest/gtest.h>

#include "hexa_posture_cpp/pose.hpp"

namespace {

using hexa_posture::add;
using hexa_posture::BodyPose;
using hexa_posture::clamp;
using hexa_posture::compose_layered;
using hexa_posture::IDENTITY;
using hexa_posture::lerp;
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

TEST(Pose, LerpEndpointsAndMidpoint) {
  const BodyPose a{0.01, -0.02, 0.005, 0.05, -0.05, 0.1};
  const BodyPose b{0.04, -0.01, 0.03, 0.2, -0.1, 0.3};
  EXPECT_EQ(lerp(a, b, 0.0), a);  // t=0 returns a exactly
  const BodyPose end = lerp(a, b, 1.0);
  EXPECT_NEAR(end.x, b.x, 1e-12);
  EXPECT_NEAR(end.yaw, b.yaw, 1e-12);
  const BodyPose mid = lerp(a, b, 0.5);
  EXPECT_NEAR(mid.x, 0.025, 1e-12);
  EXPECT_NEAR(mid.y, -0.015, 1e-12);
  EXPECT_NEAR(mid.z, 0.0175, 1e-12);
  EXPECT_NEAR(mid.roll, 0.125, 1e-12);
  EXPECT_NEAR(mid.pitch, -0.075, 1e-12);
  EXPECT_NEAR(mid.yaw, 0.2, 1e-12);
}

// A dialed-in user posture beyond the reserved headroom must NOT clip the
// animation asymmetrically — the pre-fix clamp(add(user, anim)) failure case.
TEST(Pose, ComposeLayeredKeepsAnimationSymmetric) {
  const PoseLimits limits;  // x = 0.05
  PoseLimits reserve;       // default; only x matters here
  reserve.x = 0.02;
  const BodyPose user{0.045, 0.0, 0.0, 0.0, 0.0, 0.0};  // beyond user_env (0.03)
  const BodyPose anim_pos{0.02, 0.0, 0.0, 0.0, 0.0, 0.0};
  const BodyPose anim_neg{-0.02, 0.0, 0.0, 0.0, 0.0, 0.0};
  const double xp = compose_layered(user, anim_pos, limits, reserve).x;
  const double xn = compose_layered(user, anim_neg, limits, reserve).x;
  // Baseline is the user clamped to user_env (0.03); the animation swings a full
  // symmetric +/-0.02 about it and stays inside the 0.05 envelope.
  EXPECT_NEAR(xp, 0.05, 1e-12);
  EXPECT_NEAR(xn, 0.01, 1e-12);
  const double baseline = 0.5 * (xp + xn);
  EXPECT_NEAR(xp - baseline, baseline - xn, 1e-12);  // symmetric swing
  EXPECT_LE(xp, limits.x + 1e-12);
}

// A reserve larger than the axis limit floors the user envelope to zero — the
// animation gets the whole budget, the static user pose is dropped on that axis.
TEST(Pose, ComposeLayeredReserveExceedingLimitFloorsUserEnvelopeToZero) {
  const PoseLimits limits;  // x = 0.05
  PoseLimits reserve;
  reserve.x = 0.08;  // > limit
  const BodyPose user{0.04, 0.0, 0.0, 0.0, 0.0, 0.0};
  const BodyPose anim{0.01, 0.0, 0.0, 0.0, 0.0, 0.0};
  const BodyPose out = compose_layered(user, anim, limits, reserve);
  EXPECT_NEAR(out.x, 0.01, 1e-12);  // user contributes 0; only the animation
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
