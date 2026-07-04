// Host unit tests for the float Vec3 (plan part 04).
//
// Hand-computed expectations for the ops the forked kinematics / gait / posture
// code relies on. Runs natively (no Pico SDK): the whole point of the host
// harness is to exercise the pure float math off-target.

#include <cmath>

#include <gtest/gtest.h>

#include "vec3.hpp"

using hexa::Vec3;

TEST(Vec3, ConstructAndAccess) {
  Vec3 v(1.0f, 2.0f, 3.0f);
  EXPECT_FLOAT_EQ(v.x, 1.0f);
  EXPECT_FLOAT_EQ(v.y, 2.0f);
  EXPECT_FLOAT_EQ(v.z, 3.0f);
  // Eigen-style indexed access used throughout the ported code.
  EXPECT_FLOAT_EQ(v[0], 1.0f);
  EXPECT_FLOAT_EQ(v[1], 2.0f);
  EXPECT_FLOAT_EQ(v[2], 3.0f);
  v[1] = 9.0f;
  EXPECT_FLOAT_EQ(v.y, 9.0f);
}

TEST(Vec3, ZeroDefault) {
  EXPECT_EQ(Vec3::Zero(), Vec3(0.0f, 0.0f, 0.0f));
  EXPECT_EQ(Vec3(), Vec3::Zero());
}

TEST(Vec3, AddSubNegate) {
  Vec3 a(1.0f, 2.0f, 3.0f);
  Vec3 b(4.0f, -1.0f, 0.5f);
  EXPECT_EQ(a + b, Vec3(5.0f, 1.0f, 3.5f));
  EXPECT_EQ(a - b, Vec3(-3.0f, 3.0f, 2.5f));
  EXPECT_EQ(-a, Vec3(-1.0f, -2.0f, -3.0f));
  Vec3 c = a;
  c += b;
  EXPECT_EQ(c, a + b);
  c -= b;
  EXPECT_EQ(c, a);
}

TEST(Vec3, ScalarMulDiv) {
  Vec3 a(2.0f, -4.0f, 6.0f);
  EXPECT_EQ(a * 0.5f, Vec3(1.0f, -2.0f, 3.0f));
  EXPECT_EQ(0.5f * a, Vec3(1.0f, -2.0f, 3.0f));  // scalar on the left
  EXPECT_EQ(a / 2.0f, Vec3(1.0f, -2.0f, 3.0f));
  Vec3 c = a;
  c *= 2.0f;
  EXPECT_EQ(c, Vec3(4.0f, -8.0f, 12.0f));
  c /= 4.0f;
  EXPECT_EQ(c, Vec3(1.0f, -2.0f, 3.0f));
}

TEST(Vec3, Dot) {
  Vec3 a(1.0f, 2.0f, 3.0f);
  Vec3 b(4.0f, -5.0f, 6.0f);
  // 1*4 + 2*(-5) + 3*6 = 4 - 10 + 18 = 12
  EXPECT_FLOAT_EQ(a.dot(b), 12.0f);
  EXPECT_FLOAT_EQ(hexa::dot(a, b), 12.0f);
}

TEST(Vec3, Cross) {
  Vec3 x(1.0f, 0.0f, 0.0f);
  Vec3 y(0.0f, 1.0f, 0.0f);
  EXPECT_EQ(x.cross(y), Vec3(0.0f, 0.0f, 1.0f));   // x cross y = z
  EXPECT_EQ(y.cross(x), Vec3(0.0f, 0.0f, -1.0f));  // anti-commutative
  Vec3 a(2.0f, 3.0f, 4.0f);
  Vec3 b(5.0f, 6.0f, 7.0f);
  // (3*7-4*6, 4*5-2*7, 2*6-3*5) = (-3, 6, -3)
  EXPECT_EQ(hexa::cross(a, b), Vec3(-3.0f, 6.0f, -3.0f));
}

TEST(Vec3, NormAndSquaredNorm) {
  Vec3 v(3.0f, 4.0f, 0.0f);
  EXPECT_FLOAT_EQ(v.squaredNorm(), 25.0f);
  EXPECT_FLOAT_EQ(v.norm(), 5.0f);  // hypotf-based
  Vec3 w(1.0f, 2.0f, 2.0f);
  EXPECT_FLOAT_EQ(w.norm(), 3.0f);  // sqrt(1+4+4) = 3
  EXPECT_FLOAT_EQ(hexa::norm(w), 3.0f);
}

TEST(Vec3, Normalized) {
  Vec3 v(0.0f, 0.0f, 5.0f);
  EXPECT_EQ(v.normalized(), Vec3(0.0f, 0.0f, 1.0f));
  // Degenerate: zero vector normalizes to zero (no NaN).
  EXPECT_EQ(Vec3::Zero().normalized(), Vec3::Zero());
}

TEST(Vec3, ConstexprUsable) {
  constexpr Vec3 a(1.0f, 2.0f, 3.0f);
  constexpr Vec3 b = a * 2.0f + Vec3(0.0f, 0.0f, 1.0f);
  static_assert(b.x == 2.0f && b.y == 4.0f && b.z == 7.0f);
  static_assert(a.dot(a) == 14.0f);
  SUCCEED();
}
