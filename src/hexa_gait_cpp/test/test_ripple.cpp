// gtest port of hexa_gait/test/test_ripple.py — exercises the pure Ripple gait
// strategy (no ROS, no engine). Behavioural parity with the Python suite.
//
// The Ripple class is not header-visible, so every strategy is built through the
// registry factory (strategies().at("ripple")() / ("crawl")). Python class
// attributes (Ripple.duty_factor, Ripple.phase_offsets) map to the instance
// methods duty_factor() / phase_offsets(); the foot_target signature is the
// same (phase, stride, leg).

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/gaits/registry.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {
namespace {

// Port of _leg(): a single leg with a non-trivial mount and nominal stance.
LegContext make_leg(Vec3 nominal = Vec3(0.2, 0.1, -0.1)) {
  LegContext leg;
  leg.name = "l_front";
  leg.mount_xyz = Vec3(0.083, 0.0575, 0.0);
  leg.mount_yaw = 30.0 * M_PI / 180.0;
  leg.nominal_stance = nominal;
  return leg;
}

// Port of _stride(). duty_factor defaults to ripple's 5/6.
StrideParams make_stride(Vec3 stride = Vec3(0.0, 0.0, 0.0),
                         double duty_factor = 5.0 / 6.0) {
  StrideParams s;
  s.stride_vector = stride;
  s.cycle_time = 2.0;
  s.duty_factor = duty_factor;
  s.swing_clearance = 0.03;
  s.swing_width = 0.0;
  s.controller_dt = 0.02;
  return s;
}

TEST(Ripple, DutyFactorFiveSixths) {
  auto ripple = strategies().at("ripple")();
  EXPECT_NEAR(ripple->duty_factor(), 5.0 / 6.0, 1e-12);
}

TEST(Ripple, SharesMetachronalOffsetsWithCrawl) {
  // Python: `Ripple.phase_offsets is METACHRONAL_OFFSETS`. METACHRONAL_OFFSETS
  // is not exported; both crawl and ripple return a reference to the same
  // internal static table, so the identity check becomes pointer identity of
  // the referenced PhaseOffsets objects.
  auto ripple = strategies().at("ripple")();
  auto crawl = strategies().at("crawl")();
  EXPECT_EQ(&ripple->phase_offsets(), &crawl->phase_offsets());
}

TEST(Ripple, ZeroStrideHoldsNominalXyAtAllPhases) {
  const LegContext leg = make_leg();
  const StrideParams stride = make_stride();
  auto ripple = strategies().at("ripple")();
  for (double phase : {0.0, 0.05, 0.1, 0.2, 0.5, 0.9, 0.99}) {
    const Vec3 target = ripple->foot_target(phase, stride, leg);
    EXPECT_NEAR(target[0], leg.nominal_stance[0], 1e-9);
    EXPECT_NEAR(target[1], leg.nominal_stance[1], 1e-9);
  }
}

TEST(Ripple, PhaseZeroEmitsPep) {
  const LegContext leg = make_leg();
  const Vec3 stride_vec(0.18, 0.0, 0.0);
  const StrideParams stride = make_stride(stride_vec);
  auto ripple = strategies().at("ripple")();
  const Vec3 pep(leg.nominal_stance[0] - 0.5 * stride_vec[0],
                 leg.nominal_stance[1] - 0.5 * stride_vec[1],
                 leg.nominal_stance[2] - 0.5 * stride_vec[2]);
  const Vec3 target = ripple->foot_target(0.0, stride, leg);
  EXPECT_NEAR(target[0], pep[0], 1e-9);
  EXPECT_NEAR(target[1], pep[1], 1e-9);
  EXPECT_NEAR(target[2], pep[2], 1e-9);
}

TEST(Ripple, TouchdownPhaseEmitsAep) {
  const LegContext leg = make_leg();
  const Vec3 stride_vec(0.18, 0.0, 0.0);
  const StrideParams stride = make_stride(stride_vec);
  auto ripple = strategies().at("ripple")();
  const Vec3 aep(leg.nominal_stance[0] + 0.5 * stride_vec[0],
                 leg.nominal_stance[1] + 0.5 * stride_vec[1],
                 leg.nominal_stance[2] + 0.5 * stride_vec[2]);
  const double swing_end = 1.0 - ripple->duty_factor();  // 1/6
  const Vec3 target = ripple->foot_target(swing_end, stride, leg);
  EXPECT_NEAR(target[0], aep[0], 1e-9);
  EXPECT_NEAR(target[1], aep[1], 1e-9);
  EXPECT_NEAR(target[2], aep[2], 1e-9);
}

TEST(Ripple, SwingLiftsAboveNominalZ) {
  const LegContext leg = make_leg();
  const StrideParams stride = make_stride(Vec3(0.18, 0.0, 0.0));
  auto ripple = strategies().at("ripple")();
  const double swing_mid = 0.5 * (1.0 - ripple->duty_factor());
  const Vec3 target = ripple->foot_target(swing_mid, stride, leg);
  EXPECT_GT(target[2], leg.nominal_stance[2] + 1e-6);
}

TEST(Ripple, StanceStaysAtGround) {
  const LegContext leg = make_leg();
  const StrideParams stride = make_stride(Vec3(0.18, 0.0, 0.0));
  auto ripple = strategies().at("ripple")();
  const double swing_end = 1.0 - ripple->duty_factor();  // 1/6 ~ 0.1667
  // Stance covers most of [0, 1) for ripple.
  for (double phase : {swing_end, 0.25, 0.5, 0.75, 0.99}) {
    const Vec3 target = ripple->foot_target(phase, stride, leg);
    EXPECT_NEAR(target[2], leg.nominal_stance[2], 1e-9);
  }
}

}  // namespace
}  // namespace hexa_gait

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
