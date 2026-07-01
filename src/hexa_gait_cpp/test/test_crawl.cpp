// gtest port of hexa_gait/test/test_crawl.py — exercises the pure Crawl gait
// strategy (no ROS). Strategy classes are not header-visible, so the strategy is
// built via the registry factory (strategies().at("crawl")()). Attributes become
// methods: Crawl.duty_factor -> crawl->duty_factor(), Crawl.phase_offsets ->
// crawl->phase_offsets(), Crawl().foot_target(...) -> crawl->foot_target(...).

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/gaits/registry.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {
namespace {

// Port of the _leg() fixture.
LegContext make_leg(Vec3 nominal = Vec3(0.2, 0.1, -0.1)) {
  LegContext ctx;
  ctx.name = "l_front";
  ctx.mount_xyz = Vec3(0.083, 0.0575, 0.0);
  ctx.mount_yaw = 30.0 * M_PI / 180.0;
  ctx.nominal_stance = nominal;
  return ctx;
}

// Port of the _stride() fixture. Default duty_factor is Crawl's 2/3.
StrideParams make_stride(Vec3 stride = Vec3(0.0, 0.0, 0.0),
                         double duty_factor = 2.0 / 3.0) {
  StrideParams s;
  s.stride_vector = stride;
  s.cycle_time = 1.5;
  s.duty_factor = duty_factor;
  s.swing_clearance = 0.03;
  s.swing_width = 0.0;
  s.controller_dt = 0.02;
  return s;
}

TEST(Crawl, DutyFactorTwoThirds) {
  auto crawl = strategies().at("crawl")();
  EXPECT_NEAR(crawl->duty_factor(), 2.0 / 3.0, 1e-12);
}

TEST(Crawl, UsesMetachronalOffsets) {
  // Crawl and Ripple share METACHRONAL_OFFSETS; the difference is duty factor
  // only. Offsets are the mirror of lift-off times (lift-off at master =
  // (1 - offset) mod 1), so the realized wave is rear -> middle -> front per
  // side with the left side half a cycle later.
  auto crawl = strategies().at("crawl")();
  auto ripple = strategies().at("ripple")();
  // Python asserted `Crawl.phase_offsets is METACHRONAL_OFFSETS`. METACHRONAL_
  // OFFSETS is not exported; both crawl and ripple return the same internal
  // static table by reference, so the identity check becomes a pointer-equality
  // check against ripple's phase_offsets.
  EXPECT_EQ(&crawl->phase_offsets(), &ripple->phase_offsets());

  const auto& o = crawl->phase_offsets().offsets();
  EXPECT_NEAR(o.at("r_rear"), 0.0, 1e-9);
  EXPECT_NEAR(o.at("r_middle"), 2.0 / 3.0, 1e-9);
  EXPECT_NEAR(o.at("r_front"), 1.0 / 3.0, 1e-9);
  EXPECT_NEAR(o.at("l_rear"), 0.5, 1e-9);
  EXPECT_NEAR(o.at("l_middle"), 1.0 / 6.0, 1e-9);
  EXPECT_NEAR(o.at("l_front"), 5.0 / 6.0, 1e-9);
}

TEST(Crawl, ZeroStrideHoldsNominalXyAtAllPhases) {
  const LegContext leg = make_leg();
  const StrideParams stride = make_stride();
  auto crawl = strategies().at("crawl")();
  for (double phase : {0.0, 0.1, 0.25, 0.3, 0.5, 0.7, 0.99}) {
    const Vec3 target = crawl->foot_target(phase, stride, leg);
    EXPECT_NEAR(target[0], leg.nominal_stance[0], 1e-9);
    EXPECT_NEAR(target[1], leg.nominal_stance[1], 1e-9);
  }
}

TEST(Crawl, PhaseZeroEmitsPep) {
  const LegContext leg = make_leg();
  const Vec3 stride_vec(0.18, 0.0, 0.0);
  const StrideParams stride = make_stride(stride_vec);
  auto crawl = strategies().at("crawl")();
  const Vec3 pep(leg.nominal_stance[0] - 0.5 * stride_vec[0],
                 leg.nominal_stance[1] - 0.5 * stride_vec[1],
                 leg.nominal_stance[2] - 0.5 * stride_vec[2]);
  const Vec3 target = crawl->foot_target(0.0, stride, leg);
  EXPECT_NEAR(target[0], pep[0], 1e-9);
  EXPECT_NEAR(target[1], pep[1], 1e-9);
  EXPECT_NEAR(target[2], pep[2], 1e-9);
}

TEST(Crawl, TouchdownPhaseEmitsAep) {
  const LegContext leg = make_leg();
  const Vec3 stride_vec(0.18, 0.0, 0.0);
  const StrideParams stride = make_stride(stride_vec);
  auto crawl = strategies().at("crawl")();
  const Vec3 aep(leg.nominal_stance[0] + 0.5 * stride_vec[0],
                 leg.nominal_stance[1] + 0.5 * stride_vec[1],
                 leg.nominal_stance[2] + 0.5 * stride_vec[2]);
  // phase = 1 - beta = 1/3 is touchdown for crawl.
  const double swing_end = 1.0 - crawl->duty_factor();
  const Vec3 target = crawl->foot_target(swing_end, stride, leg);
  EXPECT_NEAR(target[0], aep[0], 1e-9);
  EXPECT_NEAR(target[1], aep[1], 1e-9);
  EXPECT_NEAR(target[2], aep[2], 1e-9);
}

TEST(Crawl, SwingLiftsAboveNominalZ) {
  const LegContext leg = make_leg();
  const StrideParams stride = make_stride(Vec3(0.18, 0.0, 0.0));
  auto crawl = strategies().at("crawl")();
  // Mid-swing is at half of [0, 1 - beta).
  const double swing_mid = 0.5 * (1.0 - crawl->duty_factor());
  const Vec3 target = crawl->foot_target(swing_mid, stride, leg);
  EXPECT_GT(target[2], leg.nominal_stance[2] + 1e-6);
}

TEST(Crawl, StanceStaysAtGround) {
  const LegContext leg = make_leg();
  const StrideParams stride = make_stride(Vec3(0.18, 0.0, 0.0));
  auto crawl = strategies().at("crawl")();
  const double swing_end = 1.0 - crawl->duty_factor();
  // Stance covers [swing_end, 1.0).
  for (double phase : {swing_end, swing_end + 0.1, 0.6, 0.99}) {
    const Vec3 target = crawl->foot_target(phase, stride, leg);
    EXPECT_NEAR(target[2], leg.nominal_stance[2], 1e-9);
  }
}

}  // namespace
}  // namespace hexa_gait

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
