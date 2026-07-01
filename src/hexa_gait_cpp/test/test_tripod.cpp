// gtest port of hexa_gait/test/test_tripod.py — exercises the pure Tripod
// strategy (duty 0.5) via the registry factory. Behavioural parity with the
// Python suite: each test builds a LegContext / StrideParams and asserts
// foot_target's phase / PEP / AEP / swing-lift / stance behaviour.
//
// The Python tests construct `Tripod()` directly; the C++ strategy classes are
// not header-visible, so we build via strategies().at("tripod")() from the
// registry factory instead.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/gaits/registry.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {
namespace {

// Port of _leg().
LegContext make_leg(Vec3 nominal = Vec3(0.2, 0.1, -0.1)) {
  LegContext leg;
  leg.name = "l_front";
  leg.mount_xyz = Vec3(0.083, 0.0575, 0.0);
  leg.mount_yaw = 30.0 * M_PI / 180.0;  // math.radians(30)
  leg.nominal_stance = nominal;
  return leg;
}

// Port of _stride().
StrideParams make_stride(Vec3 stride = Vec3(0.0, 0.0, 0.0)) {
  StrideParams s;
  s.stride_vector = stride;
  s.cycle_time = 1.2;
  s.duty_factor = 0.5;
  s.swing_clearance = 0.03;
  s.swing_width = 0.0;
  s.controller_dt = 0.02;
  return s;
}

TEST(Tripod, ZeroStrideHoldsNominalXyAtAllPhases) {
  const LegContext leg = make_leg();
  const StrideParams stride = make_stride();
  auto tripod = strategies().at("tripod")();
  // With zero stride, PEP = AEP = nominal, so the foot's XY stays at
  // nominal_xy regardless of phase. Z still lifts during swing (the
  // trajectory's mid_z is nominal_z + swing_clearance even when the
  // stride collapses to a point).
  for (double phase : {0.0, 0.1, 0.25, 0.49, 0.5, 0.75, 0.99}) {
    const Vec3 target = tripod->foot_target(phase, stride, leg);
    EXPECT_NEAR(target[0], leg.nominal_stance[0], 1e-9);
    EXPECT_NEAR(target[1], leg.nominal_stance[1], 1e-9);
  }
  // Stance phase keeps Z at nominal too (no lift).
  for (double phase : {0.5, 0.7, 0.99}) {
    EXPECT_NEAR(tripod->foot_target(phase, stride, leg)[2],
                leg.nominal_stance[2], 1e-9);
  }
}

TEST(Tripod, PhaseZeroEmitsPep) {
  const LegContext leg = make_leg();
  const Vec3 stride_vec(0.18, 0.0, 0.0);
  const StrideParams stride = make_stride(stride_vec);
  auto tripod = strategies().at("tripod")();
  const Vec3 pep(leg.nominal_stance[0] - 0.5 * stride_vec[0],
                 leg.nominal_stance[1] - 0.5 * stride_vec[1],
                 leg.nominal_stance[2] - 0.5 * stride_vec[2]);
  const Vec3 target = tripod->foot_target(0.0, stride, leg);
  EXPECT_NEAR(target[0], pep[0], 1e-9);
  EXPECT_NEAR(target[1], pep[1], 1e-9);
  EXPECT_NEAR(target[2], pep[2], 1e-9);
}

TEST(Tripod, TouchdownPhaseEmitsAep) {
  const LegContext leg = make_leg();
  const Vec3 stride_vec(0.18, 0.0, 0.0);
  const StrideParams stride = make_stride(stride_vec);
  auto tripod = strategies().at("tripod")();
  const Vec3 aep(leg.nominal_stance[0] + 0.5 * stride_vec[0],
                 leg.nominal_stance[1] + 0.5 * stride_vec[1],
                 leg.nominal_stance[2] + 0.5 * stride_vec[2]);
  // Phase = 0.5 is the start of stance (AEP); the stance curve starts here.
  const Vec3 target = tripod->foot_target(0.5, stride, leg);
  EXPECT_NEAR(target[0], aep[0], 1e-9);
  EXPECT_NEAR(target[1], aep[1], 1e-9);
  EXPECT_NEAR(target[2], aep[2], 1e-9);
}

TEST(Tripod, SwingLiftsAboveNominalZ) {
  const LegContext leg = make_leg();
  const StrideParams stride = make_stride(Vec3(0.18, 0.0, 0.0));
  auto tripod = strategies().at("tripod")();
  // Mid-swing the foot should be at the apex height (above PEP/AEP z).
  const Vec3 target = tripod->foot_target(0.25, stride, leg);
  EXPECT_GT(target[2], leg.nominal_stance[2] + 1e-6);
}

TEST(Tripod, StanceStaysAtGround) {
  const LegContext leg = make_leg();
  const StrideParams stride = make_stride(Vec3(0.18, 0.0, 0.0));
  auto tripod = strategies().at("tripod")();
  // Stance covers [0.5, 1.0); z should equal nominal_z throughout.
  for (double phase : {0.5, 0.7, 0.9, 0.99}) {
    const Vec3 target = tripod->foot_target(phase, stride, leg);
    EXPECT_NEAR(target[2], leg.nominal_stance[2], 1e-9);
  }
}

}  // namespace
}  // namespace hexa_gait

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
