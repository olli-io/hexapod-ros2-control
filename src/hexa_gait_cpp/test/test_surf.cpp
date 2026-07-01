// gtest port of hexa_gait/test/test_surf.py — exercises the Surf strategy
// (duty factor, tripod-clustered lift-off offsets, and the PEP/AEP/stance
// foot targets) as a pure function, no ROS.
//
// The Surf strategy class is not header-visible; it is built through the
// registry factory (strategies().at("surf")()). Python attributes map to
// methods: duty_factor(), unstable(), phase_offsets(). The exported
// SURF_OFFSETS constant has no C++ counterpart, so the
// `Surf.phase_offsets is SURF_OFFSETS` identity assertion is DROPPED (see the
// per-test comment); its offset values are read back via
// surf->phase_offsets().offsets() instead.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/gaits/registry.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {
namespace {

// Surf's duty factor, mirrored for the stride/PEP/AEP helpers below. Kept in one
// place so the constant matches the strategy under test.
constexpr double kSurfDutyFactor = 5.0 / 8.0;

std::unique_ptr<Strategy> make_surf() { return strategies().at("surf")(); }

// Port of _leg(): l_front with a concrete mount and nominal stance.
LegContext make_leg(Vec3 nominal = Vec3(0.2, 0.1, -0.1)) {
  LegContext leg;
  leg.name = "l_front";
  leg.mount_xyz = Vec3(0.083, 0.0575, 0.0);
  leg.mount_yaw = 30.0 * M_PI / 180.0;
  leg.nominal_stance = nominal;
  return leg;
}

// Port of _stride().
StrideParams make_stride(Vec3 stride = Vec3::Zero(),
                         double duty_factor = kSurfDutyFactor) {
  StrideParams s;
  s.stride_vector = stride;
  s.cycle_time = 0.9;
  s.duty_factor = duty_factor;
  s.swing_clearance = 0.03;
  s.swing_width = 0.0;
  s.controller_dt = 0.02;
  return s;
}

TEST(Surf, DutyFactorFiveEighths) {
  auto surf = make_surf();
  EXPECT_NEAR(surf->duty_factor(), 5.0 / 8.0, 1e-9);
  // Not in the Python source, but the C++ port checks the unstable flag here
  // since Surf.unstable == True and there is no dedicated Python test for it.
  EXPECT_TRUE(surf->unstable());
}

TEST(Surf, LiftOffsClusterByTripod) {
  // Lift-off happens at master = (1 - offset) mod 1. The two natural tripods
  // lift as staggered groups half a cycle apart; the 1/10 within-group stagger
  // keeps mixed airborne triples impossible.
  //
  // DROPPED: `Surf.phase_offsets is SURF_OFFSETS` — SURF_OFFSETS is not exported
  // from the C++ port, so the identity check has no counterpart. Its intent
  // (that the strategy exposes exactly these offsets) is preserved by reading
  // the values back off the strategy below.
  auto surf = make_surf();
  const auto& offsets = surf->phase_offsets().offsets();

  std::map<std::string, double> lift;
  for (const auto& [n, o] : offsets) {
    lift[n] = pymod(1.0 - o, 1.0);
  }
  EXPECT_NEAR(lift.at("r_front"), 4.0 / 5.0, 1e-9);
  EXPECT_NEAR(lift.at("l_middle"), 9.0 / 10.0, 1e-9);
  EXPECT_NEAR(lift.at("r_rear"), 0.0, 1e-9);
  // Mirror group: each contralateral leg half a cycle later, same internal
  // order.
  const std::pair<std::string, std::string> mirrors[] = {
      {"r_front", "l_front"},
      {"l_middle", "r_middle"},
      {"r_rear", "l_rear"},
  };
  for (const auto& [first, mirror] : mirrors) {
    EXPECT_NEAR(pymod(lift.at(mirror) - lift.at(first), 1.0), 0.5, 1e-9)
        << first << " -> " << mirror;
  }
}

TEST(Surf, AirborneSetNeverMixesTripodsBeyondAPair) {
  // At every master phase the airborne set is a subset of one tripod, plus at
  // most one trailing leg of the other at the group seams.
  auto surf = make_surf();
  const auto& offsets = surf->phase_offsets().offsets();

  const std::map<std::string, int> tripod = {
      {"r_front", 0}, {"l_middle", 0}, {"r_rear", 0},
      {"l_front", 1}, {"r_middle", 1}, {"l_rear", 1},
  };
  const double swing_end = 1.0 - kSurfDutyFactor;
  for (int i = 0; i < 4800; ++i) {
    const double master = (i + 0.5) / 4800.0;
    int in_a = 0;
    int in_b = 0;
    for (const auto& [n, o] : offsets) {
      if (pymod(master + o, 1.0) < swing_end) {
        (tripod.at(n) == 0 ? in_a : in_b)++;
      }
    }
    EXPECT_LE(std::min(in_a, in_b), 1) << "master=" << master;
  }
}

TEST(Surf, ZeroStrideHoldsNominalXyAtAllPhases) {
  auto surf = make_surf();
  const LegContext leg = make_leg();
  const StrideParams stride = make_stride();
  for (double phase : {0.0, 0.1, 0.25, 0.5, 0.7, 0.99}) {
    const Vec3 target = surf->foot_target(phase, stride, leg);
    EXPECT_NEAR(target[0], leg.nominal_stance[0], 1e-9);
    EXPECT_NEAR(target[1], leg.nominal_stance[1], 1e-9);
  }
}

TEST(Surf, PhaseZeroEmitsPep) {
  auto surf = make_surf();
  const LegContext leg = make_leg();
  const Vec3 stride_vec(0.18, 0.0, 0.0);
  const StrideParams stride = make_stride(stride_vec);
  const Vec3 pep(leg.nominal_stance[0] - 0.5 * stride_vec[0],
                 leg.nominal_stance[1] - 0.5 * stride_vec[1],
                 leg.nominal_stance[2] - 0.5 * stride_vec[2]);
  const Vec3 target = surf->foot_target(0.0, stride, leg);
  EXPECT_NEAR(target[0], pep[0], 1e-9);
  EXPECT_NEAR(target[1], pep[1], 1e-9);
  EXPECT_NEAR(target[2], pep[2], 1e-9);
}

TEST(Surf, TouchdownPhaseEmitsAep) {
  auto surf = make_surf();
  const LegContext leg = make_leg();
  const Vec3 stride_vec(0.18, 0.0, 0.0);
  const StrideParams stride = make_stride(stride_vec);
  const Vec3 aep(leg.nominal_stance[0] + 0.5 * stride_vec[0],
                 leg.nominal_stance[1] + 0.5 * stride_vec[1],
                 leg.nominal_stance[2] + 0.5 * stride_vec[2]);
  // phase = 1 - beta = 3/8 is touchdown for surf.
  const double swing_end = 1.0 - kSurfDutyFactor;
  const Vec3 target = surf->foot_target(swing_end, stride, leg);
  EXPECT_NEAR(target[0], aep[0], 1e-9);
  EXPECT_NEAR(target[1], aep[1], 1e-9);
  EXPECT_NEAR(target[2], aep[2], 1e-9);
}

TEST(Surf, StanceStaysAtGround) {
  auto surf = make_surf();
  const LegContext leg = make_leg();
  const StrideParams stride = make_stride(Vec3(0.18, 0.0, 0.0));
  const double swing_end = 1.0 - kSurfDutyFactor;
  for (double phase : {swing_end, swing_end + 0.1, 0.7, 0.99}) {
    const Vec3 target = surf->foot_target(phase, stride, leg);
    EXPECT_NEAR(target[2], leg.nominal_stance[2], 1e-9);
  }
}

}  // namespace
}  // namespace hexa_gait

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
