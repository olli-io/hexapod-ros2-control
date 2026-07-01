// gtest port of hexa_gait/test/test_stability.py — quasi-static stability
// margins of the gait strategies.
//
// Evaluates each registered strategy closed-form at constant command: per-leg
// phases from the strategy's own offsets, stance feet from its pure
// foot_target, and support_polygon_margin sampled across a full master cycle
// for a sweep of commands at fractions of the gait's velocity cap. At constant
// cmd_vel the engine's StanceIntegrator reproduces the strategy's stance Bezier
// exactly, so this pins the steady-state stability of (phase offsets, beta)
// without engine state.
//
// CoM is the body origin (no posture offsets in this layer). Sampling uses a
// half-offset grid plus points just before and just after every lift-off /
// touchdown seam; the seam instants themselves are measure-zero hand-offs where
// the foot is grounded on both sides.
//
// Per-gait floors are pinned a few mm under the measured worst case so an
// offsets / beta edit that degrades stability fails loudly. Measured worst
// margins with this geometry (commands up to 80 % of each gait's cap):
//
// - tripod   — +0.018 m
// - tetrapod — +0.026 m
// - crawl    — +0.043 m
// - surf     — +0.044 m
// - ripple   — +0.054 m
//
// Surf with evenly spread metachronal offsets is quasi-statically unstable at
// every duty factor below tetrapod's (~ -0.013 m at beta = 5/8); the
// tripod-grouped offsets in gaits/surf.py are what make it positive. If a floor
// fails, look at the offset tables first.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "hexa_gait_cpp/clock.hpp"
#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/gaits/registry.hpp"
#include "hexa_gait_cpp/stability.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {
namespace {

// ─── support_polygon_margin unit behaviour ──────────────────────────

TEST(Stability, MarginSquareCentre) {
  const std::vector<std::pair<double, double>> feet = {
      {0.1, 0.1}, {0.1, -0.1}, {-0.1, -0.1}, {-0.1, 0.1}};
  EXPECT_NEAR(support_polygon_margin(feet), 0.1, 1e-9);
}

TEST(Stability, MarginNegativeOutside) {
  const std::vector<std::pair<double, double>> feet = {
      {0.1, 0.1}, {0.1, 0.2}, {0.2, 0.15}};
  const double m = support_polygon_margin(feet);
  EXPECT_LT(m, 0.0);
  // Nearest hull point is (0.1, 0.1).
  EXPECT_NEAR(m, -std::hypot(0.1, 0.1), 1e-9);
}

TEST(Stability, MarginDegenerateSupportNeverPositive) {
  EXPECT_EQ(support_polygon_margin({}),
            -std::numeric_limits<double>::infinity());
  EXPECT_NEAR(support_polygon_margin({{0.05, 0.0}}), -0.05, 1e-9);
  // Two feet: distance to the segment, negated.
  EXPECT_NEAR(support_polygon_margin({{0.1, 0.1}, {0.1, -0.1}}), -0.1, 1e-9);
  // Collinear triple is still a degenerate hull.
  EXPECT_NEAR(
      support_polygon_margin({{0.1, -0.1}, {0.1, 0.0}, {0.1, 0.1}}), -0.1,
      1e-9);
}

TEST(Stability, MarginComOffset) {
  const std::vector<std::pair<double, double>> feet = {
      {0.1, 0.1}, {0.1, -0.1}, {-0.1, -0.1}, {-0.1, 0.1}};
  EXPECT_NEAR(support_polygon_margin(feet, {0.05, 0.0}), 0.05, 1e-9);
}

// ─── steady-state strategy sweep ─────────────────────────────────────

struct Nominal {
  std::string name;
  double x;
  double y;
  double z;
};

// Mirror of _NOMINAL: foot directly under each hip at a fixed walk-plane z.
const std::vector<Nominal>& nominal_table() {
  static const std::vector<Nominal> table = {
      {"l_front", 0.15, 0.10, -0.10},   {"r_front", 0.15, -0.10, -0.10},
      {"l_middle", 0.0, 0.12, -0.10},   {"r_middle", 0.0, -0.12, -0.10},
      {"l_rear", -0.15, 0.10, -0.10},   {"r_rear", -0.15, -0.10, -0.10},
  };
  return table;
}

// Mirror of _CTX: LegContext with mount at (x, y, 0) and nominal at (x, y, z).
const std::map<std::string, LegContext>& contexts() {
  static const std::map<std::string, LegContext> ctx = [] {
    std::map<std::string, LegContext> out;
    for (const auto& n : nominal_table()) {
      LegContext c;
      c.name = n.name;
      c.mount_xyz = Vec3(n.x, n.y, 0.0);
      c.mount_yaw = 0.0;
      c.nominal_stance = Vec3(n.x, n.y, n.z);
      out[n.name] = c;
    }
    return out;
  }();
  return ctx;
}

constexpr double kStrideLength = 0.10;
constexpr double kMinSwingTime = 0.30;
constexpr double kMaxSwingTime = 0.40;
constexpr int kNGrid = 480;

double r_outer() {
  double r = 0.0;
  for (const auto& n : nominal_table()) {
    r = std::max(r, std::hypot(n.x, n.y));
  }
  return r;
}

// Mirror of _commands: commands at fractions of the gait's own velocity cap.
// The cap mirrors the engine's envelope:
// stride_length * (1 - beta) / (min_swing_time * beta); yaw commands scale the
// same per-leg budget by the outer nominal-foot radius.
std::vector<std::array<double, 3>> commands(double beta) {
  const double cap =
      kStrideLength * (1.0 - beta) / (kMinSwingTime * beta);
  const double r = r_outer();
  return {
      {0.8 * cap, 0.0, 0.0},
      {0.0, 0.8 * cap, 0.0},
      {0.55 * cap, 0.55 * cap, 0.0},
      {0.0, 0.0, 0.8 * cap / r},
      {0.4 * cap, 0.0, 0.4 * cap / r},
  };
}

// Mirror of _stride_params: per-leg StrideParams at constant cmd, mirroring a
// GAIT tick.
std::map<std::string, StrideParams> stride_params(
    const Strategy& strategy, const std::array<double, 3>& cmd) {
  const double beta = strategy.duty_factor();
  const auto v_legs =
      per_leg_planar_velocity(contexts(), {cmd[0], cmd[1]}, cmd[2]);
  double max_v = 0.0;
  for (const auto& [name, v] : v_legs) {
    max_v = std::max(max_v, std::hypot(v.first, v.second));
  }
  const double cycle_time = derive_cycle_time(
      max_v, kStrideLength, beta, kMinSwingTime / (1.0 - beta),
      kMaxSwingTime / (1.0 - beta));
  const double stance_time = cycle_time * beta;
  std::map<std::string, StrideParams> out;
  for (const auto& [name, v] : v_legs) {
    StrideParams p;
    p.stride_vector =
        stride_vector(v.first, v.second, stance_time, kStrideLength);
    p.cycle_time = cycle_time;
    p.duty_factor = beta;
    p.swing_clearance = 0.03;
    p.swing_width = 0.0;
    p.controller_dt = 0.02;
    out[name] = p;
  }
  return out;
}

// Mirror of _masters: half-offset grid plus samples either side of every seam.
std::vector<double> masters(const Strategy& strategy) {
  const double beta = strategy.duty_factor();
  std::set<double> seams;
  for (const auto& [leg, o] : strategy.phase_offsets().offsets()) {
    seams.insert(pymod(1.0 - o, 1.0));                  // lift-off
    seams.insert(pymod(1.0 - o - (1.0 - beta), 1.0));   // touchdown
  }
  std::vector<double> out;
  out.reserve(kNGrid + 2 * seams.size());
  for (int i = 0; i < kNGrid; ++i) {
    out.push_back((i + 0.5) / kNGrid);
  }
  for (const double e : seams) {
    out.push_back(pymod(e + 1e-6, 1.0));
    out.push_back(pymod(e - 1e-6, 1.0));
  }
  return out;
}

// Mirror of _worst_margin.
double worst_margin(const std::string& gait_name) {
  const auto strategy = strategies().at(gait_name)();
  const double beta = strategy->duty_factor();
  const auto& offsets = strategy->phase_offsets().offsets();
  double worst = std::numeric_limits<double>::infinity();
  for (const auto& cmd : commands(beta)) {
    const auto params = stride_params(*strategy, cmd);
    for (const double master : masters(*strategy)) {
      std::vector<std::pair<double, double>> feet;
      for (const auto& name : LEG_NAMES) {
        const double phase = pymod(master + offsets.at(name), 1.0);
        if (phase < 1.0 - beta) {
          continue;  // swing — foot airborne
        }
        const Vec3 target =
            strategy->foot_target(phase, params.at(name), contexts().at(name));
        feet.push_back({target[0], target[1]});
      }
      worst = std::min(worst, support_polygon_margin(feet));
    }
  }
  return worst;
}

// Mirror of _MARGIN_FLOORS.
const std::map<std::string, double>& margin_floors() {
  static const std::map<std::string, double> floors = {
      {"tripod", 0.012}, {"tetrapod", 0.020}, {"crawl", 0.038},
      {"surf", 0.038},   {"ripple", 0.048},
  };
  return floors;
}

TEST(Stability, EveryRegisteredGaitHasAPinnedFloor) {
  std::set<std::string> floor_names;
  for (const auto& [name, floor] : margin_floors()) {
    floor_names.insert(name);
  }
  std::set<std::string> strategy_names;
  for (const auto& [name, factory] : strategies()) {
    strategy_names.insert(name);
  }
  EXPECT_EQ(floor_names, strategy_names);
}

// Port of the parametrized test_gait_keeps_com_inside_support_polygon: a single
// TEST looping over the (sorted) pinned floors with SCOPED_TRACE per gait.
TEST(Stability, GaitKeepsComInsideSupportPolygon) {
  for (const auto& [gait_name, floor] : margin_floors()) {
    SCOPED_TRACE(gait_name);
    const double worst = worst_margin(gait_name);
    EXPECT_GT(worst, floor)
        << gait_name << ": worst quasi-static margin " << worst
        << " m (pinned floor " << floor << " m)";
  }
}

TEST(Stability, UnstableMarksPinSurfAndCrawl) {
  // The unstable class attribute is what teleop's allow_unstable_gaits: false
  // filters out of the D-pad gait rotation. Pin the set so a new or edited
  // strategy makes an explicit choice here.
  std::set<std::string> unstable;
  for (const auto& [name, factory] : strategies()) {
    if (factory()->unstable()) {
      unstable.insert(name);
    }
  }
  const std::set<std::string> expected = {"surf", "crawl"};
  EXPECT_EQ(unstable, expected);
}

}  // namespace
}  // namespace hexa_gait

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
