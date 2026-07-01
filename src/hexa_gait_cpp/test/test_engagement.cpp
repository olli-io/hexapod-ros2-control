// gtest port of hexa_gait/test/test_engagement.py — exercises the pure C++
// EngagementController (no ROS). Behavioural parity with the Python suite.
//
// The Python suite reaches into the private ctrl._master in several guards and
// in test_internal_v_body_smoothstep_then_holds. There is no public master()
// getter; the substitute is exit_master(), which equals _master throughout
// engagement (exit_master() == pymod(master_, 1.0) and master_ stays in [0, 1)
// until the final wrap on DONE). Each substitution is flagged inline below.
//
// pytest.mark.parametrize over (Tripod, Crawl, Ripple) is realised as a loop
// over the registry strategy names within one TEST, mirroring
// test_engine.cpp's DeriveCycleTimeReadsStrategyDutyFactor.

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "hexa_gait_cpp/clock.hpp"
#include "hexa_gait_cpp/engagement.hpp"
#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/gaits/registry.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {
namespace {

// Symmetric stance shared with test_engine.py. Front / rear sit at
// hypot(0.15, 0.10) m from the body centre; middle legs at 0.12 m.
const std::map<std::string, Vec3>& mounts() {
  static const std::map<std::string, Vec3> m = {
      {"l_front", Vec3(0.15, 0.10, 0.0)},
      {"r_front", Vec3(0.15, -0.10, 0.0)},
      {"l_middle", Vec3(0.0, 0.12, 0.0)},
      {"r_middle", Vec3(0.0, -0.12, 0.0)},
      {"l_rear", Vec3(-0.15, 0.10, 0.0)},
      {"r_rear", Vec3(-0.15, -0.10, 0.0)},
  };
  return m;
}

const std::vector<std::string>& tripod_a() {
  static const std::vector<std::string> a = {"l_front", "r_middle", "l_rear"};
  return a;
}
const std::vector<std::string>& tripod_b() {
  static const std::vector<std::string> b = {"r_front", "l_middle", "r_rear"};
  return b;
}

// Engagement reference parameters. With stride_length = 0.10 m,
// duty_factor = 0.5 and cmd v_x = 0.20 m/s the engine derives cycle_time = 1.0 s.
constexpr double kStrideLength = 0.10;
constexpr double kDutyFactor = 0.5;
constexpr double kSwingClearance = 0.03;
constexpr double kSwingWidth = 0.0;
constexpr double kControllerDt = 0.02;

std::map<std::string, Vec3> nominal_stance() {
  std::map<std::string, Vec3> out;
  for (const auto& [n, xyz] : mounts()) {
    out[n] = Vec3(xyz[0], xyz[1], -0.10);
  }
  return out;
}

std::map<std::string, LegContext> leg_contexts() {
  const auto nominal = nominal_stance();
  std::map<std::string, LegContext> out;
  for (const auto& n : LEG_NAMES) {
    LegContext ctx;
    ctx.name = n;
    ctx.mount_xyz = mounts().at(n);
    ctx.mount_yaw = 0.0;
    ctx.nominal_stance = nominal.at(n);
    out[n] = ctx;
  }
  return out;
}

// Mirror of _controller(**overrides): named defaults with per-field overrides
// via designated initialisers (C++20).
struct ControllerParams {
  double stride_length = kStrideLength;
  double min_cycle_time = 0.5;
  double max_cycle_time = 2.0;
  double duty_factor = kDutyFactor;
  double swing_clearance = kSwingClearance;
  double swing_width = kSwingWidth;
  double controller_dt = kControllerDt;
};

EngagementController make_controller(ControllerParams p = {}) {
  return EngagementController(nominal_stance(), p.stride_length,
                             p.min_cycle_time, p.max_cycle_time, p.duty_factor,
                             p.swing_clearance, p.swing_width, p.controller_dt);
}

// Port of _begin: build the registry strategy, arm the controller, and return
// the owning pointer so it outlives every update() call (begin() stores a raw
// pointer into the controller).
std::unique_ptr<Strategy> begin_with(EngagementController& ctrl,
                                     const std::string& name = "tripod") {
  auto strategy = strategies().at(name)();
  ctrl.begin(*strategy, leg_contexts());
  return strategy;
}

// Port of _drive: tick until DONE and return the final per-leg output.
std::map<std::string, LegOutput> drive(EngagementController& ctrl,
                                       std::pair<double, double> v_cmd_xy,
                                       double omega_cmd = 0.0, double dt = 0.02,
                                       int max_ticks = 400) {
  std::map<std::string, LegOutput> last;
  for (int i = 0; i < max_ticks; ++i) {
    last = ctrl.update(dt, v_cmd_xy, omega_cmd);
    if (ctrl.state() == EngagementState::DONE) {
      return last;
    }
  }
  ADD_FAILURE() << "engagement did not reach DONE within max_ticks";
  return last;
}

// Tripod phase offsets — the anonymous-namespace tripod_offsets() in
// registry.cpp is not exported, so mirror its values here for the bad strategy.
PhaseOffsets tripod_offsets() {
  return PhaseOffsets({
      {"l_front", 0.0},
      {"r_middle", 0.0},
      {"l_rear", 0.0},
      {"r_front", 0.5},
      {"l_middle", 0.5},
      {"r_rear", 0.5},
  });
}

// Port of _expected_gait_foot: the strategy-prescribed body-frame foot target
// for a given master phase and constant forward command.
Vec3 expected_gait_foot(const Strategy& strategy, const std::string& name,
                        double master, double v_cmd_x,
                        double stride_length = kStrideLength) {
  const auto legs = leg_contexts();
  const double offset = strategy.phase_offsets().at(name);
  const double phase = pymod(master + offset, 1.0);
  const double duty = strategy.duty_factor();
  const double cycle_time = stride_length / (v_cmd_x * duty);
  const double stance_time = cycle_time * duty;
  StrideParams stride;
  stride.stride_vector = Vec3(v_cmd_x * stance_time, 0.0, 0.0);
  stride.cycle_time = cycle_time;
  stride.duty_factor = duty;
  stride.swing_clearance = kSwingClearance;
  stride.swing_width = kSwingWidth;
  stride.controller_dt = kControllerDt;
  return strategy.foot_target(phase, stride, legs.at(name));
}

// ── Tests ─────────────────────────────────────────────────────────────────

TEST(Engagement, FirstTickNoPositionJumpAnywhere) {
  // The bug this controller exists to fix: at master ~ 0, no foot should have
  // lurched away from NOMINAL. Both tripods stay close.
  EngagementController ctrl = make_controller();
  auto strategy = begin_with(ctrl);
  const auto nominal = nominal_stance();
  const auto out = ctrl.update(0.02, {0.20, 0.0}, 0.0);
  for (const auto& name : LEG_NAMES) {
    const double dx = std::abs(out.at(name).foot_target[0] - nominal.at(name)[0]);
    const double dy = std::abs(out.at(name).foot_target[1] - nominal.at(name)[1]);
    EXPECT_LT(dx, 5e-3) << name << " jumped horizontally";
    EXPECT_LT(dy, 5e-3);
  }
}

TEST(Engagement, TripodRolesDuringInitialHalfCycle) {
  // During master < 0.5 (tripod's first touchdown), Tripod A is in
  // INITIAL_SWING and Tripod B in INITIAL_STANCE. The window is restricted to
  // the first half cycle. Python guarded on ctrl._master < 0.4; exit_master()
  // equals _master throughout engagement.
  EngagementController ctrl = make_controller();
  auto strategy = begin_with(ctrl);
  const auto nominal = nominal_stance();
  while (ctrl.state() != EngagementState::DONE && ctrl.exit_master() < 0.4) {
    const auto out = ctrl.update(0.02, {0.20, 0.0}, 0.0);
    for (const auto& name : tripod_a()) {
      EXPECT_FALSE(out.at(name).stance);
    }
    for (const auto& name : tripod_b()) {
      EXPECT_TRUE(out.at(name).stance);
      EXPECT_NEAR(out.at(name).foot_target[2], nominal.at(name)[2], 1e-12);
    }
  }
}

TEST(Engagement, ConstantCmdTripodALandsAtPep) {
  // Tripod A is initial-swing: leaves NOMINAL at master = 0, lands at AEP at
  // master = 0.5, then runs GAIT_LIKE stance to PEP by master = 1.0.
  EngagementController ctrl = make_controller();
  auto strategy = begin_with(ctrl);
  const auto out = drive(ctrl, {0.20, 0.0});
  const auto nominal = nominal_stance();
  const double stride_x = 0.10;  // v = 0.20, beta = 0.5, stance_time = 0.5 s.
  for (const auto& name : tripod_a()) {
    const double pep_x = nominal.at(name)[0] - 0.5 * stride_x;
    EXPECT_NEAR(out.at(name).foot_target[0], pep_x, 3e-3)
        << name << " did not reach PEP";
    EXPECT_NEAR(out.at(name).foot_target[1], nominal.at(name)[1], 1e-6);
  }
}

TEST(Engagement, ConstantCmdTripodBLandsAtAep) {
  // Tripod B is initial-stance: integrates stance from NOMINAL to PEP over
  // master [0, 0.5], then swings to live AEP by master = 1.0.
  EngagementController ctrl = make_controller();
  auto strategy = begin_with(ctrl);
  const auto out = drive(ctrl, {0.20, 0.0});
  const auto nominal = nominal_stance();
  const double stride_x = 0.10;
  for (const auto& name : tripod_b()) {
    const double aep_x = nominal.at(name)[0] + 0.5 * stride_x;
    EXPECT_NEAR(out.at(name).foot_target[0], aep_x, 3e-3)
        << name << " did not reach AEP";
    EXPECT_NEAR(out.at(name).foot_target[1], nominal.at(name)[1], 1e-6);
  }
}

TEST(Engagement, SwingClearsStepHeight) {
  const double swing_clearance = 0.03;
  ControllerParams p;
  p.swing_clearance = swing_clearance;
  EngagementController ctrl = make_controller(p);
  auto strategy = begin_with(ctrl);
  const auto nominal = nominal_stance();
  std::map<std::string, bool> seen_lifted;
  for (const auto& n : tripod_a()) {
    seen_lifted[n] = false;
  }
  for (int i = 0; i < 40; ++i) {
    const auto out = ctrl.update(0.02, {0.20, 0.0}, 0.0);
    for (const auto& name : tripod_a()) {
      if (out.at(name).foot_target[2] >
          nominal.at(name)[2] + swing_clearance * 0.5) {
        seen_lifted[name] = true;
      }
    }
    if (ctrl.state() == EngagementState::DONE) {
      break;
    }
  }
  for (const auto& [name, lifted] : seen_lifted) {
    EXPECT_TRUE(lifted) << "swing never lifted: " << name;
  }
}

TEST(Engagement, InternalVBodySmoothstepThenHolds) {
  // The internal body velocity ramps via smoothstep over [0, smoothstep_window]
  // then holds at cmd_vel. Python read ctrl._master; exit_master() equals it
  // during engagement.
  const double cmd_v_x = 0.20;
  EngagementController ctrl = make_controller();
  auto strategy = begin_with(ctrl);
  std::vector<double> samples;
  std::vector<double> saturated_masters;
  while (ctrl.state() != EngagementState::DONE) {
    ctrl.update(0.005, {cmd_v_x, 0.0}, 0.0);
    const double envelope = ctrl.v_body()[0] / cmd_v_x;
    samples.push_back(envelope);
    if (ctrl.exit_master() >= ctrl.smoothstep_window()) {
      saturated_masters.push_back(envelope);
    }
  }

  ASSERT_FALSE(samples.empty());
  EXPECT_NEAR(samples.front(), 0.0, 0.05);
  EXPECT_NEAR(samples.back(), 1.0, 1e-9);
  // Monotone non-decreasing across the whole cycle.
  for (size_t i = 1; i < samples.size(); ++i) {
    EXPECT_GE(samples[i], samples[i - 1] - 1e-9);
  }
  // Hold at 1.0 once master passes the window; allow a one-tick warm-up.
  for (size_t i = 1; i < saturated_masters.size(); ++i) {
    EXPECT_NEAR(saturated_masters[i], 1.0, 1e-9);
  }
}

TEST(Engagement, SmoothstepWindowMatchesFirstTouchdown) {
  // Tripod's first touchdown is master = 0.5; crawl / ripple earliest is 1/6.
  const std::vector<std::pair<std::string, double>> cases = {
      {"tripod", 0.5}, {"crawl", 1.0 / 6.0}, {"ripple", 1.0 / 6.0}};
  for (const auto& [name, expected] : cases) {
    auto proto = strategies().at(name)();
    ControllerParams p;
    p.duty_factor = proto->duty_factor();
    EngagementController ctrl = make_controller(p);
    auto strategy = begin_with(ctrl, name);
    EXPECT_NEAR(ctrl.smoothstep_window(), expected, 1e-12) << name;
  }
}

TEST(Engagement, SwingTouchdownVelocityMatchesSteadyState) {
  // swing_target_velocity = -v_body: foot velocity at swing -> stance handover
  // equals steady-state stance velocity (-v_cmd).
  const double cmd_v_x = 0.20;
  const double dt = 0.001;
  ControllerParams p;
  p.controller_dt = dt;
  EngagementController ctrl = make_controller(p);
  auto strategy = begin_with(ctrl);
  std::vector<std::pair<double, Vec3>> trace;
  double elapsed = 0.0;
  while (ctrl.state() != EngagementState::DONE) {
    const auto out = ctrl.update(dt, {cmd_v_x, 0.0}, 0.0);
    elapsed += dt;
    trace.push_back({elapsed, out.at("l_front").foot_target});
  }
  ASSERT_GE(trace.size(), 6u);
  const auto& [t_a, p_a] = trace[trace.size() - 6];
  const auto& [t_b, p_b] = trace.back();
  const double v_foot_x = (p_b[0] - p_a[0]) / (t_b - t_a);
  EXPECT_NEAR(v_foot_x, -cmd_v_x, 5e-3);
}

TEST(Engagement, EngagementTracksGrowingCmdVel) {
  // cmd_vel ramps 0 -> target during engagement; by master = 1.0 the cmd has
  // saturated, so Tripod A parks near PEP and Tripod B near AEP for final cmd.
  EngagementController ctrl = make_controller();
  auto strategy = begin_with(ctrl);
  const double target_v = 0.20;
  const double dt = 0.005;
  double elapsed = 0.0;
  std::map<std::string, LegOutput> last_out;
  bool have_out = false;
  while (ctrl.state() != EngagementState::DONE) {
    const double cmd_v_x = std::min(target_v, target_v * elapsed / 0.3);
    last_out = ctrl.update(dt, {cmd_v_x, 0.0}, 0.0);
    have_out = true;
    elapsed += dt;
  }
  ASSERT_TRUE(have_out);
  const auto nominal = nominal_stance();
  const double stride_x = 0.10;
  const double pep_x = nominal.at("l_front")[0] - 0.5 * stride_x;
  const double aep_x = nominal.at("r_front")[0] + 0.5 * stride_x;
  EXPECT_NEAR(last_out.at("l_front").foot_target[0], pep_x, 0.02);
  EXPECT_NEAR(last_out.at("r_front").foot_target[0], aep_x, 0.02);
}

TEST(Engagement, PureYawInnerVsOuterStride) {
  // Pure omega: outer legs (front/rear) sweep a larger tangential arc than
  // inner (middle). At engagement end each initial-stance leg lands at its AEP.
  const double omega = 1.0;
  EngagementController ctrl = make_controller();
  auto strategy = begin_with(ctrl);
  const auto out = drive(ctrl, {0.0, 0.0}, omega);
  const auto nominal = nominal_stance();

  const double outer_r = std::hypot(0.15, 0.10);
  const double inner_r = 0.12;
  const double expected_cycle = kStrideLength / (omega * outer_r * kDutyFactor);
  const double expected_stance_time = expected_cycle * kDutyFactor;

  const double outer_displacement = std::hypot(
      out.at("r_front").foot_target[0] - nominal.at("r_front")[0],
      out.at("r_front").foot_target[1] - nominal.at("r_front")[1]);
  const double inner_displacement = std::hypot(
      out.at("l_middle").foot_target[0] - nominal.at("l_middle")[0],
      out.at("l_middle").foot_target[1] - nominal.at("l_middle")[1]);
  const double expected_outer = 0.5 * omega * outer_r * expected_stance_time;
  const double expected_inner = 0.5 * omega * inner_r * expected_stance_time;
  EXPECT_NEAR(outer_displacement, expected_outer, 3e-3);
  EXPECT_NEAR(inner_displacement, expected_inner, 3e-3);
  EXPECT_GT(outer_displacement, inner_displacement);
}

TEST(Engagement, ExitMasterWrapsToZero) {
  // Engagement covers a full master cycle; the modular handoff phase is 0.0
  // regardless of the active gait's duty factor.
  for (const auto& name : {"tripod", "crawl", "ripple"}) {
    auto proto = strategies().at(name)();
    ControllerParams p;
    p.duty_factor = proto->duty_factor();
    EngagementController ctrl = make_controller(p);
    auto strategy = begin_with(ctrl, name);
    EXPECT_NEAR(ctrl.exit_master(), 0.0, 1e-9) << name;
  }
}

TEST(Engagement, IdleEmitsNominalStance) {
  EngagementController ctrl = make_controller();
  const auto nominal = nominal_stance();
  // No begin() call: state stays IDLE, output is nominal stance.
  const auto out = ctrl.update(0.02, {0.20, 0.0}, 0.0);
  for (const auto& name : LEG_NAMES) {
    EXPECT_DOUBLE_EQ(out.at(name).foot_target[0], nominal.at(name)[0]);
    EXPECT_DOUBLE_EQ(out.at(name).foot_target[1], nominal.at(name)[1]);
    EXPECT_DOUBLE_EQ(out.at(name).foot_target[2], nominal.at(name)[2]);
    EXPECT_TRUE(out.at(name).stance);
  }
}

// Port of the Python _BadStrategy: duty_factor mismatches the controller's, so
// begin() must reject it. Mirrors the SpyStrategy pattern in test_engine.cpp.
class BadStrategy : public Strategy {
 public:
  BadStrategy() : offsets_(tripod_offsets()) {}
  const PhaseOffsets& phase_offsets() const override { return offsets_; }
  double duty_factor() const override { return 0.5; }
  bool unstable() const override { return false; }
  Vec3 foot_target(double, const StrideParams&,
                   const LegContext&) const override {
    throw std::logic_error("not implemented");
  }

 private:
  PhaseOffsets offsets_;
};

TEST(Engagement, BeginRejectsStrategyDutyMismatch) {
  // Python raised ValueError -> std::invalid_argument here.
  ControllerParams p;
  p.duty_factor = 0.6;
  EngagementController ctrl = make_controller(p);
  BadStrategy bad;
  EXPECT_THROW(ctrl.begin(bad, leg_contexts()), std::invalid_argument);
}

// ── GAIT continuity at engagement end (parametrized in Python) ──────────────

TEST(Engagement, EngagementEndMatchesStrategyForConstantCmd) {
  // Every leg lands on its strategy-prescribed position by master = 1.0.
  const double v_cmd_x = 0.10;
  for (const auto& name : {"tripod", "crawl", "ripple"}) {
    auto proto = strategies().at(name)();
    ControllerParams p;
    p.duty_factor = proto->duty_factor();
    EngagementController ctrl = make_controller(p);
    auto strategy = begin_with(ctrl, name);
    const auto out = drive(ctrl, {v_cmd_x, 0.0}, 0.0, 0.02, 600);

    for (const auto& leg : LEG_NAMES) {
      const Vec3 expected =
          expected_gait_foot(*strategy, leg, 0.0, v_cmd_x);
      const Vec3 got = out.at(leg).foot_target;
      for (int axis = 0; axis < 3; ++axis) {
        EXPECT_NEAR(got[axis], expected[axis], 4e-3)
            << name << "/" << leg << " axis " << axis
            << " (strategy at master=0)";
      }
    }
  }
}

TEST(Engagement, NoPositionStepAtStateBoundaries) {
  // At every per-leg state boundary the foot position must be continuous.
  // Boundaries show up as stance-flag flips; bound the cross-flip step.
  const double v_cmd_x = 0.10;
  const double dt = 0.005;
  for (const auto& name : {"tripod", "crawl", "ripple"}) {
    auto proto = strategies().at(name)();
    ControllerParams p;
    p.duty_factor = proto->duty_factor();
    p.controller_dt = dt;
    EngagementController ctrl = make_controller(p);
    auto strategy = begin_with(ctrl, name);

    const double max_boundary_step = 2.0 * v_cmd_x * dt;
    std::map<std::string, Vec3> prev_targets;
    std::map<std::string, bool> prev_stance;
    bool have_prev = false;
    while (ctrl.state() != EngagementState::DONE) {
      const auto out = ctrl.update(dt, {v_cmd_x, 0.0}, 0.0);
      for (const auto& leg : LEG_NAMES) {
        const Vec3 pb = out.at(leg).foot_target;
        if (have_prev && prev_stance[leg] != out.at(leg).stance) {
          const Vec3 pa = prev_targets[leg];
          const double step = std::hypot(pb[0] - pa[0], pb[1] - pa[1]);
          EXPECT_LT(step, max_boundary_step)
              << name << "/" << leg << " stepped across stance flip at master="
              << ctrl.exit_master();
        }
        prev_targets[leg] = pb;
        prev_stance[leg] = out.at(leg).stance;
      }
      have_prev = true;
    }
  }
}

TEST(Engagement, EngagementReachesDone) {
  for (const auto& name : {"tripod", "crawl", "ripple"}) {
    auto proto = strategies().at(name)();
    ControllerParams p;
    p.duty_factor = proto->duty_factor();
    EngagementController ctrl = make_controller(p);
    auto strategy = begin_with(ctrl, name);
    drive(ctrl, {0.10, 0.0}, 0.0, 0.02, 600);
    EXPECT_EQ(ctrl.state(), EngagementState::DONE) << name;
  }
}

TEST(Engagement, RippleGaitLikeStanceWorldInvariantUnderVelocityStep) {
  // The GAIT_LIKE branch must integrate stance legs against the internal body
  // velocity. Drive far enough that several legs sit in GAIT_LIKE stance, then
  // step v_cmd; each such leg's world-frame foot must hold. Python guarded on
  // ctrl._master < 0.7; exit_master() equals it during engagement.
  auto proto = strategies().at("ripple")();
  ControllerParams p;
  p.duty_factor = proto->duty_factor();
  EngagementController ctrl = make_controller(p);
  auto strategy = begin_with(ctrl, "ripple");

  const double v_x = 0.10;
  const double dt = 0.005;
  double body_x = 0.0;
  double body_y = 0.0;
  std::map<std::string, LegOutput> out;
  bool have_out = false;
  while (ctrl.state() != EngagementState::DONE && ctrl.exit_master() < 0.7) {
    out = ctrl.update(dt, {v_x, 0.0}, 0.0);
    have_out = true;
    body_x += ctrl.v_body()[0] * dt;
    body_y += ctrl.v_body()[1] * dt;
  }
  ASSERT_TRUE(have_out);

  std::map<std::string, std::pair<double, double>> before;
  for (const auto& name : LEG_NAMES) {
    if (out.at(name).stance) {
      before[name] = {body_x + out.at(name).foot_target[0],
                      body_y + out.at(name).foot_target[1]};
    }
  }
  ASSERT_GE(before.size(), 3u);

  const std::pair<double, double> v_new = {0.05, 0.05};
  out = ctrl.update(dt, v_new, 0.0);
  body_x += ctrl.v_body()[0] * dt;
  body_y += ctrl.v_body()[1] * dt;

  for (const auto& [name, wbefore] : before) {
    if (!out.at(name).stance) {
      continue;
    }
    const double wx_after = body_x + out.at(name).foot_target[0];
    const double wy_after = body_y + out.at(name).foot_target[1];
    EXPECT_LT(std::abs(wx_after - wbefore.first), 1e-3) << name << " world dx";
    EXPECT_LT(std::abs(wy_after - wbefore.second), 1e-3) << name << " world dy";
  }
}

TEST(Engagement, LegsEnterStanceAfterFirstTouchdown) {
  // After each leg's first touchdown master it transitions to GAIT_LIKE, where
  // stance=True whenever phase >= swing_end. Python read ctrl._master;
  // exit_master() equals it during engagement.
  for (const auto& name : {"crawl", "ripple"}) {
    auto proto = strategies().at(name)();
    const double duty_factor = proto->duty_factor();
    ControllerParams p;
    p.duty_factor = duty_factor;
    EngagementController ctrl = make_controller(p);
    auto strategy = begin_with(ctrl, name);
    const auto& offsets = strategy->phase_offsets();

    const double v_cmd_x = 0.10;
    std::map<std::string, bool> saw_stance;
    for (const auto& n : LEG_NAMES) {
      saw_stance[n] = false;
    }
    while (ctrl.state() != EngagementState::DONE) {
      const auto out = ctrl.update(0.02, {v_cmd_x, 0.0}, 0.0);
      for (const auto& leg : LEG_NAMES) {
        const double phase = pymod(ctrl.exit_master() + offsets.at(leg), 1.0);
        if (phase >= 1.0 - duty_factor + 0.05 && out.at(leg).stance) {
          saw_stance[leg] = true;
        }
      }
    }
    for (const auto& leg : LEG_NAMES) {
      EXPECT_TRUE(saw_stance[leg])
          << name << ": leg never reported stance: " << leg;
    }
  }
}

}  // namespace
}  // namespace hexa_gait

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
