// Behavioural unit tests for the float gait port (plan part 06).
//
// Float-only (no double reference): exercises the ported clock, trajectory,
// strategies, and the engine state machine directly. Built through
// hexa_host_test() so it compiles under -Wdouble-promotion — the same gate the
// firmware build applies to the port sources.

#include <algorithm>
#include <cmath>
#include <map>
#include <string>

#include <gtest/gtest.h>

#include "gait/clock.hpp"
#include "gait/engine.hpp"
#include "gait/gaits/registry.hpp"
#include "gait/limits.hpp"
#include "gait/trajectory.hpp"

namespace g = hexa::gait;

namespace {

constexpr float kDt = 0.02f;

// Drive the engine cold-start ladder to STAND (cmd_vel held at zero).
void run_to_stand(g::Engine& e) {
  ASSERT_TRUE(e.start_initialize());
  for (int i = 0; i < 200 && e.state() != g::EngineState::STAND; ++i) {
    e.update(kDt, {0.0f, 0.0f}, 0.0f);
  }
  ASSERT_EQ(e.state(), g::EngineState::STAND);
}

// ── Swing-arc sampling ──

constexpr float kSwingTime = 0.35f;
constexpr float kStepHeight = 0.08f;
constexpr float kLegSpeed = 0.06f;
const g::Vec3 kPep(0.15f, 0.05f, -0.12f);
const g::Vec3 kAep(0.20f, 0.05f, -0.12f);

struct SwingProfile {
  g::Vec3 start = g::Vec3::Zero();
  g::Vec3 end = g::Vec3::Zero();
  g::Vec3 touchdown_velocity = g::Vec3::Zero();  // m/s, real time
  float apex_height = 0.0f;      // m above touchdown level
  float min_height = 0.0f;       // m above touchdown level
  float peak_descent = 0.0f;     // m/s downward
  float max_velocity_jump = 0.0f;  // m/s between adjacent samples
};

// Walk the whole swing at a fine, uniform phase step and reduce it to the
// quantities the touchdown behaviour depends on. Velocities are finite
// differences in real time, so a genuine C1 break shows up as a single large
// max_velocity_jump while a smooth curve stays at O(acceleration * step).
SwingProfile profile_swing(float apex_fraction, float touchdown_velocity) {
  const g::Vec3 v_in(-kLegSpeed, 0.0f, 0.0f);
  const g::Vec3 v_out(-kLegSpeed, 0.0f, -touchdown_velocity);
  const auto at = [&](float phase) {
    return g::swing_arc(phase, kPep, kAep, kStepHeight, 0.0f, 1, kSwingTime,
                        v_in, v_out, apex_fraction);
  };

  constexpr int kSteps = 2000;
  constexpr float kStep = 1.0f / static_cast<float>(kSteps);

  SwingProfile p;
  p.start = at(0.0f);
  p.end = at(1.0f);
  p.min_height = 1.0f;

  g::Vec3 prev_point = p.start;
  g::Vec3 prev_velocity = g::Vec3::Zero();
  for (int i = 1; i <= kSteps; ++i) {
    const float phase = static_cast<float>(i) * kStep;
    const g::Vec3 point = at(phase);
    const g::Vec3 velocity = (point - prev_point) / (kStep * kSwingTime);

    const float height = point.z - kAep.z;
    p.apex_height = std::max(p.apex_height, height);
    p.min_height = std::min(p.min_height, height);
    p.peak_descent = std::max(p.peak_descent, -velocity.z);
    if (i > 1) {
      const g::Vec3 jump = velocity - prev_velocity;
      p.max_velocity_jump = std::max(
          p.max_velocity_jump,
          std::sqrt(jump.x * jump.x + jump.y * jump.y + jump.z * jump.z));
    }
    p.touchdown_velocity = velocity;
    prev_point = point;
    prev_velocity = velocity;
  }
  return p;
}

}  // namespace

TEST(Clock, AdvanceWrapsAndProjectsOffsets) {
  g::PhaseOffsets offsets({
      {"l_front", 0.0f},
      {"r_middle", 0.0f},
      {"l_rear", 0.0f},
      {"r_front", 0.5f},
      {"l_middle", 0.5f},
      {"r_rear", 0.5f},
  });
  g::GaitClock gc(offsets);
  gc.advance(0.25f, 1.0f);  // master -> 0.25
  EXPECT_NEAR(gc.master(), 0.25f, 1e-6f);
  const auto phases = gc.phases();
  EXPECT_NEAR(phases.at("l_front"), 0.25f, 1e-6f);
  EXPECT_NEAR(phases.at("r_front"), 0.75f, 1e-6f);  // +0.5 offset
  gc.advance(0.5f, 1.0f);                           // master -> 0.75
  EXPECT_NEAR(gc.phases().at("r_front"), 0.25f, 1e-6f);  // wrapped
}

TEST(Registry, TripodOffsetsAreAntiphase) {
  const auto strat = g::strategies().at("tripod")();
  EXPECT_FLOAT_EQ(strat->duty_factor(), 0.5f);
  const auto& off = strat->phase_offsets();
  for (const char* leg : {"l_front", "r_middle", "l_rear"}) {
    EXPECT_FLOAT_EQ(off.at(leg), 0.0f) << leg;
  }
  for (const char* leg : {"r_front", "l_middle", "r_rear"}) {
    EXPECT_FLOAT_EQ(off.at(leg), 0.5f) << leg;
  }
}

TEST(Registry, AllGaitsPresent) {
  for (const char* name : {"tripod", "surf", "tetrapod", "crawl", "ripple"}) {
    EXPECT_NE(g::strategies().find(name), g::strategies().end()) << name;
  }
}

TEST(Trajectory, QuarticBezierHitsEndpoints) {
  g::BezierNodes nodes = {g::Vec3(0, 0, 0), g::Vec3(1, 0, 0), g::Vec3(2, 1, 0),
                          g::Vec3(3, 1, 0), g::Vec3(4, 0, 0)};
  const g::Vec3 p0 = g::quartic_bezier(nodes, 0.0f);
  const g::Vec3 p1 = g::quartic_bezier(nodes, 1.0f);
  EXPECT_NEAR(p0.x, 0.0f, 1e-6f);
  EXPECT_NEAR(p1.x, 4.0f, 1e-6f);
  EXPECT_NEAR(p1.y, 0.0f, 1e-6f);
}

TEST(Trajectory, SwingArcReturnsToGroundAtTouchdown) {
  const g::Vec3 pep(0.15f, 0.05f, -0.12f);
  const g::Vec3 aep(0.20f, 0.05f, -0.12f);
  const g::Vec3 start = g::swing_arc(0.0f, pep, aep, 0.08f, 0.0f, 1, 0.4f);
  const g::Vec3 apex = g::swing_arc(0.5f, pep, aep, 0.08f, 0.0f, 1, 0.4f);
  const g::Vec3 end = g::swing_arc(1.0f, pep, aep, 0.08f, 0.0f, 1, 0.4f);
  EXPECT_NEAR(start.x, pep.x, 1e-4f);
  EXPECT_NEAR(start.z, pep.z, 1e-4f);
  EXPECT_NEAR(end.x, aep.x, 1e-4f);
  EXPECT_NEAR(end.z, aep.z, 1e-4f);
  // Apex clears the ground by roughly step_height.
  EXPECT_GT(apex.z, pep.z + 0.05f);
}

TEST(Trajectory, SwingArcEndpointsHoldUnderApexSplit) {
  for (const float apex_fraction : {0.5f, 0.45f, 0.35f}) {
    const SwingProfile p = profile_swing(apex_fraction, 0.0f);
    EXPECT_NEAR(p.start.x, kPep.x, 1e-4f) << apex_fraction;
    EXPECT_NEAR(p.start.z, kPep.z, 1e-4f) << apex_fraction;
    EXPECT_NEAR(p.end.x, kAep.x, 1e-4f) << apex_fraction;
    EXPECT_NEAR(p.end.z, kAep.z, 1e-4f) << apex_fraction;
    EXPECT_GT(p.apex_height, 0.05f) << apex_fraction;
  }
}

// The two swing halves are separate quartics joined at the apex. When they span
// different durations the apex tangent has to be scaled by the duration ratio;
// without that scaling the foot's velocity steps discontinuously mid-swing.
TEST(Trajectory, SwingArcApexStaysVelocityContinuousUnderSplit) {
  for (const float apex_fraction : {0.5f, 0.45f, 0.35f}) {
    const SwingProfile p = profile_swing(apex_fraction, 0.0f);
    EXPECT_LT(p.max_velocity_jump, 0.05f)
        << "velocity discontinuity at apex_fraction " << apex_fraction;
  }
}

TEST(Trajectory, SwingArcHonoursTouchdownVelocity) {
  for (const float v_td : {0.0f, 0.02f, 0.05f}) {
    const SwingProfile p = profile_swing(0.45f, v_td);
    EXPECT_NEAR(p.touchdown_velocity.x, -kLegSpeed, 5e-3f) << v_td;
    EXPECT_NEAR(p.touchdown_velocity.z, -v_td, 5e-3f) << v_td;
  }
}

TEST(Trajectory, SwingArcNeverDipsBelowTouchdownLevel) {
  for (const float apex_fraction : {0.5f, 0.45f, 0.35f}) {
    for (const float v_td : {0.0f, 0.05f}) {
      const SwingProfile p = profile_swing(apex_fraction, v_td);
      EXPECT_GT(p.min_height, -1.0e-5f) << apex_fraction << " / " << v_td;
    }
  }
}

// The point of the split: giving the descent a larger share of the swing
// stretches its curve, so the foot comes down more slowly for the same height.
TEST(Trajectory, LongerDescentLowersPeakDescentRate) {
  const float even = profile_swing(0.5f, 0.0f).peak_descent;
  const float split = profile_swing(0.45f, 0.0f).peak_descent;
  const float longer = profile_swing(0.35f, 0.0f).peak_descent;
  EXPECT_LT(split, even);
  EXPECT_LT(longer, split);
  // Pins the even-split closed form, (32/9) * step_height / swing_time.
  EXPECT_NEAR(even, 3.5556f * kStepHeight / kSwingTime, 0.02f);
}

TEST(Engine, ColdStartReachesStand) {
  auto e = g::make_default_engine("tripod");
  EXPECT_EQ(e->state(), g::EngineState::FOLDED);
  run_to_stand(*e);
}

TEST(Engine, EnterFaultFromAnyStateHoldsFoldedPose) {
  // A fault can strike mid-walk: enter_fault must latch FAULT from GAIT and emit
  // the folded initial stance (servos limp on the real board).
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  for (int i = 0; i < 200 && e->state() != g::EngineState::GAIT; ++i) {
    e->update(kDt, {0.06f, 0.0f}, 0.0f);
  }
  ASSERT_EQ(e->state(), g::EngineState::GAIT);

  e->enter_fault();
  EXPECT_EQ(e->state(), g::EngineState::FAULT);

  // The folded baseline (initial stance) matches a cold FOLDED engine's output.
  auto folded = g::make_default_engine("tripod");
  const auto faulted_out = e->update(kDt, {0.0f, 0.0f}, 0.0f);
  const auto folded_out = folded->update(kDt, {0.0f, 0.0f}, 0.0f);
  for (const auto& [name, lo] : folded_out) {
    EXPECT_FLOAT_EQ(faulted_out.at(name).foot_target.x, lo.foot_target.x) << name;
    EXPECT_FLOAT_EQ(faulted_out.at(name).foot_target.y, lo.foot_target.y) << name;
    EXPECT_FLOAT_EQ(faulted_out.at(name).foot_target.z, lo.foot_target.z) << name;
    EXPECT_TRUE(faulted_out.at(name).stance) << name;
  }
}

TEST(Engine, FaultRecoversViaInitializeLadder) {
  // Recovery is byte-for-byte the startup + initialize path: start_initialize()
  // is valid from FAULT and runs the same ladder to STAND.
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  e->enter_fault();
  ASSERT_EQ(e->state(), g::EngineState::FAULT);

  ASSERT_TRUE(e->start_initialize());  // rejected from most states; allowed here
  EXPECT_EQ(e->state(), g::EngineState::INITIALIZE);
  for (int i = 0; i < 200 && e->state() != g::EngineState::STAND; ++i) {
    e->update(kDt, {0.0f, 0.0f}, 0.0f);
  }
  EXPECT_EQ(e->state(), g::EngineState::STAND);
}

TEST(Engine, StartInitializeRejectedWhileStood) {
  // Guard sanity: start_initialize only fires from FOLDED or FAULT, not STAND.
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  EXPECT_FALSE(e->start_initialize());
  EXPECT_EQ(e->state(), g::EngineState::STAND);
}

TEST(Engine, ForwardCommandWalksTripodInAntiphase) {
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);

  // Command a modest forward velocity and let the engagement complete.
  bool reached_gait = false;
  for (int i = 0; i < 400; ++i) {
    e->update(kDt, {0.06f, 0.0f}, 0.0f);
    if (e->state() == g::EngineState::GAIT) {
      reached_gait = true;
      break;
    }
  }
  ASSERT_TRUE(reached_gait);

  // Over a full cycle, tripod A and tripod B must each spend time in swing, and
  // never be airborne simultaneously (the defining tripod property).
  bool a_swung = false;
  bool b_swung = false;
  for (int i = 0; i < 60; ++i) {
    const auto out = e->update(kDt, {0.06f, 0.0f}, 0.0f);
    const bool a_swing = !out.at("l_front").stance;
    const bool b_swing = !out.at("r_front").stance;
    a_swung = a_swung || a_swing;
    b_swung = b_swung || b_swing;
    EXPECT_FALSE(a_swing && b_swing) << "both tripods airborne at tick " << i;
  }
  EXPECT_TRUE(a_swung);
  EXPECT_TRUE(b_swung);
}

// A swing is planned from the foot's lift-off state but lands into a stance that
// integrates the *live* command. If the touchdown end of the swing were latched
// at lift-off, a command that moved during the swing would leave the foot
// arriving at one velocity and being dragged at another the very next tick.
TEST(Engine, TouchdownStaysVelocityContinuousWhileCommandRamps) {
  constexpr float kTickDt = 0.005f;
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);

  float v_x = 0.06f;
  for (int i = 0; i < 800 && e->state() != g::EngineState::GAIT; ++i) {
    e->update(kTickDt, {v_x, 0.0f}, 0.0f);
  }
  ASSERT_EQ(e->state(), g::EngineState::GAIT);

  // Ramp the command hard enough to shift the touchdown point by centimetres
  // over a single swing, then watch every swing -> stance handover.
  struct LegTrace {
    g::Vec3 target = g::Vec3::Zero();
    g::Vec3 last_swing_velocity = g::Vec3::Zero();
    bool have_target = false;
    bool have_swing_velocity = false;
    int swing_streak = 0;
    int stance_streak = 0;
  };
  std::map<std::string, LegTrace> trace;
  int touchdowns = 0;
  float worst_jump = 0.0f;
  std::string worst_leg;

  for (int i = 0; i < 600; ++i) {
    v_x = std::min(v_x + 0.0008f, 0.12f);
    const auto out = e->update(kTickDt, {v_x, 0.0f}, 0.0f);
    for (const auto& [name, leg] : out) {
      LegTrace& t = trace[name];
      const g::Vec3 velocity =
          t.have_target ? (leg.foot_target - t.target) / kTickDt
                        : g::Vec3::Zero();
      t.swing_streak = leg.stance ? 0 : t.swing_streak + 1;
      t.stance_streak = leg.stance ? t.stance_streak + 1 : 0;

      // A velocity sample only means something when both of its endpoints lie
      // on the same side of the seam, hence the two-consecutive-tick guards.
      if (!leg.stance && t.swing_streak >= 2 && t.have_target) {
        t.last_swing_velocity = velocity;
        t.have_swing_velocity = true;
      }
      // Second stance tick, not the first: the first straddles the seam and
      // carries the sub-millimetre snap onto the AEP, which is a sampling
      // artefact rather than a velocity mismatch. Comparing clean swing and
      // stance samples isolates what a latched touchdown target would break.
      // Z is excluded on purpose — stance holds z fixed, so touchdown_velocity
      // is a deliberate vertical step.
      if (leg.stance && t.stance_streak == 2 && t.have_swing_velocity) {
        const g::Vec3 jump = velocity - t.last_swing_velocity;
        const float magnitude = std::sqrt(jump.x * jump.x + jump.y * jump.y);
        ++touchdowns;
        if (magnitude > worst_jump) {
          worst_jump = magnitude;
          worst_leg = name;
        }
      }
      t.target = leg.foot_target;
      t.have_target = true;
    }
  }

  ASSERT_GE(touchdowns, 6) << "too few touchdowns observed to be meaningful";
  EXPECT_LT(worst_jump, 0.01f)
      << "foot velocity steps at touchdown on " << worst_leg;
}

TEST(Engine, ZeroCommandPausesAndReseatsToStand) {
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  for (int i = 0; i < 200 && e->state() != g::EngineState::GAIT; ++i) {
    e->update(kDt, {0.06f, 0.0f}, 0.0f);
  }
  ASSERT_EQ(e->state(), g::EngineState::GAIT);

  // Drop the command: engine debounces, pauses, then reseats back to STAND.
  bool returned = false;
  for (int i = 0; i < 400; ++i) {
    e->update(kDt, {0.0f, 0.0f}, 0.0f);
    if (e->state() == g::EngineState::STAND) {
      returned = true;
      break;
    }
  }
  EXPECT_TRUE(returned);
}

TEST(Engine, SetStrategyDefersWhileWalking) {
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  for (int i = 0; i < 200 && e->state() != g::EngineState::GAIT; ++i) {
    e->update(kDt, {0.06f, 0.0f}, 0.0f);
  }
  ASSERT_EQ(e->state(), g::EngineState::GAIT);

  EXPECT_TRUE(e->set_strategy("tetrapod"));
  EXPECT_EQ(e->pending_strategy_name().value_or(""), "tetrapod");

  // Keep commanding forward; the engine pauses, reseats, applies the change,
  // and re-engages under the new strategy.
  bool applied = false;
  for (int i = 0; i < 600; ++i) {
    e->update(kDt, {0.06f, 0.0f}, 0.0f);
    if (e->strategy_name() == "tetrapod") {
      applied = true;
      break;
    }
  }
  EXPECT_TRUE(applied);
  EXPECT_FLOAT_EQ(e->strategy_name() == "tetrapod" ? 2.0f / 3.0f : 0.0f,
                  2.0f / 3.0f);
}

TEST(Limits, ScaleToEnvelopeIsNoOpWhenInRange) {
  const auto caps = g::load_velocity_caps_from_config();
  EXPECT_GT(caps.linear_max("tripod"), 0.0f);
  std::map<std::string, g::Vec3> mounts;
  for (int i = 0; i < 6; ++i) {
    mounts[g::LEG_NAMES[i]] = g::Vec3(0.08f, 0.05f, 0.0f);
  }
  auto [vx, vy, wz] = g::scale_to_envelope(0.01f, 0.0f, 0.0f, mounts,
                                           caps.linear_max("tripod"),
                                           caps.angular_max, 0.6f);
  EXPECT_NEAR(vx, 0.01f, 1e-6f);
  EXPECT_NEAR(vy, 0.0f, 1e-6f);
  EXPECT_NEAR(wz, 0.0f, 1e-6f);
}
