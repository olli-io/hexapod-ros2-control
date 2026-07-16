// Behavioural unit tests for the float gait port (plan part 06).
//
// Float-only (no double reference): exercises the ported clock, trajectory,
// strategies, and the engine state machine directly. Built through
// hexa_host_test() so it compiles under -Wdouble-promotion — the same gate the
// firmware build applies to the port sources.

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
  const g::Vec3 start = g::swing_arc(0.0f, pep, aep, 0.08f, 0.0f, 1, 0.4f, kDt);
  const g::Vec3 apex = g::swing_arc(0.5f, pep, aep, 0.08f, 0.0f, 1, 0.4f, kDt);
  const g::Vec3 end = g::swing_arc(1.0f, pep, aep, 0.08f, 0.0f, 1, 0.4f, kDt);
  EXPECT_NEAR(start.x, pep.x, 1e-4f);
  EXPECT_NEAR(start.z, pep.z, 1e-4f);
  EXPECT_NEAR(end.x, aep.x, 1e-4f);
  EXPECT_NEAR(end.z, aep.z, 1e-4f);
  // Apex clears the ground by roughly step_height.
  EXPECT_GT(apex.z, pep.z + 0.05f);
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
