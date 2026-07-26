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

#include "config_generated.hpp"
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

struct SwingTrace {
  g::Vec3 start = g::Vec3::Zero();
  g::Vec3 end = g::Vec3::Zero();
  g::Vec3 touchdown_velocity = g::Vec3::Zero();  // m/s, real time
  g::Vec3 liftoff_velocity = g::Vec3::Zero();    // m/s, real time
  float apex_height = 0.0f;      // m above touchdown level
  float min_height = 0.0f;       // m above touchdown level
  float peak_descent = 0.0f;     // m/s downward
  float max_velocity_jump = 0.0f;  // m/s between adjacent samples
  // Worst downward speed over the part of the descent that is already within
  // `scrub_band` of the ground. This — not the speed at the touchdown point
  // itself — is what the foot actually lands at whenever the terrain or the
  // servo tracking is off by a millimetre or two, so it is the number that
  // decides whether a landing is soft.
  float near_ground_descent = 0.0f;
  // Largest sideways excursion from the straight PEP -> AEP line, in metres.
  float max_lateral = 0.0f;
  // How far the tip has slid against the ground, in metres: the worst gap
  // between where it is and where a planted foot would be, measured only while
  // it is at or below `scrub_band` and so could still be in contact.
  //
  // Deliberately a position comparison rather than a velocity one. Differencing
  // two ~0.12 m samples over a 0.2 ms step leaves float32 with about 1e-4 m/s of
  // pure cancellation noise, which is the same order as the effect under test.
  float max_ground_offset = 0.0f;
};

g::SwingProfile make_profile(float apex_fraction, float touchdown_velocity,
                             float width = 0.0f) {
  g::SwingProfile profile;
  profile.clearance = kStepHeight;
  profile.width = width;
  profile.apex_fraction = apex_fraction;
  profile.touchdown_velocity = touchdown_velocity;
  return profile;
}

// Walk the whole swing at a fine, uniform phase step and reduce it to the
// quantities the touchdown behaviour depends on. Velocities are finite
// differences in real time, so a genuine C1 break shows up as a single large
// max_velocity_jump while a smooth curve stays at O(acceleration * step).
SwingTrace profile_swing(float apex_fraction, float touchdown_velocity,
                         float width = 0.0f, float scrub_band = 0.002f) {
  const g::SwingProfile profile =
      make_profile(apex_fraction, touchdown_velocity, width);
  const g::Vec3 v_ground(-kLegSpeed, 0.0f, 0.0f);
  const auto at = [&](float phase) {
    return g::swing_arc(phase, kPep, kAep, 1, kSwingTime, profile, v_ground,
                        v_ground);
  };

  constexpr int kSteps = 2000;
  constexpr float kStep = 1.0f / static_cast<float>(kSteps);

  SwingTrace p;
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
    p.max_lateral = std::max(p.max_lateral, std::abs(point.y - kAep.y));
    if (phase > apex_fraction && height <= scrub_band) {
      p.near_ground_descent = std::max(p.near_ground_descent, -velocity.z);
    }
    if (i > 1) {
      const g::Vec3 jump = velocity - prev_velocity;
      p.max_velocity_jump = std::max(
          p.max_velocity_jump,
          std::sqrt(jump.x * jump.x + jump.y * jump.y + jump.z * jump.z));
    }
    if (height <= scrub_band) {
      // Where a foot planted at this end of the swing would be by now. The
      // lift-off end tracks forward from the PEP; the touchdown end tracks
      // backward from the AEP it is about to land on.
      const float tau = phase * kSwingTime;
      const g::Vec3 planted = phase < 0.5f
                                  ? kPep + v_ground * tau
                                  : kAep + v_ground * (tau - kSwingTime);
      const float dx = point.x - planted.x;
      const float dy = point.y - planted.y;
      p.max_ground_offset =
          std::max(p.max_ground_offset, std::sqrt(dx * dx + dy * dy));
    }
    if (i == 1) {
      p.liftoff_velocity = velocity;
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
  const g::SwingProfile profile{.clearance = 0.08f, .width = 0.0f};
  const g::Vec3 start = g::swing_arc(0.0f, pep, aep, 1, 0.4f, profile);
  const g::Vec3 apex = g::swing_arc(0.5f, pep, aep, 1, 0.4f, profile);
  const g::Vec3 end = g::swing_arc(1.0f, pep, aep, 1, 0.4f, profile);
  EXPECT_NEAR(start.x, pep.x, 1e-4f);
  EXPECT_NEAR(start.z, pep.z, 1e-4f);
  EXPECT_NEAR(end.x, aep.x, 1e-4f);
  EXPECT_NEAR(end.z, aep.z, 1e-4f);
  // Apex clears the ground by roughly step_height.
  EXPECT_GT(apex.z, pep.z + 0.05f);
}

TEST(Trajectory, SwingArcEndpointsHoldUnderApexSplit) {
  for (const float apex_fraction : {0.5f, 0.45f, 0.35f}) {
    const SwingTrace p = profile_swing(apex_fraction, 0.0f);
    EXPECT_NEAR(p.start.x, kPep.x, 1e-4f) << apex_fraction;
    EXPECT_NEAR(p.start.z, kPep.z, 1e-4f) << apex_fraction;
    EXPECT_NEAR(p.end.x, kAep.x, 1e-4f) << apex_fraction;
    EXPECT_NEAR(p.end.z, kAep.z, 1e-4f) << apex_fraction;
    EXPECT_GT(p.apex_height, 0.05f) << apex_fraction;
  }
}

// The lift profile is two halves of a quintic smoothstep joined at the apex.
// Both halves have zero slope there, so the join carries no velocity step
// however unevenly the split divides the swing.
TEST(Trajectory, SwingArcApexStaysVelocityContinuousUnderSplit) {
  for (const float apex_fraction : {0.5f, 0.45f, 0.35f}) {
    const SwingTrace p = profile_swing(apex_fraction, 0.0f);
    EXPECT_LT(p.max_velocity_jump, 0.05f)
        << "velocity discontinuity at apex_fraction " << apex_fraction;
  }
}

TEST(Trajectory, SwingArcHonoursTouchdownVelocity) {
  for (const float v_td : {0.0f, 0.02f, 0.05f}) {
    const SwingTrace p = profile_swing(0.45f, v_td);
    EXPECT_NEAR(p.touchdown_velocity.x, -kLegSpeed, 5e-3f) << v_td;
    EXPECT_NEAR(p.touchdown_velocity.z, -v_td, 5e-3f) << v_td;
  }
}

TEST(Trajectory, SwingArcNeverDipsBelowTouchdownLevel) {
  for (const float apex_fraction : {0.5f, 0.45f, 0.35f}) {
    for (const float v_td : {0.0f, 0.05f}) {
      const SwingTrace p = profile_swing(apex_fraction, v_td);
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
  // Pins the closed form: a quintic descent of `step_height` over
  // (1 - apex_fraction) * swing_time peaks at 1.875 * height / duration.
  EXPECT_NEAR(even, 3.75f * kStepHeight / kSwingTime, 0.02f);
  EXPECT_NEAR(longer, 1.875f * kStepHeight / (0.65f * kSwingTime), 0.02f);
}

// ── Ground track and touchdown ──

// The swing is a blend between the two ground lines, weighted by a septic ease
// whose first three derivatives vanish at both ends. So the foot leaves and
// meets the ground travelling with the ground, and pulls away from it only as
// the fourth power of the elapsed swing — by the time it is a couple of
// millimetres up it has barely moved against the ground it may still be
// touching. That is the scrub seen on the rear feet.
TEST(Trajectory, SwingLeavesAndMeetsTheGroundAlongTheGroundTrack) {
  constexpr float kBand = 0.002f;
  const SwingTrace p = profile_swing(0.5f, 0.01f, 0.0f, kBand);

  // Exact at the seams: no horizontal step into or out of stance.
  EXPECT_NEAR(p.liftoff_velocity.x, -kLegSpeed, 1e-3f);
  EXPECT_NEAR(p.liftoff_velocity.y, 0.0f, 1e-3f);
  EXPECT_NEAR(p.touchdown_velocity.x, -kLegSpeed, 1e-3f);
  EXPECT_NEAR(p.touchdown_velocity.y, 0.0f, 1e-3f);

  // And sub-millimetre over the whole stretch where the foot is low enough to
  // still be in contact.
  EXPECT_LT(p.max_ground_offset, 1e-3f);
}

// step_height means "how high the foot lifts off the ground", and both ends of
// the curve land exactly where they were asked to.
TEST(Trajectory, SwingPreservesEndpointsApexAndGroundLevel) {
  for (const float apex_fraction : {0.5f, 0.4f}) {
    const SwingTrace p = profile_swing(apex_fraction, 0.01f);
    EXPECT_NEAR(p.start.x, kPep.x, 1e-4f) << apex_fraction;
    EXPECT_NEAR(p.start.z, kPep.z, 1e-4f) << apex_fraction;
    EXPECT_NEAR(p.end.x, kAep.x, 1e-4f) << apex_fraction;
    EXPECT_NEAR(p.end.z, kAep.z, 1e-4f) << apex_fraction;
    EXPECT_NEAR(p.apex_height, kStepHeight, 2e-3f) << apex_fraction;
    // The foot never digs below the ground it is stepping onto.
    EXPECT_GT(p.min_height, -1e-5f) << apex_fraction;
  }
}

// The regression this shape exists to prevent.
//
// The earlier construction wrapped the arc in a ground-matched touchdown ramp
// that had to cross a fixed band — a tenth of the step height — inside a fixed
// 6% of the swing. That met the ground at exactly touchdown_velocity, but only
// in the limit at ground level: it was still doing better than 0.4 m/s a
// couple of millimetres up, so any terrain bump or servo lag inside that band
// turned into a hard landing. Braking over the whole descent instead keeps the
// approach slow where it matters, not just at the single point it is measured.
TEST(Trajectory, DescentIsAlreadySlowBeforeItReachesTheGround) {
  constexpr float kBand = 0.002f;
  const SwingTrace even = profile_swing(0.5f, 0.01f, 0.0f, kBand);
  EXPECT_LT(even.near_ground_descent, 0.25f);

  // And handing the descent more of the swing slows it further still.
  const SwingTrace longer = profile_swing(0.35f, 0.01f, 0.0f, kBand);
  EXPECT_LT(longer.near_ground_descent, even.near_ground_descent);
}

// The lateral arch peaks at swing_width and closes back to the straight
// PEP -> AEP line at both ends, so it cannot push the seam with stance sideways.
TEST(Trajectory, SwingWidthArchesAndCloses) {
  constexpr float kWidth = 0.02f;
  const SwingTrace p = profile_swing(0.5f, 0.01f, kWidth);
  EXPECT_NEAR(p.max_lateral, kWidth, 1e-4f);
  EXPECT_NEAR(p.start.y, kPep.y, 1e-5f);
  EXPECT_NEAR(p.end.y, kAep.y, 1e-5f);
  EXPECT_NEAR(p.liftoff_velocity.y, 0.0f, 1e-3f);
  EXPECT_NEAR(p.touchdown_velocity.y, 0.0f, 1e-3f);
}

// The arch does not depend on there being any lift: the pause descent builds a
// profile with zero clearance and a non-zero width.
TEST(Trajectory, SwingWidthSurvivesZeroClearance) {
  g::SwingProfile profile;
  profile.clearance = 0.0f;
  profile.width = 0.02f;
  const g::Vec3 apex = g::swing_arc(0.5f, kPep, kAep, 1, kSwingTime, profile,
                                    g::Vec3::Zero(), g::Vec3::Zero());
  EXPECT_NEAR(apex.y - kAep.y, 0.02f, 1e-5f);
  EXPECT_NEAR(apex.z, kAep.z, 1e-5f);
}

// The foot has to complete its step in the time it has, so anything that steals
// swing time or ground from the arc shows up as a whip at mid-swing — on the
// robot, a twitch, and at longer strides a foot that outruns its own touchdown.
// Nothing in the curve steals either any more, so the peak tip speed has to sit
// close to the analytical floor: the ground-relative displacement swept by the
// septic ease, whose peak slope is 35/16.
TEST(Engine, PeakTipSpeedStaysCloseToTheAnalyticalFloor) {
  constexpr float kTickDt = 0.005f;

  for (const float min_swing : {0.4f, 0.6f}) {
    g::EngineConfig cfg = g::engine_config_from_config();
    cfg.min_swing_time = min_swing;
    cfg.max_swing_time = min_swing * 1.33f;
    const float swing_end = g::swing_end_phase(0.5f, cfg.swing_phase_margin);
    // Saturating command: the worst case for the arc.
    const float v_cmd = cfg.stride_length * swing_end /
                        (cfg.min_swing_time * (1.0f - swing_end));

    auto e = g::make_default_engine(
        "tripod", hexa::config::kLegSpecs, cfg, hexa::config::kStandingPose,
        hexa::config::kInitialPose, hexa::config::kCoxaToBottom);
    e->start_initialize();
    for (int i = 0; i < 6000 && e->state() != g::EngineState::STAND; ++i) {
      e->update(kTickDt, {0.0f, 0.0f}, 0.0f);
    }
    for (int i = 0; i < 6000 && e->state() != g::EngineState::GAIT; ++i) {
      e->update(kTickDt, {v_cmd, 0.0f}, 0.0f);
    }
    ASSERT_EQ(e->state(), g::EngineState::GAIT) << min_swing;

    g::Vec3 prev = g::Vec3::Zero();
    bool have = false;
    float peak = 0.0f;
    for (int i = 0; i < 900; ++i) {
      const auto out = e->update(kTickDt, {v_cmd, 0.0f}, 0.0f);
      const auto& lo = out.at("l_front");
      if (have && !lo.stance) {
        const g::Vec3 v = (lo.foot_target - prev) / kTickDt;
        peak = std::max(peak, std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z));
      }
      prev = lo.foot_target;
      have = true;
    }

    // Ground-relative, the foot covers its own stride plus the ground the body
    // travels while it is airborne. Against the body it covers that swept at
    // 35/16 at the fastest, less the ground speed it is swimming against.
    const float ground = cfg.stride_length * swing_end / (1.0f - swing_end);
    const float expected =
        (2.1875f * (cfg.stride_length + ground) - ground) / min_swing;
    EXPECT_GT(peak, 0.85f * expected) << min_swing;
    EXPECT_LT(peak, 1.15f * expected) << min_swing;
  }
}

// ── Swing-window margin ──

TEST(Registry, SwingWindowShrinksByMarginForEveryGait) {
  for (const char* name : {"tripod", "surf", "tetrapod", "crawl", "ripple"}) {
    const float beta = g::strategies().at(name)()->duty_factor();
    const float nominal = 1.0f - beta;
    EXPECT_NEAR(g::swing_end_phase(beta, 0.0f), nominal, 1e-6f) << name;
    EXPECT_NEAR(g::swing_end_phase(beta, 0.12f), nominal * 0.88f, 1e-6f) << name;
    // Clamped, so a bad edit can never collapse the window or invert it.
    EXPECT_NEAR(g::swing_end_phase(beta, 5.0f), nominal * 0.6f, 1e-6f) << name;
    EXPECT_NEAR(g::swing_end_phase(beta, -1.0f), nominal, 1e-6f) << name;
  }
  // Tripod is the headline case: swing [0, 0.44), stance [0.44, 1).
  EXPECT_NEAR(g::swing_end_phase(0.5f, 0.12f), 0.44f, 1e-6f);
}

namespace {

// Walk `gait` at a steady command for a few cycles and record, per tick, how
// many feet are on the ground.
std::map<int, int> stance_count_histogram(const char* gait, int ticks) {
  constexpr float kTickDt = 0.005f;
  auto e = g::make_default_engine(gait);
  run_to_stand(*e);
  for (int i = 0; i < 2000 && e->state() != g::EngineState::GAIT; ++i) {
    e->update(kTickDt, {0.05f, 0.0f}, 0.0f);
  }
  EXPECT_EQ(e->state(), g::EngineState::GAIT) << gait;

  std::map<int, int> histogram;
  for (int i = 0; i < ticks; ++i) {
    const auto out = e->update(kTickDt, {0.05f, 0.0f}, 0.0f);
    int stance = 0;
    for (const auto& [name, leg] : out) {
      (void)name;
      if (leg.stance) ++stance;
    }
    ++histogram[stance];
  }
  return histogram;
}

}  // namespace

// The headline behaviour: at every handover the incoming legs are already
// carrying weight before the outgoing ones let go. Without the margin the swap
// is a knife edge — the gaits below momentarily drop to 3, 4 and 2 stance legs
// respectively at the instant of the swap.
TEST(Engine, EveryHandoverHasAStretchWithAllSixFeetDown) {
  struct Case {
    const char* gait;
    int min_stance;  // floor the margin must guarantee
  };
  for (const Case& c : {Case{"tripod", 3}, Case{"tetrapod", 4},
                        Case{"ripple", 5}}) {
    const auto histogram = stance_count_histogram(c.gait, 1200);

    int total = 0;
    int lowest = 6;
    for (const auto& [stance, ticks] : histogram) {
      total += ticks;
      lowest = std::min(lowest, stance);
    }
    ASSERT_GT(total, 0) << c.gait;

    EXPECT_GE(lowest, c.min_stance) << c.gait;
    EXPECT_GE(lowest, 3) << c.gait << " is statically unstable";

    const auto six = histogram.find(6);
    ASSERT_NE(six, histogram.end())
        << c.gait << " never has all six feet planted";
    // The overlap is a real stretch of the cycle, not a single tick that could
    // be swallowed by timing jitter.
    const float fraction =
        static_cast<float>(six->second) / static_cast<float>(total);
    EXPECT_GT(fraction, 0.04f) << c.gait;
  }
}

// The margin shortens the swing window, so the cycle has to stretch to keep the
// foot in the air for its configured swing time. If cycle_time_bounds still
// scaled by the nominal 1/(1 - beta) the feet would be rushed through the air by
// exactly the margin.
TEST(Engine, SwingTimeStaysWithinConfiguredBoundsUnderTheMargin) {
  constexpr float kTickDt = 0.005f;
  const auto cfg = g::engine_config_from_config();
  for (const char* gait : {"tripod", "tetrapod", "ripple"}) {
    for (const float v_x : {0.02f, 0.05f, 0.10f}) {
      auto e = g::make_default_engine(gait);
      run_to_stand(*e);
      for (int i = 0; i < 3000 && e->state() != g::EngineState::GAIT; ++i) {
        e->update(kTickDt, {v_x, 0.0f}, 0.0f);
      }
      ASSERT_EQ(e->state(), g::EngineState::GAIT) << gait << " @ " << v_x;

      // Count the ticks one leg spends airborne over one complete swing.
      int swing_ticks = 0;
      int best = 0;
      bool seen_stance = false;
      for (int i = 0; i < 2000; ++i) {
        const auto out = e->update(kTickDt, {v_x, 0.0f}, 0.0f);
        const bool stance = out.at("l_front").stance;
        seen_stance = seen_stance || stance;
        if (!stance && seen_stance) {
          ++swing_ticks;
        } else if (stance && swing_ticks > 0) {
          best = std::max(best, swing_ticks);
          swing_ticks = 0;
        }
      }
      ASSERT_GT(best, 0) << gait << " @ " << v_x;
      const float swing_time = static_cast<float>(best) * kTickDt;
      // One tick of slack at each end for the phase quantisation.
      EXPECT_GE(swing_time, cfg.min_swing_time - 2.0f * kTickDt)
          << gait << " @ " << v_x;
      EXPECT_LE(swing_time, cfg.max_swing_time + 2.0f * kTickDt)
          << gait << " @ " << v_x;
    }
  }
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
