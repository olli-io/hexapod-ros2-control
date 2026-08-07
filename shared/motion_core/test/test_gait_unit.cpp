// Behavioural unit tests for the float gait port (plan part 06).
//
// Float-only (no double reference): exercises the ported clock, trajectory,
// strategies, and the engine state machine directly. Built through
// hexa_host_test() so it compiles under -Wdouble-promotion — the same gate the
// firmware build applies to the port sources.

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include <gtest/gtest.h>

#include "config_generated.hpp"
#include "gait/clock.hpp"
#include "gait/engine.hpp"
#include "gait/gaits/registry.hpp"
#include "gait/limits.hpp"
#include "gait/trajectory.hpp"
#include "kinematics/body_transform.hpp"
#include "kinematics/leg_ik.hpp"

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
  // How tall the gentle part of the landing is, in metres: the height of the
  // highest point on the descent from which the foot is already coming down no
  // faster than `gentle_rate`, with nothing faster below it.
  //
  // This is the number the whole shape exists to make large. The servos resolve
  // the foot's height to a couple of tenths of a millimetre, so contact happens
  // anywhere inside that much of the intended touchdown point. Unless the band
  // that lands softly is taller than the error, the softness is never the thing
  // that actually happens.
  float gentle_band = 0.0f;
  // Height (m) the foot still has left when it has covered 90% of its forward
  // travel. A swing is meant to put the foot down onto its touchdown point
  // from above; if this collapses, the foot instead reaches its final height
  // early and sweeps the rest of the way in at ankle level, catching on
  // whatever the step was supposed to clear.
  float height_at_90pct_forward = 0.0f;
  // Largest rise, in metres, anywhere after the apex. The descent is meant to
  // be monotonic — a bump in it would show up as the foot lifting again on
  // the way down.
  float descent_rise = 0.0f;
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
                             float width = 0.0f, float probe_height = 0.0f,
                             float liftoff_velocity = 0.0f) {
  g::SwingProfile profile;
  profile.clearance = kStepHeight;
  profile.width = width;
  profile.apex_fraction = apex_fraction;
  profile.touchdown_velocity = touchdown_velocity;
  profile.touchdown_probe_height = probe_height;
  profile.liftoff_velocity = liftoff_velocity;
  return profile;
}

// Walk the whole swing at a fine, uniform phase step and reduce it to the
// quantities the touchdown behaviour depends on. Velocities are finite
// differences in real time, so a genuine C1 break shows up as a single large
// max_velocity_jump while a smooth curve stays at O(acceleration * step).
SwingTrace profile_swing(float apex_fraction, float touchdown_velocity,
                         float width = 0.0f, float scrub_band = 0.002f,
                         float probe_height = 0.0f,
                         float liftoff_velocity = 0.0f) {
  const g::SwingProfile profile = make_profile(
      apex_fraction, touchdown_velocity, width, probe_height, liftoff_velocity);
  // What counts as landing at the intended speed rather than merely heading
  // towards it. Half as much again as asked for is generous; the point of the
  // measurement is the *height* over which it holds.
  const float gentle_rate = 1.5f * touchdown_velocity;
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
    // Height decreases monotonically down the descent, so the last sample that
    // is still coming down too fast is the floor of the gentle band: everything
    // below it lands at the intended speed.
    if (phase > apex_fraction && -velocity.z > gentle_rate) {
      p.gentle_band = height;
    }
    if (phase > apex_fraction) {
      p.descent_rise = std::max(p.descent_rise, point.z - prev_point.z);
    }
    // Forward progress is measured against the arc's own span, so this is a
    // pure shape question and independent of stride length or ground speed.
    if (p.height_at_90pct_forward == 0.0f &&
        (point.x - p.start.x) >= 0.9f * (kAep.x - kPep.x)) {
      p.height_at_90pct_forward = height;
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

// ── The asymmetric shape: brisk lift-off, probed touchdown ──

namespace {
constexpr float kApex = 0.45f;         // swing_apex_fraction
constexpr float kProbe = 0.0025f;      // touchdown_probe_height
constexpr float kTouchdownV = 0.05f;   // touchdown_velocity
constexpr float kLiftoffV = 0.05f;     // liftoff_velocity
// About what one servo step works out to at the foot on this machine. Not a
// tolerance — the quantity the probe height has to beat.
constexpr float kServoQuantum = 3.0e-4f;
}  // namespace

// The reason the probe exists.
//
// Braking over the whole descent reaches touchdown_velocity only in the limit,
// at ground level and nowhere else. Measured as a height — the stretch the foot
// is already descending gently over — that landing is a fifth of a millimetre
// tall, which is under a single servo step. The foot therefore never actually
// lands at the speed it was asked to: contact happens wherever quantisation and
// leg-to-leg height error put it, up on the fast part of the curve.
//
// The straight final segment makes the gentle stretch a height we choose.
TEST(Trajectory, ProbeMakesTheGentleLandingTallerThanTheServoResolution) {
  // The regression, at the tuning that provoked it: braking asymptotically
  // towards a touchdown_velocity this small reaches it only in the limit at
  // ground level, so the soft landing is shorter than the servos can resolve
  // and is not a landing the robot ever actually performs.
  const SwingTrace old_tuning = profile_swing(0.35f, 0.001f);
  EXPECT_LT(old_tuning.gentle_band, kServoQuantum);

  // Raising touchdown_velocity alone widens the band — it grows as
  // (v * descent_time)^1.5 — but it is still an asymptote the curve merely
  // approaches, and only a couple of servo steps tall.
  const SwingTrace braked = profile_swing(kApex, kTouchdownV, 0.0f, 0.002f);
  EXPECT_GT(braked.gentle_band, kServoQuantum);
  EXPECT_LT(braked.gentle_band, 3.0f * kServoQuantum);

  // The probe replaces the asymptote with a height we choose: at least as tall
  // as asked for, with the brake's own tail adding a little on top for free.
  const SwingTrace probed =
      profile_swing(kApex, kTouchdownV, 0.0f, 0.002f, kProbe);
  EXPECT_GT(probed.gentle_band, 0.9f * kProbe);
  EXPECT_GT(probed.gentle_band, 3.0f * braked.gentle_band);
}

// Inside the probe the foot comes down at touchdown_velocity and nothing
// faster, so it lands at the same speed wherever in the band contact happens.
// That is the whole difference between a probe and an asymptote.
TEST(Trajectory, ProbeHoldsTouchdownVelocityAllTheWayDown) {
  const SwingTrace p =
      profile_swing(kApex, kTouchdownV, 0.0f, 0.9f * kProbe, kProbe);
  EXPECT_NEAR(p.near_ground_descent, kTouchdownV, 2.0e-3f);
  EXPECT_NEAR(p.touchdown_velocity.z, -kTouchdownV, 2.0e-3f);
}

// The other half of the asymmetry. Zero initial vertical speed leaves the foot
// creeping off the ground inside a single servo step while the leg is still
// carrying load; a prescribed speed breaks contact definitely.
TEST(Trajectory, LiftoffLeavesTheGroundAtTheConfiguredSpeed) {
  const SwingTrace still = profile_swing(kApex, kTouchdownV);
  EXPECT_NEAR(still.liftoff_velocity.z, 0.0f, 2.0e-3f);

  const SwingTrace brisk =
      profile_swing(kApex, kTouchdownV, 0.0f, 0.002f, kProbe, kLiftoffV);
  EXPECT_NEAR(brisk.liftoff_velocity.z, kLiftoffV, 2.0e-3f);

  // Still along the ground track horizontally: a brisk lift is vertical only.
  EXPECT_NEAR(brisk.liftoff_velocity.x, -kLegSpeed, 1e-3f);
  EXPECT_NEAR(brisk.liftoff_velocity.y, 0.0f, 1e-3f);
}

// Zeroing both new knobs restores the plain braked descent exactly, which is
// what makes them safe to A/B against on the robot.
TEST(Trajectory, ZeroedKnobsRestoreTheUnshapedCurve) {
  const SwingTrace zeroed =
      profile_swing(0.35f, 0.01f, 0.02f, 0.002f, 0.0f, 0.0f);
  const SwingTrace bare = profile_swing(0.35f, 0.01f, 0.02f);
  EXPECT_FLOAT_EQ(zeroed.apex_height, bare.apex_height);
  EXPECT_FLOAT_EQ(zeroed.peak_descent, bare.peak_descent);
  EXPECT_FLOAT_EQ(zeroed.liftoff_velocity.z, bare.liftoff_velocity.z);
  EXPECT_FLOAT_EQ(zeroed.touchdown_velocity.z, bare.touchdown_velocity.z);
  EXPECT_FLOAT_EQ(zeroed.max_lateral, bare.max_lateral);
}

// Everything the plain arc guarantees still holds once both ends are shaped:
// the endpoints are exact, step_height still means how high the foot lifts, the
// foot never digs below the ground it is landing on, and there is no C1 break
// at the apex or at the brake -> probe seam.
TEST(Trajectory, ShapedEndsPreserveTheArcInvariants) {
  for (const float apex_fraction : {0.35f, 0.45f, 0.55f}) {
    for (const float probe : {0.0f, kProbe, 0.006f}) {
      const SwingTrace p = profile_swing(apex_fraction, kTouchdownV, 0.02f,
                                         0.002f, probe, kLiftoffV);
      const std::string at = "apex=" + std::to_string(apex_fraction) +
                             " probe=" + std::to_string(probe);
      EXPECT_NEAR(p.start.x, kPep.x, 1e-4f) << at;
      EXPECT_NEAR(p.start.z, kPep.z, 1e-4f) << at;
      EXPECT_NEAR(p.end.x, kAep.x, 1e-4f) << at;
      EXPECT_NEAR(p.end.z, kAep.z, 1e-4f) << at;
      EXPECT_NEAR(p.apex_height, kStepHeight, 3e-3f) << at;
      EXPECT_GT(p.min_height, -1e-5f) << at;
      EXPECT_LT(p.max_velocity_jump, 0.05f) << at;
    }
  }
}

// A swing puts the foot down onto its touchdown point from above. It must not
// reach its final height early and then sweep the rest of the way in at ankle
// level — that is a foot dragged towards its target rather than placed on it,
// and it catches on everything the step height was chosen to clear.
//
// Two independent ways to lose this, both of which look harmless in isolation:
// topping the arc out early (a low apex_fraction, chosen to lengthen the
// descent), and a probe that lasts too long (a low touchdown_velocity, since
// the probe's duration is its height divided by its speed). Either one flattens
// the approach; together they flatten it badly.
TEST(Trajectory, SwingComesDownOntoItsTouchdownPointFromAbove) {
  const SwingTrace p =
      profile_swing(kApex, kTouchdownV, 0.0f, 0.002f, kProbe, kLiftoffV);
  EXPECT_GT(p.height_at_90pct_forward, 0.2f * kStepHeight);

  // Both failure modes, shown to be failure modes.
  const SwingTrace early_apex =
      profile_swing(0.25f, kTouchdownV, 0.0f, 0.002f, kProbe, kLiftoffV);
  EXPECT_LT(early_apex.height_at_90pct_forward, p.height_at_90pct_forward);

  const SwingTrace slow_probe =
      profile_swing(kApex, 0.01f, 0.0f, 0.002f, kProbe, kLiftoffV);
  EXPECT_LT(slow_probe.height_at_90pct_forward, p.height_at_90pct_forward);
}

// Nothing after the apex goes back up: a foot that lifts again on its way down
// has a bump in the descent, which reads on the robot as a hitch just before
// touchdown.
TEST(Trajectory, DescentNeverRises) {
  for (const float apex_fraction : {0.35f, 0.45f, 0.55f}) {
    for (const float probe : {0.0f, kProbe, 0.006f}) {
      const SwingTrace p = profile_swing(apex_fraction, kTouchdownV, 0.02f,
                                         0.002f, probe, kLiftoffV);
      EXPECT_LT(p.descent_rise, 1e-6f)
          << "apex=" << apex_fraction << " probe=" << probe;
    }
  }
}

// A probe asked for more of the descent than there is stays bounded: the brake
// keeps enough of the swing to shed the step height, and the arc still lands
// where it was asked to.
TEST(Trajectory, OversizedProbeIsClampedRatherThanCollapsingTheBrake) {
  const SwingTrace p = profile_swing(kApex, kTouchdownV, 0.0f, 0.002f,
                                     10.0f * kStepHeight, kLiftoffV);
  EXPECT_NEAR(p.end.z, kAep.z, 1e-4f);
  EXPECT_NEAR(p.apex_height, kStepHeight, 3e-3f);
  EXPECT_GT(p.min_height, -1e-5f);
  EXPECT_LT(p.max_velocity_jump, 0.05f);
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

// The two shape terms are independent: the lateral arch survives a profile that
// asks for no lift at all, rather than collapsing with the clearance.
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
        "tripod", hexa::config::kLegSpecs, cfg, g::standing_pose_from_config(),
        hexa::config::kInitialPose, hexa::config::kCoxaToBottom,
        hexa::config::kFootRadius);
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

// crawl and surf have no instant with all six down, so they are out of scope
// above — but three loaded feet is the floor for every gait, and it is what a
// stop interrupting one is entitled to assume.
TEST(Engine, NoGaitWalksOnFewerThanThreeFeet) {
  for (const char* gait : {"tripod", "tetrapod", "surf", "crawl", "ripple"}) {
    int lowest = 6;
    for (const auto& [stance, ticks] : stance_count_histogram(gait, 1200)) {
      (void)ticks;
      lowest = std::min(lowest, stance);
    }
    EXPECT_GE(lowest, 3) << gait << " is statically unstable";
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

// ── Direction reversal ──
//
// Flicking the stick from full left to full right turns the command under six
// planted feet. The stance target is an integral from touchdown, so without a
// bound a leg walks back past its own touchdown point for the rest of a stance
// window — worst on the middle legs, whose own radial reach axis *is* body y.

namespace {

constexpr float kReversalDt = 0.005f;

// The saturating lateral command for a gait, i.e. the cap
// pipeline_config_loader derives and scale_to_envelope enforces.
float saturating_speed(const g::EngineConfig& cfg, float duty_factor = 0.5f) {
  const float swing_end = g::swing_end_phase(duty_factor, cfg.swing_phase_margin);
  return cfg.stride_length * swing_end /
         (cfg.min_swing_time * (1.0f - swing_end));
}

// One tick of BodyVelocityLimiter's vectorial linear slew, so the reversal these
// tests drive is the one the robot actually sees. Lateral-only, so the vector
// slew reduces to a scalar one.
float slew_toward(float v, float target, float accel, float dt) {
  const float step = accel * dt;
  const float delta = target - v;
  return std::fabs(delta) <= step ? target : v + std::copysign(step, delta);
}

struct ReversalStats {
  float peak_excursion = 0.0f;   // max |target.xy - nominal.xy| over all legs
  // Same, but stance legs only, where the target *is* the anchor and the bound
  // therefore applies exactly — the swing arch and ground-line extension ride on
  // top of it and would blur the guarantee.
  float peak_stance_excursion = 0.0f;
  float worst_velocity_jump = 0.0f;  // max tick-to-tick step, same stance streak
  int unreachable = 0;               // ticks whose IK target left the workspace
  int ticks = 0;
  int clock_stalls = 0;   // ticks where master_phase() did not move
  int frozen_swings = 0;  // airborne legs holding an identical target
  std::string worst_leg;        // leg that hit peak_excursion
  std::string unreachable_leg;  // first leg whose target left the workspace
};

// How hard the command turns. The engine is a library — nothing in it requires
// the caller to slew — so both ends of the range have to hold.
enum class Slew {
  kLimiter,  // what BodyVelocityLimiter actually delivers
  kInstant,  // a step command, e.g. a twist_mux source swap
};

// Walk laterally at `+speed` until the gait is steady, then turn the command to
// `-speed` and keep going. `flip_offset` shifts the reversal within the gait
// cycle, so a sweep covers every phase a leg can be caught in.
ReversalStats drive_reversal(float speed, float flip_offset,
                             bool reverse = true,
                             Slew slew = Slew::kLimiter) {
  const auto nominal = g::nominal_stance_from_config();
  const auto specs = g::leg_specs_from_config();
  const float accel =
      slew == Slew::kInstant
          ? std::numeric_limits<float>::max()
          : speed / hexa::config::kControl.vmax_ramp_time_linear;

  auto e = g::make_default_engine("tripod");
  for (int i = 0; i < 6000 && e->state() != g::EngineState::STAND; ++i) {
    e->update(kReversalDt, {0.0f, 0.0f}, 0.0f);
    e->start_initialize();
  }
  EXPECT_EQ(e->state(), g::EngineState::STAND);

  float v_y = 0.0f;
  const auto tick = [&](float target) {
    v_y = slew_toward(v_y, target, accel, kReversalDt);
    return e->update(kReversalDt, {0.0f, v_y}, 0.0f);
  };

  // Settle: reach GAIT, then a couple of full cycles at the saturating command
  // so every leg has cycled through the planner at least once.
  for (int i = 0; i < 3000 && e->state() != g::EngineState::GAIT; ++i) {
    tick(speed);
  }
  EXPECT_EQ(e->state(), g::EngineState::GAIT);
  for (int i = 0; i < 600; ++i) tick(speed);
  for (int i = 0; i < static_cast<int>(flip_offset / kReversalDt); ++i) {
    tick(speed);
  }

  ReversalStats s;
  struct LegTrace {
    g::Vec3 target = g::Vec3::Zero();
    g::Vec3 velocity = g::Vec3::Zero();
    bool have = false;
    int stance_streak = 0;
    int swing_streak = 0;
  };
  std::map<std::string, LegTrace> trace;
  float prev_master = e->master_phase();

  for (int i = 0; i < 1200; ++i) {
    const auto out = tick(reverse ? -speed : speed);
    ++s.ticks;
    if (std::fabs(e->master_phase() - prev_master) < 1e-9f) ++s.clock_stalls;
    prev_master = e->master_phase();

    for (const auto& [name, leg] : out) {
      LegTrace& t = trace[name];
      const g::Vec3 d = leg.foot_target - nominal.at(name);
      const float excursion = std::hypot(d.x, d.y);
      if (excursion > s.peak_excursion) {
        s.peak_excursion = excursion;
        s.worst_leg = name;
      }
      if (leg.stance) {
        s.peak_stance_excursion = std::max(s.peak_stance_excursion, excursion);
      }

      const g::Vec3 in_leg =
          hexa::body_to_leg(leg.foot_target, specs.at(name));
      try {
        (void)hexa::inverse_kinematics(in_leg, specs.at(name));
      } catch (const hexa::UnreachableTarget&) {
        ++s.unreachable;
        if (s.unreachable_leg.empty()) s.unreachable_leg = name;
      }

      t.stance_streak = leg.stance ? t.stance_streak + 1 : 0;
      t.swing_streak = leg.stance ? 0 : t.swing_streak + 1;
      if (!leg.stance && t.have &&
          (leg.foot_target - t.target).norm() < 1e-9f) {
        ++s.frozen_swings;
      }
      // Only compare two samples taken inside the same stance streak, so the
      // touchdown seam's deliberate vertical step is never differenced.
      const g::Vec3 velocity =
          t.have ? (leg.foot_target - t.target) / kReversalDt : g::Vec3::Zero();
      if (t.have && t.stance_streak >= 3) {
        const g::Vec3 jump = velocity - t.velocity;
        s.worst_velocity_jump =
            std::max(s.worst_velocity_jump, std::hypot(jump.x, jump.y));
      }
      t.velocity = velocity;
      t.target = leg.foot_target;
      t.have = true;
    }
  }
  return s;
}

}  // namespace

// The headline regression. A stance anchor that runs on past its band drives the
// middle legs to the edge of their workspace — outward until the leg is dead
// straight, and inward past the IK's *inner* annulus (|femur - tibia|), where
// Pipeline::compose_gait silently freezes that leg on its last-good angles.
TEST(Engine, StickReversalKeepsEveryFootReachable) {
  const auto cfg = g::engine_config_from_config();
  const float speed = saturating_speed(cfg);
  const float cycle = cfg.max_swing_time /
                      g::swing_end_phase(0.5f, cfg.swing_phase_margin);

  // Against the slewed command, which is the only one the engine ever sees:
  // Pipeline runs every tick through Control::shape / BodyVelocityLimiter. A
  // stepped reversal additionally bulges the *swing*, whose lift-off ground line
  // is latched at v_origin — a separate mechanism this bound does not address.
  for (int i = 0; i < 12; ++i) {
    const float offset = cycle * static_cast<float>(i) / 12.0f;
    const ReversalStats s = drive_reversal(speed, offset);
    EXPECT_EQ(s.unreachable, 0) << "flip offset " << offset
                                << " left the workspace on " << s.unreachable_leg;
  }
}

// The bound's guarantee is a hard ceiling on the anchor, not a tendency: however
// hard the command turns, a planted foot stays within `1 + kStanceExcursionGrace`
// of half a stride from its nominal. Unbounded, a stepped reversal walks it out
// to three times that.
TEST(Engine, StanceAnchorNeverPassesItsCeiling) {
  const auto cfg = g::engine_config_from_config();
  const float speed = saturating_speed(cfg);
  const float band = 0.5f * cfg.stride_length;
  // Mirrors kStanceExcursionGrace, which is engine-internal.
  const float ceiling = band * 1.25f;
  const float cycle = cfg.max_swing_time /
                      g::swing_end_phase(0.5f, cfg.swing_phase_margin);

  const float steady =
      drive_reversal(speed, 0.0f, /*reverse=*/false).peak_stance_excursion;
  EXPECT_NEAR(steady, band, speed * kReversalDt * 2.0f)
      << "a steady walk should ride exactly on the band, and no further";

  for (const Slew slew : {Slew::kLimiter, Slew::kInstant}) {
    for (int i = 0; i < 12; ++i) {
      const float offset = cycle * static_cast<float>(i) / 12.0f;
      const ReversalStats s = drive_reversal(speed, offset, true, slew);
      EXPECT_LT(s.peak_stance_excursion, ceiling)
          << "flip offset " << offset << " on " << s.worst_leg;
    }
  }
}

// Whatever the bound does under a reversal, it has to be invisible the rest of
// the time: at a constant command every stance leg tracks the ground exactly.
TEST(Engine, SteadyWalkNeverTouchesTheStanceBound) {
  const auto cfg = g::engine_config_from_config();
  const auto nominal = g::nominal_stance_from_config();
  const float speed = saturating_speed(cfg);

  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  for (int i = 0; i < 3000 && e->state() != g::EngineState::GAIT; ++i) {
    e->update(kReversalDt, {0.0f, speed}, 0.0f);
  }
  ASSERT_EQ(e->state(), g::EngineState::GAIT);
  for (int i = 0; i < 600; ++i) e->update(kReversalDt, {0.0f, speed}, 0.0f);

  std::map<std::string, g::Vec3> prev;
  std::map<std::string, int> streak;
  float worst_error = 0.0f;
  float reached = 0.0f;
  for (int i = 0; i < 1200; ++i) {
    const auto out = e->update(kReversalDt, {0.0f, speed}, 0.0f);
    for (const auto& [name, leg] : out) {
      streak[name] = leg.stance ? streak[name] + 1 : 0;
      if (leg.stance && streak[name] >= 2) {
        // A planted foot tracks the ground exactly: -v_leg * dt, unattenuated.
        const g::Vec3 d = leg.foot_target - prev[name];
        worst_error =
            std::max(worst_error, std::fabs(std::hypot(d.x, d.y) -
                                            speed * kReversalDt));
      }
      const g::Vec3 e_xy = leg.foot_target - nominal.at(name);
      if (leg.stance) reached = std::max(reached, std::hypot(e_xy.x, e_xy.y));
      prev[name] = leg.foot_target;
    }
  }
  EXPECT_LT(worst_error, 1e-5f) << "the bound is attenuating a steady walk";
  // And it rides right up to the band, so the knee cannot be set any lower
  // without biting in ordinary use.
  EXPECT_NEAR(reached, 0.5f * cfg.stride_length, speed * kReversalDt * 2.0f);
}

// The bound is a wall, not a spring: a leg carried out to the ceiling has to come
// back at full rate the moment the command turns, or it lags its own recovery.
TEST(Engine, StanceAnchorRecoversInwardAtFullRate) {
  const auto cfg = g::engine_config_from_config();
  const float band = 0.5f * cfg.stride_length;
  const float ceiling = band * 1.15f;
  g::StanceIntegrator stance;
  const g::Vec3 nominal(0.0f, 0.2f, -0.066f);
  const g::StanceBand bound{nominal, band, ceiling};
  const std::string leg = "l_middle";
  constexpr float dt = 0.005f;
  const float v = 0.15f;

  // Seat the anchor on nominal, then drive it outward until it stops.
  stance.step(leg, true, nominal, {0.0f, 0.0f}, dt, bound);
  for (int i = 0; i < 4000; ++i) {
    stance.step(leg, true, nominal, {0.0f, -v}, dt, bound);
  }
  const g::Vec3 pinned = *stance.step(leg, true, nominal, {0.0f, -v}, dt, bound);
  EXPECT_LT(pinned.y - nominal.y, ceiling + 1e-6f)
      << "the anchor passed its ceiling";
  EXPECT_GT(pinned.y - nominal.y, band)
      << "the drive never pushed the anchor out of its band";

  const g::Vec3 back = *stance.step(leg, true, nominal, {0.0f, v}, dt, bound);
  EXPECT_NEAR(pinned.y - back.y, v * dt, 1e-7f)
      << "inward recovery is being attenuated";
}

// A hard clip on the anchor would bound the excursion just as well and put a
// velocity kink in the foot target at the moment it bit. The eased saturation is
// the reason this passes.
TEST(Engine, FootTargetStaysVelocityContinuousAtTheStanceBound) {
  const auto cfg = g::engine_config_from_config();
  const float speed = saturating_speed(cfg);
  const float cycle = cfg.max_swing_time /
                      g::swing_end_phase(0.5f, cfg.swing_phase_margin);

  for (int i = 0; i < 12; ++i) {
    const float offset = cycle * static_cast<float>(i) / 12.0f;
    const ReversalStats s = drive_reversal(speed, offset);
    // A hard clip would shed the foot's whole ground speed in a single tick.
    // Easing it across the grace band spreads that over ten or more, so state
    // the bound against the speed itself rather than an absolute number that a
    // stride_length retune would invalidate.
    EXPECT_LT(s.worst_velocity_jump, 0.1f * speed) << "flip offset " << offset;
  }
}

// The command crosses cmd_zero_tol on its way through a reversal. Holding the
// targets there froze the whole gait for ~0.2 s with a leg hanging in the air.
TEST(Engine, GaitKeepsRunningWhileTheCommandCrossesZero) {
  const auto cfg = g::engine_config_from_config();
  const ReversalStats s = drive_reversal(saturating_speed(cfg), 0.0f);
  EXPECT_EQ(s.clock_stalls, 0) << "the gait clock stopped mid-reversal";
  EXPECT_EQ(s.frozen_swings, 0) << "an airborne leg held its target";
}

// ── Settling to a stop ──
//
// Stopping is not a separate mechanism: at a zero command the stride collapses,
// so every leg's AEP *is* its nominal stance and the gait re-plants its own feet
// one swing at a time. SETTLING is that, run to completion — no pause ladder, no
// reseat, no engagement pass to get back out of.

namespace {

// Walk laterally at `speed` until the gait is steady, then release the stick and
// tick until STAND. `flip_offset` shifts the release within the gait cycle so a
// sweep covers every phase a leg can be caught in. Returns what the settle did.
struct SettleStats {
  bool used_reseat = false;
  bool reached_stand = false;
  float settle_seconds = 0.0f;   // from the release to STAND
  float worst_off_nominal = 0.0f;  // max |target.xy - nominal.xy| at the handoff
  float worst_velocity_jump = 0.0f;  // max tick-to-tick step, same stance streak
  int clock_stalls = 0;
  int frozen_swings = 0;
  int settling_ticks = 0;
  int swings_after_release = 0;  // most lift-offs any one leg made after cmd zero
  std::string worst_leg;
  std::string last_state;
  // Fastest a foot moved in one tick during the stop, and during the steady
  // walk before it, as a yardstick.
  float worst_tick_jump = 0.0f;
  float walk_tick_jump = 0.0f;
  std::string jump_leg;
  float jump_at = 0.0f;
  std::string jump_state;
};

SettleStats drive_settle(float speed, float flip_offset,
                         const std::string& gait = "tripod",
                         Slew slew = Slew::kLimiter) {
  const auto nominal = g::nominal_stance_from_config();
  // Releasing the stick does not step the engine's command to zero: Control
  // slews it out over vmax_ramp_time_linear, so the last few swings before the
  // command reads zero land at a shrinking stride, a little short of nominal.
  // That ramp is the reason the settle has to judge where the feet are rather
  // than count touchdowns.
  const float accel =
      slew == Slew::kInstant
          ? std::numeric_limits<float>::max()
          : speed / hexa::config::kControl.vmax_ramp_time_linear;

  auto e = g::make_default_engine(gait);
  run_to_stand(*e);
  for (int i = 0; i < 6000 && e->state() != g::EngineState::GAIT; ++i) {
    e->update(kReversalDt, {0.0f, speed}, 0.0f);
  }
  EXPECT_EQ(e->state(), g::EngineState::GAIT) << gait;
  SettleStats s;
  // Steady walk, and the yardstick it sets: the fastest any foot moves in one
  // tick while the gait is doing its ordinary job.
  {
    std::map<std::string, g::Vec3> prev;
    std::map<std::string, bool> have;
    for (int i = 0; i < 1200; ++i) {
      for (const auto& [n, leg] : e->update(kReversalDt, {0.0f, speed}, 0.0f)) {
        if (have[n]) {
          s.walk_tick_jump =
              std::max(s.walk_tick_jump, (leg.foot_target - prev[n]).norm());
        }
        prev[n] = leg.foot_target;
        have[n] = true;
      }
    }
  }
  for (int i = 0; i < static_cast<int>(flip_offset / kReversalDt); ++i) {
    e->update(kReversalDt, {0.0f, speed}, 0.0f);
  }

  struct LegTrace {
    g::Vec3 target = g::Vec3::Zero();
    g::Vec3 velocity = g::Vec3::Zero();
    bool have = false;
    bool was_stance = true;
    int stance_streak = 0;
    int liftoffs = 0;
  };
  std::map<std::string, LegTrace> trace;
  float prev_master = e->master_phase();
  float v_y = speed;

  // Everything the settle is judged on is measured from the moment the *engine*
  // sees a zero command, not from the release: the ramp before that is ordinary
  // walking, and the legs are supposed to keep stepping through it.
  int zero_tick = -1;

  for (int i = 0; i < 2000; ++i) {
    v_y = slew_toward(v_y, 0.0f, accel, kReversalDt);
    const auto out = e->update(kReversalDt, {0.0f, v_y}, 0.0f);
    const bool cmd_zero =
        std::fabs(v_y) < g::engine_config_from_config().cmd_zero_tol;
    if (cmd_zero && zero_tick < 0) zero_tick = i;
    if (e->state() == g::EngineState::SETTLING) ++s.settling_ticks;
    if (e->state() == g::EngineState::RESEATING) s.used_reseat = true;
    if (e->state() == g::EngineState::STAND) {
      s.reached_stand = true;
      s.settle_seconds = static_cast<float>(i - zero_tick) * kReversalDt;
      for (const auto& [name, leg] : out) {
        const g::Vec3 d = leg.foot_target - nominal.at(name);
        const float off = std::hypot(d.x, d.y);
        if (off > s.worst_off_nominal) {
          s.worst_off_nominal = off;
          s.worst_leg = name;
        }
      }
      break;
    }

    if (std::fabs(e->master_phase() - prev_master) < 1e-9f) ++s.clock_stalls;
    prev_master = e->master_phase();

    for (const auto& [name, leg] : out) {
      LegTrace& t = trace[name];
      if (t.was_stance && !leg.stance && zero_tick >= 0) ++t.liftoffs;
      t.was_stance = leg.stance;
      s.swings_after_release = std::max(s.swings_after_release, t.liftoffs);
      t.stance_streak = leg.stance ? t.stance_streak + 1 : 0;
      if (!leg.stance && t.have && (leg.foot_target - t.target).norm() < 1e-9f) {
        ++s.frozen_swings;
      }
      // Only differenced inside one stance streak, so the touchdown seam's
      // deliberate vertical step is never sampled.
      const g::Vec3 velocity =
          t.have ? (leg.foot_target - t.target) / kReversalDt : g::Vec3::Zero();
      if (t.have && t.stance_streak >= 3) {
        const g::Vec3 jump = velocity - t.velocity;
        s.worst_velocity_jump =
            std::max(s.worst_velocity_jump, std::hypot(jump.x, jump.y));
      }
      if (t.have && zero_tick >= 0) {
        const float jump = (leg.foot_target - t.target).norm();
        if (jump > s.worst_tick_jump) {
          s.worst_tick_jump = jump;
          s.jump_leg = name;
          s.jump_at = static_cast<float>(i - zero_tick) * kReversalDt;
          s.jump_state = g::state_name(e->state());
        }
      }
      t.velocity = velocity;
      t.target = leg.foot_target;
      t.have = true;
    }
  }
  s.last_state = g::state_name(e->state());
  return s;
}

// Worst case: the debounce, then one full settle cycle for the last leg to get
// its turn in the air and land.
float settle_budget(const g::EngineConfig& cfg) {
  const float swing_end = g::swing_end_phase(0.5f, cfg.swing_phase_margin);
  return cfg.settle_debounce_delay + cfg.settle_swing_time / swing_end;
}

// The command the engine walks on, shaped through the same envelope the ROS /
// firmware callers apply, so a per-gait test drives what the gait can actually
// take rather than saturating the stance excursion bound.
std::tuple<float, float, float> shaped_full_stick(const char* gait) {
  const auto nominal = g::nominal_stance_from_config();
  const auto caps =
      g::load_velocity_caps_from_config(g::outer_stance_radius(nominal));
  return g::scale_to_envelope(caps.linear_max(gait), 0.0f,
                              caps.angular_max(gait), nominal,
                              caps.linear_max(gait), caps.yaw_bias(gait));
}

}  // namespace

// Releasing mid-engagement must re-plant the feet, not declare them home.
//
// The engagement integrates its stance legs at the *commanded* velocity, so a
// zero command freezes each one where the walk had carried it — nothing brings
// them back to nominal on their own, and a leg still waiting for its first
// lift-off never swings at all. Assigning nominal_ at the handoff therefore
// teleported every planted foot at once, with no lift. Worst on a ripple, whose
// engagement is a full cycle long, so a short drive usually ends inside it.
TEST(Engine, ReleasingMidEngagementReplantsInsteadOfTeleporting) {
  constexpr float kTickDt = 0.005f;
  const auto nominal = g::nominal_stance_from_config();

  for (const char* gait : {"tripod", "tetrapod", "surf", "crawl", "ripple"}) {
    const auto [v_x, v_y, omega] = shaped_full_stick(gait);
    for (const int drive_ticks : {100, 300, 600}) {
      auto e = g::make_default_engine(gait);
      run_to_stand(*e);
      for (int i = 0; i < drive_ticks; ++i) {
        e->update(kTickDt, {v_x, v_y}, omega);
      }

      std::map<std::string, g::Vec3> prev;
      float worst_jump = 0.0f;
      std::string worst_leg;
      bool stood = false;
      for (int i = 0; i < 3000; ++i) {
        const auto out = e->update(kTickDt, {0.0f, 0.0f}, 0.0f);
        for (const auto& [name, leg] : out) {
          auto it = prev.find(name);
          if (it != prev.end()) {
            const float jump = (leg.foot_target - it->second).norm();
            if (jump > worst_jump) {
              worst_jump = jump;
              worst_leg = name;
            }
          }
          prev[name] = leg.foot_target;
        }
        if (e->state() == g::EngineState::STAND && i > 3) {
          stood = true;
          break;
        }
      }

      const std::string where =
          std::string(gait) + " released after " + std::to_string(drive_ticks) +
          " ticks";
      ASSERT_TRUE(stood) << where << " never reached STAND";
      // A swing at the settle's own pace is the fastest a foot is ever asked to
      // move; anything beyond that is a teleport, not a step. The pre-fix
      // handoff jumped up to half a stride in a single tick.
      EXPECT_LT(worst_jump, 0.004f)
          << where << ": " << worst_leg << " moved " << worst_jump * 1000.0f
          << " mm in one tick";
      for (const auto& n : g::LEG_NAMES) {
        EXPECT_LT((prev.at(n) - nominal.at(n)).norm(), 1e-5f)
            << where << ": stood on " << n << " off its nominal stance";
      }
    }
  }
}

// The stop has to keep the robot standing, whichever route it takes. A release
// inside the engagement is the hard case: it hands the reseat ladder feet that
// are still in the air, and the metachronal gaits engage for one whole cycle
// (1.7 s on a crawl, 3.4 s on a ripple), so short drives end there routinely.
TEST(Engine, AStopNeverDropsBelowThreeLoadedFeet) {
  constexpr float kTickDt = 0.005f;
  const auto cfg = g::engine_config_from_config();
  const auto nominal = g::nominal_stance_from_config();
  const float budget = 4.0f * cfg.reseat_pair_swing_time +
                       3.0f * cfg.reseat_pair_dwell_time +
                       cfg.settle_debounce_delay + cfg.settle_swing_time;

  for (const char* gait : {"tripod", "tetrapod", "surf", "crawl", "ripple"}) {
    const auto [v_x, v_y, omega] = shaped_full_stick(gait);
    bool covered_engagement = false;

    for (const int drive_ticks : {40, 80, 160, 240, 320, 400, 800}) {
      auto e = g::make_default_engine(gait);
      run_to_stand(*e);
      for (int i = 0; i < drive_ticks; ++i) {
        e->update(kTickDt, {v_x, v_y}, omega);
      }
      const bool released_engaging =
          e->state() == g::EngineState::ENGAGING;
      covered_engagement = covered_engagement || released_engaging;

      int fewest_loaded = 6;
      int stop_ticks = -1;
      for (int i = 0; i < 4000; ++i) {
        const auto out = e->update(kTickDt, {0.0f, 0.0f}, 0.0f);
        int loaded = 0;
        for (const auto& [name, leg] : out) {
          // A foot only carries weight if it is both claimed as stance and
          // actually on the ground; the two used to disagree.
          const float band = nominal.at(name)[2] + cfg.touchdown_probe_height;
          if (leg.stance && leg.foot_target[2] <= band) {
            ++loaded;
          }
        }
        fewest_loaded = std::min(fewest_loaded, loaded);
        if (e->state() == g::EngineState::STAND && i > 3) {
          stop_ticks = i;
          break;
        }
      }

      const std::string where = std::string(gait) + " released after " +
                                std::to_string(drive_ticks) + " ticks" +
                                (released_engaging ? " (engaging)" : "");
      ASSERT_GE(stop_ticks, 0) << where << " never reached STAND";
      EXPECT_GE(fewest_loaded, 3) << where << " went statically unstable";
      EXPECT_LT(static_cast<float>(stop_ticks) * kTickDt, budget) << where;
    }

    EXPECT_TRUE(covered_engagement)
        << gait << " never released inside its engagement";
  }
}

TEST(Engine, ZeroCommandSettlesToStand) {
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  for (int i = 0; i < 200 && e->state() != g::EngineState::GAIT; ++i) {
    e->update(kDt, {0.06f, 0.0f}, 0.0f);
  }
  ASSERT_EQ(e->state(), g::EngineState::GAIT);

  // Drop the command: the engine debounces, settles, and stands. It never
  // detours through a re-plant — RESEATING is the height-change path only.
  bool saw_settling = false;
  bool returned = false;
  for (int i = 0; i < 400; ++i) {
    e->update(kDt, {0.0f, 0.0f}, 0.0f);
    saw_settling = saw_settling || e->state() == g::EngineState::SETTLING;
    EXPECT_NE(e->state(), g::EngineState::RESEATING);
    if (e->state() == g::EngineState::STAND) {
      returned = true;
      break;
    }
  }
  EXPECT_TRUE(saw_settling);
  EXPECT_TRUE(returned);
}

// The premise of the whole design: a settling swing aims at the live AEP, which
// at a zero stride *is* the nominal stance. If that ever stopped being exact,
// the robot would stand on a stance it never re-planted onto.
TEST(Engine, EveryFootLandsOnNominalWhenTheSettleFinishes) {
  const auto cfg = g::engine_config_from_config();
  const float speed = saturating_speed(cfg);
  const float cycle =
      cfg.max_swing_time / g::swing_end_phase(0.5f, cfg.swing_phase_margin);

  for (int i = 0; i < 12; ++i) {
    const float offset = cycle * static_cast<float>(i) / 12.0f;
    const SettleStats s = drive_settle(speed, offset);
    ASSERT_TRUE(s.reached_stand) << "release offset " << offset;
    EXPECT_LT(s.worst_off_nominal, 1e-6f)
        << "release offset " << offset << " stood on " << s.worst_leg
        << " " << s.worst_off_nominal << " m off its nominal stance";
  }
}

// The settle is bounded by its own cycle, whatever phase the release lands in —
// there is no state that can sit and wait.
TEST(Engine, SettleFinishesWithinOneCycle) {
  const auto cfg = g::engine_config_from_config();
  const float speed = saturating_speed(cfg);
  const float budget = settle_budget(cfg);
  const float cycle =
      cfg.max_swing_time / g::swing_end_phase(0.5f, cfg.swing_phase_margin);

  for (int i = 0; i < 12; ++i) {
    const float offset = cycle * static_cast<float>(i) / 12.0f;
    const SettleStats s = drive_settle(speed, offset);
    ASSERT_TRUE(s.reached_stand) << "release offset " << offset;
    EXPECT_LT(s.settle_seconds, budget + 4.0f * kReversalDt)
        << "release offset " << offset;
    EXPECT_GT(s.settling_ticks, 0) << "never entered SETTLING";
  }
}

// No leg picks its foot up twice to reach the same place. This is what a latch
// armed at the SETTLING edge got wrong: legs that had already re-planted during
// the debounce were made to swing again to prove it, so a tripod stop took two
// visible cycles instead of one.
TEST(Engine, NoLegLiftsTwiceToSettle) {
  const auto cfg = g::engine_config_from_config();
  const float speed = saturating_speed(cfg);
  const float cycle =
      cfg.max_swing_time / g::swing_end_phase(0.5f, cfg.swing_phase_margin);

  for (int i = 0; i < 12; ++i) {
    const float offset = cycle * static_cast<float>(i) / 12.0f;
    const SettleStats s = drive_settle(speed, offset);
    ASSERT_TRUE(s.reached_stand) << "release offset " << offset;
    EXPECT_FALSE(s.used_reseat) << "a tripod should settle on its own";
    EXPECT_LE(s.swings_after_release, 1)
        << "release offset " << offset << ": a leg swung "
        << s.swings_after_release << " times to settle";
  }
}

// ── The other gaits ──
//
// A tripod walks itself home in one cycle. The longer duty factors take longer
// over it, and a crawl — whose swings run end to end, so there is never an
// instant with all six feet down — cannot finish at all. Those hold their
// lift-offs, let what is already in the air land, and hand the rest to the
// reseat ladder. Which route a gait takes is a comparison the engine makes, not
// a list of names, so the tests state the promise rather than re-derive it.

namespace {

// The ladder route's worst case, measured from the moment the command reads
// zero: the debounce, one swing spent waiting for whatever is in the air to
// land, then three mirrored pairs with a dwell between them. The engine only
// picks the gait's own route when it is quicker than this, so it bounds every
// stop whichever route was taken.
float stop_budget(const g::EngineConfig& cfg) {
  return cfg.settle_debounce_delay + cfg.settle_swing_time +
         3.0f * cfg.reseat_pair_swing_time + 2.0f * cfg.reseat_pair_dwell_time;
}

// What the gait would need to walk every leg home itself: one cycle.
float natural_budget(const g::EngineConfig& cfg, float duty) {
  return cfg.settle_debounce_delay +
         cfg.settle_swing_time / g::swing_end_phase(duty, cfg.swing_phase_margin);
}

}  // namespace

TEST(Engine, EveryGaitSettlesOntoTheNominalStance) {
  const auto cfg = g::engine_config_from_config();
  const float budget = stop_budget(cfg);

  for (const auto& [name, factory] : g::strategies()) {
    const float duty = factory()->duty_factor();
    const SettleStats s = drive_settle(saturating_speed(cfg, duty), 0.0f, name);

    ASSERT_TRUE(s.reached_stand)
        << name << " stuck in " << s.last_state << " after "
        << s.settling_ticks << " settling ticks";
    EXPECT_LT(s.worst_off_nominal, 1e-5f) << name << " on " << s.worst_leg;
    // Whichever route it took, no foot was picked up twice to reach the same
    // place — the gait swing and the ladder swing are alternatives, not stages.
    EXPECT_LE(s.swings_after_release, 1) << name;
    EXPECT_LT(s.settle_seconds, budget + 4.0f * kReversalDt)
        << name << " settled in " << s.settle_seconds << " s against a "
        << budget << " s budget (reseat: " << s.used_reseat << ")";
    // Stopping must never move a foot faster than walking already does. This is
    // what catches a discontinuity: a stop is slower motion than a walk, so any
    // snap stands out against the gait's own yardstick immediately.
    EXPECT_LT(s.worst_tick_jump, s.walk_tick_jump)
        << name << " moved a foot " << s.worst_tick_jump * 1000.0f
        << " mm in one tick while stopping, against " << s.walk_tick_jump * 1000.0f
        << " mm walking — on " << s.jump_leg << " at t=" << s.jump_at << " in "
        << s.jump_state;
  }
}

// The two ends of the rule, pinned by name so a change of heart about either is
// deliberate: a tripod is quick enough on its own, a crawl — whose swings run
// end to end — never has the all-six-down instant that finishing needs.
TEST(Engine, TripodSettlesItselfAndACrawlUsesTheReseat) {
  const auto cfg = g::engine_config_from_config();
  const auto route = [&](const std::string& gait) {
    const float duty = g::strategies().at(gait)()->duty_factor();
    return drive_settle(saturating_speed(cfg, duty), 0.0f, gait);
  };

  const SettleStats tripod = route("tripod");
  ASSERT_TRUE(tripod.reached_stand);
  EXPECT_FALSE(tripod.used_reseat) << "a tripod should settle on its own";
  EXPECT_LT(tripod.settle_seconds,
            natural_budget(cfg, 0.5f) + 4.0f * kReversalDt);

  const SettleStats crawl = route("crawl");
  ASSERT_TRUE(crawl.reached_stand);
  EXPECT_TRUE(crawl.used_reseat)
      << "a crawl never has all six feet down, so it cannot settle on its own";
  EXPECT_LT(crawl.settle_seconds, stop_budget(cfg) + 4.0f * kReversalDt);
}

// A settle is the gait still running, so the two things that were wrong about
// the old freeze must stay fixed all the way to the handoff: the clock keeps
// turning and no airborne leg holds its target.
TEST(Engine, SettleKeepsTheGaitRunning) {
  const auto cfg = g::engine_config_from_config();
  const SettleStats s = drive_settle(saturating_speed(cfg), 0.0f);
  EXPECT_EQ(s.clock_stalls, 0) << "the gait clock stopped while settling";
  EXPECT_EQ(s.frozen_swings, 0) << "an airborne leg held its target";
}

// Arming the settle swaps the clock from max_cycle_time to the settle cycle
// time. That reshapes any in-flight swing arc, so check it does not show up as
// a step in the foot target.
TEST(Engine, SettleArmsWithoutSteppingTheFootTarget) {
  const auto cfg = g::engine_config_from_config();
  const float speed = saturating_speed(cfg);
  const float cycle =
      cfg.max_swing_time / g::swing_end_phase(0.5f, cfg.swing_phase_margin);

  for (int i = 0; i < 12; ++i) {
    const float offset = cycle * static_cast<float>(i) / 12.0f;
    const SettleStats s = drive_settle(speed, offset);
    EXPECT_LT(s.worst_velocity_jump, 0.1f * speed)
        << "release offset " << offset;
  }
}

// The clock never stops, so a command that comes back mid-settle is picked up by
// the ordinary gait tick — no engagement ladder to re-run, and no jump in the
// foot targets from re-entering one.
TEST(Engine, CommandReturningMidSettleResumesTheGaitDirectly) {
  const auto cfg = g::engine_config_from_config();
  const float speed = saturating_speed(cfg);
  auto e = g::make_default_engine("tripod");
  run_to_stand(*e);
  for (int i = 0; i < 3000 && e->state() != g::EngineState::GAIT; ++i) {
    e->update(kReversalDt, {0.0f, speed}, 0.0f);
  }
  ASSERT_EQ(e->state(), g::EngineState::GAIT);

  // Steady walk, and the yardstick it sets: how much a foot's velocity changes
  // in one ordinary tick. A resume that re-entered an engagement, or dropped a
  // held leg into an arc already part-way along, would blow straight past it.
  float walk_accel = 0.0f;
  std::map<std::string, g::Vec3> prev, prev_step;
  std::map<std::string, int> seen;
  for (int i = 0; i < 600; ++i) {
    for (const auto& [name, leg] : e->update(kReversalDt, {0.0f, speed}, 0.0f)) {
      const g::Vec3 step = leg.foot_target - prev[name];
      if (seen[name] >= 2) {
        walk_accel = std::max(walk_accel, (step - prev_step[name]).norm());
      }
      prev_step[name] = step;
      prev[name] = leg.foot_target;
      ++seen[name];
    }
  }
  ASSERT_GT(walk_accel, 0.0f);

  // Release for a shade longer than the debounce, so SETTLING is entered but
  // nowhere near done.
  const int release_ticks =
      static_cast<int>(cfg.settle_debounce_delay / kReversalDt) + 20;
  for (int i = 0; i < release_ticks; ++i) {
    for (const auto& [name, leg] : e->update(kReversalDt, {0.0f, 0.0f}, 0.0f)) {
      prev_step[name] = leg.foot_target - prev[name];
      prev[name] = leg.foot_target;
    }
  }
  ASSERT_EQ(e->state(), g::EngineState::SETTLING);

  const auto out = e->update(kReversalDt, {0.0f, speed}, 0.0f);
  EXPECT_EQ(e->state(), g::EngineState::GAIT)
      << "a returning command should resume the gait on the same tick";
  for (const auto& [name, leg] : out) {
    const g::Vec3 step = leg.foot_target - prev.at(name);
    EXPECT_LT((step - prev_step.at(name)).norm(), walk_accel)
        << name << " changed velocity more on the resume tick than it ever does"
        << " while walking";
  }
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

  // Keep commanding forward; the engine settles to a stand anyway, applies the
  // change at the handoff, and re-engages under the new strategy.
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

// ── Standing pose: per-group tip reach + splay, plus one body height ──
//
// The stance is configured by where the feet sit, not by joint angles:
// standing_pose_from solves the per-leg triple. The front, middle and rear pairs
// are configured separately and left/right mirror. These pin the splay
// mirroring, the footprint the values describe, that the groups stay distinct,
// and — the reason the splay is safe to use at all — that a reseat keeps each
// leg pointing where it already points.

namespace {

// A standing pose with the given splay on every group, and every group reaching
// out the same distance. The reach is pinned rather than inherited from the
// shipped YAML so these cases stay about the splay alone — a tuning.yaml that
// gives its groups different reaches must not change what they assert.
constexpr float kUniformReach = 0.140f;

::hexa::config::StandingPose standing_with_splay(float coxa_deg) {
  ::hexa::config::StandingPose sp = hexa::config::kStandingPose;
  for (auto& grp : sp.groups) {
    grp.tip_reach = kUniformReach;
    grp.coxa = coxa_deg * static_cast<float>(M_PI) / 180.0f;
  }
  return sp;
}

// A standing pose whose three groups reach out different distances. Kept close
// together so every leg stays inside its joint limits.
::hexa::config::StandingPose standing_with_group_reaches(float front,
                                                         float middle,
                                                         float rear) {
  ::hexa::config::StandingPose sp = hexa::config::kStandingPose;
  sp.groups[hexa::group_index(hexa::LegGroup::FRONT)].tip_reach = front;
  sp.groups[hexa::group_index(hexa::LegGroup::MIDDLE)].tip_reach = middle;
  sp.groups[hexa::group_index(hexa::LegGroup::REAR)].tip_reach = rear;
  return sp;
}

constexpr float kSplayDeg = 15.0f;
constexpr float kSplayRad = kSplayDeg * static_cast<float>(M_PI) / 180.0f;

// Planar distance from a leg's own coxa axis to its foot.
float reach_in_leg_frame(const g::Vec3& foot, const g::kin::LegSpec& spec) {
  const g::Vec3 in_leg = hexa::body_to_leg(foot, spec);
  return std::hypot(in_leg[0], in_leg[1]);
}

}  // namespace

TEST(StandingPose, SplaySignIsOutwardOnEveryLeg) {
  const auto pose = g::standing_pose_from(hexa::config::kLegSpecs,
                                          hexa::config::kCoxaToBottom,
                                          hexa::config::kFootRadius,
                                          standing_with_splay(kSplayDeg));

  // A positive coxa_deg splays outward, which is the left leg's value negated
  // for rear legs and again for right ones. Order is l_front, l_middle, l_rear,
  // r_front, r_middle, r_rear.
  const std::array<float, 6> expect_coxa = {kSplayRad,  kSplayRad,  -kSplayRad,
                                            -kSplayRad, -kSplayRad, kSplayRad};
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    EXPECT_NEAR(pose[i][0], expect_coxa[i], 1e-6f) << g::LEG_NAMES[i];
    // Every group carries the same tip_reach here, and the reach is measured
    // from each leg's own coxa axis — so the radial reach, and with it the
    // femur/tibia pair, is identical for all six whatever the splay.
    EXPECT_NEAR(pose[i][1], pose[0][1], 1e-6f) << g::LEG_NAMES[i] << " femur";
    EXPECT_NEAR(pose[i][2], pose[0][2], 1e-6f) << g::LEG_NAMES[i] << " tibia";
  }
}

TEST(StandingPose, MiddleSplaySweepsThePairTheSameWay) {
  // A middle leg already points straight out, so there is no "outward" for it:
  // positive sweeps the pair rearward, mirrored about the fore/aft axis. Both
  // middle feet must therefore move to the same x, on opposite sides.
  ::hexa::config::StandingPose sp = hexa::config::kStandingPose;
  sp.groups[hexa::group_index(hexa::LegGroup::MIDDLE)].coxa = kSplayRad;
  const auto pose = g::standing_pose_from(hexa::config::kLegSpecs,
                                          hexa::config::kCoxaToBottom,
                                          hexa::config::kFootRadius, sp);
  const auto nominal = g::nominal_stance_from(hexa::config::kLegSpecs, pose);

  const g::Vec3 l = nominal.at("l_middle");
  const g::Vec3 r = nominal.at("r_middle");
  EXPECT_NEAR(l[0], r[0], 1e-5f) << "middle feet swept to different x";
  EXPECT_NEAR(l[1], -r[1], 1e-5f) << "middle feet not mirrored in y";
  EXPECT_LT(l[0], 0.0f) << "positive middle coxa_deg should sweep rearward";
}

TEST(StandingPose, BodyHeightIsMeasuredToTheContactPoint) {
  // The whole point of threading kFootRadius through: body_height is belly
  // clearance off the *floor*, and the floor is where the tip sphere touches —
  // one radius below the centre the IK was solved for. Without the offset every
  // leg stands foot_radius too tall, uniformly and invisibly.
  const auto pose = g::standing_pose_from(
      hexa::config::kLegSpecs, hexa::config::kCoxaToBottom,
      hexa::config::kFootRadius, hexa::config::kStandingPose);

  const float want = -(hexa::config::kCoxaToBottom +
                       hexa::config::kStandingPose.body_height);
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    const g::Vec3 centre =
        hexa::forward_kinematics(pose[i], hexa::config::kLegSpecs[i]);
    EXPECT_NEAR(centre[2] - hexa::config::kFootRadius, want, 1e-6f)
        << g::LEG_NAMES[i] << " contact point";
  }
}

TEST(StandingPose, ValuesSetTheFootprint) {
  const auto sp = standing_with_splay(kSplayDeg);
  const auto pose = g::standing_pose_from(
      hexa::config::kLegSpecs, hexa::config::kCoxaToBottom,
      hexa::config::kFootRadius, sp);
  const auto nominal = g::nominal_stance_from(hexa::config::kLegSpecs, pose);

  // The IK target is the foot sphere's centre, one radius above the contact
  // point the body_height is measured to.
  const float depth =
      hexa::config::kCoxaToBottom + sp.body_height - hexa::config::kFootRadius;
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    const auto& spec = hexa::config::kLegSpecs[i];
    const g::Vec3 foot = nominal.at(g::LEG_NAMES[i]);
    const g::Vec3 in_leg = hexa::body_to_leg(foot, spec);
    const auto group = hexa::leg_group(hexa::leg_from_index(static_cast<int>(i)));

    EXPECT_NEAR(std::hypot(in_leg[0], in_leg[1]),
                sp.groups[hexa::group_index(group)].tip_reach, 1e-5f)
        << g::LEG_NAMES[i] << " tip reach from its own coxa axis";
    EXPECT_NEAR(foot[2], -depth, 1e-5f) << g::LEG_NAMES[i] << " foot depth";
    EXPECT_NEAR(std::atan2(in_leg[1], in_leg[0]), pose[i][0], 1e-5f)
        << g::LEG_NAMES[i] << " heading is mount_yaw + coxa";

    // Swing-arc mirroring keys off the sign of the nominal y (identity_y_sign),
    // so a splay must never push a leg's nominal across the body centreline.
    const bool left = g::LEG_NAMES[i][0] == 'l';
    EXPECT_EQ(foot[1] > 0.0f, left) << g::LEG_NAMES[i] << " crossed y = 0";
  }
}

TEST(StandingPose, GroupsReachOutIndependently) {
  const auto sp = standing_with_group_reaches(0.130f, 0.145f, 0.155f);
  const auto pose = g::standing_pose_from(
      hexa::config::kLegSpecs, hexa::config::kCoxaToBottom,
      hexa::config::kFootRadius, sp);
  const auto nominal = g::nominal_stance_from(hexa::config::kLegSpecs, pose);

  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    const auto group = hexa::leg_group(hexa::leg_from_index(static_cast<int>(i)));
    EXPECT_NEAR(reach_in_leg_frame(nominal.at(g::LEG_NAMES[i]),
                                   hexa::config::kLegSpecs[i]),
                sp.groups[hexa::group_index(group)].tip_reach, 1e-5f)
        << g::LEG_NAMES[i];
  }

  // Reaching further out means a flatter leg, so femur/tibia now differ between
  // groups — the assumption that one leg's triple describes all six is gone.
  const std::size_t l_front = 0, l_middle = 1, l_rear = 2;
  EXPECT_NE(pose[l_front][1], pose[l_middle][1]);
  EXPECT_NE(pose[l_middle][1], pose[l_rear][1]);
  // ...but the two legs inside a group still match.
  const std::size_t r_front = 3;
  EXPECT_NEAR(pose[l_front][1], pose[r_front][1], 1e-6f);
  EXPECT_NEAR(pose[l_front][2], pose[r_front][2], 1e-6f);
}

TEST(StandingPose, RejectsASplayOutsideTheCoxaLimit) {
  // A splay past the coxa travel window must fail at load rather than strain a
  // servo. Derive the offending angle from the configured limit rather than
  // naming a number, so widening the window in geometry.yaml can never turn
  // this into a silent no-op.
  const float coxa_limit_deg =
      hexa::config::kJointLimits[0].upper * 180.0f / static_cast<float>(M_PI);
  EXPECT_THROW(g::standing_pose_from(hexa::config::kLegSpecs,
                                     hexa::config::kCoxaToBottom,
                                     hexa::config::kFootRadius,
                                     standing_with_splay(coxa_limit_deg + 15.0f)),
               std::invalid_argument);
}

TEST(Reseat, PreservesEachLegsSwivel) {
  const auto sp = standing_with_splay(kSplayDeg);
  const auto pose = g::standing_pose_from(
      hexa::config::kLegSpecs, hexa::config::kCoxaToBottom,
      hexa::config::kFootRadius, sp);
  const auto nominal = g::nominal_stance_from(hexa::config::kLegSpecs, pose);
  const auto geometry = g::reseat_geometry_from(hexa::config::kLegSpecs, pose);
  const auto specs = g::leg_specs_from(hexa::config::kLegSpecs);

  // At the pose's own height the reseat target is the nominal stance itself.
  const auto same = g::reseat_nominal_stance(0.0f, geometry, specs, nominal);
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    const g::Vec3 a = same.at(g::LEG_NAMES[i]);
    const g::Vec3 b = nominal.at(g::LEG_NAMES[i]);
    EXPECT_NEAR(a[0], b[0], 1e-5f) << g::LEG_NAMES[i] << ".x";
    EXPECT_NEAR(a[1], b[1], 1e-5f) << g::LEG_NAMES[i] << ".y";
    EXPECT_NEAR(a[2], b[2], 1e-5f) << g::LEG_NAMES[i] << ".z";
  }

  // Raising the body widens the footprint (the tibia lean is held), but the
  // splay survives: every leg keeps the azimuth it was standing at.
  const auto raised = g::reseat_nominal_stance(0.03f, geometry, specs, nominal);
  bool widened = false;
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    const auto& spec = hexa::config::kLegSpecs[i];
    const g::Vec3 was = hexa::body_to_leg(nominal.at(g::LEG_NAMES[i]), spec);
    const g::Vec3 now = hexa::body_to_leg(raised.at(g::LEG_NAMES[i]), spec);
    EXPECT_NEAR(std::atan2(now[1], now[0]), std::atan2(was[1], was[0]), 1e-5f)
        << g::LEG_NAMES[i] << " swivelled during the reseat";
    if (std::hypot(now[0], now[1]) > std::hypot(was[0], was[1]) + 1e-4f) {
      widened = true;
    }
  }
  EXPECT_TRUE(widened) << "raising the body should have moved the feet out";
}

TEST(Reseat, PreservesEachGroupsReach) {
  // A height change re-solves the radius each foot stands at. That solve reads
  // the leg's femur/tibia lean, which now differs between groups — so it has to
  // run per leg. Sharing one leg's snapshot would drag all six onto a single
  // radius and quietly flatten the configured stance.
  const auto sp = standing_with_group_reaches(0.130f, 0.145f, 0.155f);
  const auto pose = g::standing_pose_from(
      hexa::config::kLegSpecs, hexa::config::kCoxaToBottom,
      hexa::config::kFootRadius, sp);
  const auto nominal = g::nominal_stance_from(hexa::config::kLegSpecs, pose);
  const auto geometry = g::reseat_geometry_from(hexa::config::kLegSpecs, pose);
  const auto specs = g::leg_specs_from(hexa::config::kLegSpecs);

  const auto raised = g::reseat_nominal_stance(0.02f, geometry, specs, nominal);

  const auto reach = [&](const char* name) {
    for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
      if (g::LEG_NAMES[i] == name) {
        return reach_in_leg_frame(raised.at(name), hexa::config::kLegSpecs[i]);
      }
    }
    ADD_FAILURE() << "unknown leg " << name;
    return 0.0f;
  };

  // The configured ordering front < middle < rear survives the reseat, and the
  // groups stay apart by roughly what they started apart by.
  EXPECT_LT(reach("l_front"), reach("l_middle"));
  EXPECT_LT(reach("l_middle"), reach("l_rear"));
  EXPECT_NEAR(reach("l_front"), reach("r_front"), 1e-5f);
  EXPECT_NEAR(reach("l_rear"), reach("r_rear"), 1e-5f);
  // The specific failure of a shared solve: every leg landing on one radius.
  EXPECT_GT(reach("l_rear") - reach("l_front"), 0.015f);
}

TEST(Reseat, RejectsAHeightInfeasibleForAnyGroup) {
  // One group out of range aborts the whole reseat rather than re-planting five
  // legs and stranding the sixth. Push the rear legs out far enough that they
  // run out of femur travel first, then ask for a height only they cannot make.
  const auto sp = standing_with_group_reaches(0.130f, 0.130f, 0.150f);
  const auto pose = g::standing_pose_from(
      hexa::config::kLegSpecs, hexa::config::kCoxaToBottom,
      hexa::config::kFootRadius, sp);
  const auto nominal = g::nominal_stance_from(hexa::config::kLegSpecs, pose);
  const auto geometry = g::reseat_geometry_from(hexa::config::kLegSpecs, pose);
  const auto specs = g::leg_specs_from(hexa::config::kLegSpecs);

  // Find a lift the front group can still reach but the rear group cannot.
  bool found = false;
  for (float lift = 0.01f; lift < 0.30f && !found; lift += 0.005f) {
    const auto front_only = g::reseat_geometry_from(
        hexa::config::kLegSpecs,
        g::standing_pose_from(hexa::config::kLegSpecs,
                              hexa::config::kCoxaToBottom,
                              hexa::config::kFootRadius,
                              standing_with_group_reaches(0.130f, 0.130f,
                                                          0.130f)));
    bool front_ok = true;
    try {
      g::reseat_nominal_stance(lift, front_only, specs, nominal);
    } catch (const std::invalid_argument&) {
      front_ok = false;
    }
    if (!front_ok) {
      continue;  // nothing can make this height; keep looking lower
    }
    try {
      g::reseat_nominal_stance(lift, geometry, specs, nominal);
    } catch (const std::invalid_argument&) {
      found = true;  // the rear group is what failed
    }
  }
  EXPECT_TRUE(found)
      << "expected a height the front group can reach and the rear cannot";
}

namespace {

std::unique_ptr<g::ReseatController> make_reseat(
    const std::map<std::string, g::Vec3>& current,
    const std::map<std::string, g::Vec3>& target) {
  const auto cfg = g::engine_config_from_config();
  return std::make_unique<g::ReseatController>(
      current, target, cfg.reseat_pair_swing_time, cfg.reseat_pair_dwell_time,
      cfg.reseat_profile(), cfg.controller_dt);
}

int ticks_to_done(g::ReseatController& r, float dt) {
  int ticks = 0;
  for (; ticks < 4000 && !r.done(); ++ticks) {
    r.update(dt);
  }
  return ticks;
}

}  // namespace

// An abandoned engagement hands the ladder feet that are still in the air. Those
// carry no weight, so the ladder must not count them as support, and must put
// them down before it lifts anything that is standing — otherwise the pair it
// lifts plus the pair it is hanging leaves too few feet loaded and the body
// tilts onto the gap.
TEST(Reseat, LandsAirborneFeetBeforeLiftingAnyThatAreDown) {
  constexpr float kTickDt = 0.005f;
  const auto cfg = g::engine_config_from_config();
  const auto nominal = g::nominal_stance_from_config();
  // The crawl case: the two legs whose phase offsets put them in the air at the
  // start of an engagement, caught mid-swing.
  const std::array<std::string, 2> airborne = {"l_middle", "r_rear"};

  auto current = nominal;
  for (const auto& n : g::LEG_NAMES) {
    current[n] = nominal.at(n) + g::Vec3(-0.04f, 0.0f, 0.0f);
  }
  for (const auto& n : airborne) {
    current[n] = nominal.at(n) + g::Vec3(0.02f, 0.01f, cfg.step_height);
  }

  auto r = make_reseat(current, nominal);
  ASSERT_FALSE(r->done());

  int first_liftoff_tick = -1;
  int landed_tick = -1;
  float worst_rise = 0.0f;
  int fewest_loaded = 6;
  for (int i = 0; i < 4000 && !r->done(); ++i) {
    const auto out = r->update(kTickDt);

    int loaded = 0;
    bool grounded_lifted = false;
    bool all_landed = true;
    for (const auto& [name, leg] : out) {
      const bool down =
          leg.foot_target[2] <= nominal.at(name)[2] + cfg.touchdown_probe_height;
      if (leg.stance) {
        EXPECT_TRUE(down) << name << " reported stance while off the ground";
        ++loaded;
      }
      const bool is_airborne_leg =
          std::find(airborne.begin(), airborne.end(), name) != airborne.end();
      if (is_airborne_leg) {
        if (!down) all_landed = false;
        worst_rise =
            std::max(worst_rise, leg.foot_target[2] - current.at(name)[2]);
      } else if (!down) {
        grounded_lifted = true;
      }
    }
    fewest_loaded = std::min(fewest_loaded, loaded);
    if (all_landed && landed_tick < 0) landed_tick = i;
    if (grounded_lifted && first_liftoff_tick < 0) first_liftoff_tick = i;
  }

  EXPECT_TRUE(r->done()) << "the ladder never finished";
  EXPECT_GE(fewest_loaded, 3) << "statically unstable";
  ASSERT_GE(landed_tick, 0) << "the airborne feet never came down";
  ASSERT_GE(first_liftoff_tick, 0) << "no grounded foot was ever re-planted";
  EXPECT_LT(landed_tick, first_liftoff_tick)
      << "a standing foot lifted before the airborne ones had landed";
  // A landing is a descent: swing_arc's ease5 lift and ease7 base do not cancel
  // exactly, so a few millimetres of bulge is expected; a climb is not.
  EXPECT_LT(worst_rise, 0.005f) << "a landing foot climbed above where it started";

  for (const auto& n : g::LEG_NAMES) {
    const auto out = r->update(kTickDt);
    EXPECT_LT((out.at(n).foot_target - nominal.at(n)).norm(), 1e-5f)
        << n << " did not finish on its nominal stance";
  }
}

// The landing stage has to cost nothing when it is not needed: settle_beats_reseat
// and the stop budget both price the ladder at three pair swings and two dwells,
// and both routes that reach it with six feet planted must still pay exactly that.
TEST(Reseat, CostsNothingWhenEveryFootIsAlreadyDown) {
  constexpr float kTickDt = 0.005f;
  const auto cfg = g::engine_config_from_config();
  const auto nominal = g::nominal_stance_from_config();
  const auto swing_ticks = static_cast<int>(cfg.reseat_pair_swing_time / kTickDt);
  const auto dwell_ticks = static_cast<int>(cfg.reseat_pair_dwell_time / kTickDt);

  EXPECT_TRUE(make_reseat(nominal, nominal)->done())
      << "a stance already on target should need no ladder at all";

  // Displaced in the ground plane only — the case every settle hands over.
  auto scattered = nominal;
  for (const auto& n : g::LEG_NAMES) {
    scattered[n] = nominal.at(n) + g::Vec3(-0.05f, 0.0f, 0.0f);
  }
  auto all_six = make_reseat(scattered, nominal);
  EXPECT_NEAR(ticks_to_done(*all_six, kTickDt),
              3 * swing_ticks + 2 * dwell_ticks, 8);

  auto one_pair = nominal;
  one_pair["l_front"] = nominal.at("l_front") + g::Vec3(-0.05f, 0.0f, 0.0f);
  one_pair["r_rear"] = nominal.at("r_rear") + g::Vec3(-0.05f, 0.0f, 0.0f);
  auto skipped = make_reseat(one_pair, nominal);
  EXPECT_NEAR(ticks_to_done(*skipped, kTickDt), swing_ticks, 8);
}

TEST(Limits, ScaleToEnvelopeIsNoOpWhenInRange) {
  const auto nominal = g::nominal_stance_from_config();
  const auto caps =
      g::load_velocity_caps_from_config(g::outer_stance_radius(nominal));
  EXPECT_GT(caps.linear_max("tripod"), 0.0f);
  auto [vx, vy, wz] = g::scale_to_envelope(0.01f, 0.0f, 0.0f, nominal,
                                           caps.linear_max("tripod"), 0.6f);
  EXPECT_NEAR(vx, 0.01f, 1e-6f);
  EXPECT_NEAR(vy, 0.0f, 1e-6f);
  EXPECT_NEAR(wz, 0.0f, 1e-6f);
}

// The IK/FK round trip must land on the closed form hexa_common/limits.py
// implements (standing_stance_xy): the tip sits its group's tip_reach out from
// the coxa axis at the leg's splay, rotated into the body frame and offset by
// the mount. Teleop derives its angular stick cap from that closed form while
// the pipeline derives the envelope from this FK path, so a divergence here
// would silently desynchronise the stick from the engine.
//
// Run on a stance whose groups differ in both reach and splay, so the check
// covers the sign rule rather than passing on a symmetric special case.
TEST(Limits, StanceMatchesTheClosedFormTeleopUses) {
  ::hexa::config::StandingPose sp =
      standing_with_group_reaches(0.130f, 0.145f, 0.155f);
  sp.groups[hexa::group_index(hexa::LegGroup::FRONT)].coxa = kSplayRad;
  sp.groups[hexa::group_index(hexa::LegGroup::MIDDLE)].coxa = 0.0f;
  sp.groups[hexa::group_index(hexa::LegGroup::REAR)].coxa = -kSplayRad;

  const auto pose = g::standing_pose_from(
      hexa::config::kLegSpecs, hexa::config::kCoxaToBottom,
      hexa::config::kFootRadius, sp);
  const auto nominal = g::nominal_stance_from(hexa::config::kLegSpecs, pose);

  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    const auto leg = hexa::leg_from_index(static_cast<int>(i));
    const auto group = hexa::leg_group(leg);
    const auto& grp = sp.groups[hexa::group_index(group)];
    // The closed form's sign rule, spelled out rather than shared with the code
    // under test: rear negates, right negates.
    const float sign = (group == hexa::LegGroup::REAR ? -1.0f : 1.0f) *
                       (hexa::leg_is_right(leg) ? -1.0f : 1.0f);

    const auto& spec = hexa::config::kLegSpecs[i];
    const float angle = spec.mount_yaw + sign * grp.coxa;
    const float want_x = spec.mount_xyz[0] + grp.tip_reach * std::cos(angle);
    const float want_y = spec.mount_xyz[1] + grp.tip_reach * std::sin(angle);

    const g::Vec3& got = nominal.at(g::LEG_NAMES[i]);
    EXPECT_NEAR(got[0], want_x, 1e-5f) << g::LEG_NAMES[i] << " x";
    EXPECT_NEAR(got[1], want_y, 1e-5f) << g::LEG_NAMES[i] << " y";
  }
}

// The outer stance radius is the corner feet, and it is what turns the linear
// cap into the angular one. No YAML knob is involved.
TEST(Limits, OuterStanceRadiusIsTheFurthestFoot) {
  const auto nominal = g::nominal_stance_from_config();
  const float r = g::outer_stance_radius(nominal);
  float expected = 0.0f;
  for (const auto& [name, p] : nominal) {
    (void)name;
    expected = std::max(expected, std::hypot(p[0], p[1]));
  }
  EXPECT_NEAR(r, expected, 1e-6f);
  // Comfortably outside the mount ring — this is the whole reason the yaw lever
  // arm had to move from mount_xyz to nominal_stance.
  float max_mount = 0.0f;
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    const auto& m = hexa::config::kLegSpecs[i].mount_xyz;
    max_mount = std::max(max_mount, std::hypot(m[0], m[1]));
  }
  EXPECT_GT(r, max_mount * 1.5f);
}

TEST(Limits, OuterStanceRadiusRejectsADegenerateStance) {
  std::map<std::string, g::Vec3> on_axis;
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    on_axis[g::LEG_NAMES[i]] = g::Vec3(0.0f, 0.0f, -0.05f);
  }
  EXPECT_THROW(g::outer_stance_radius(on_axis), std::invalid_argument);
}

// Pure yaw past the envelope lands on linear_max / r_outer — the derived cap —
// with no separate angular clamp in the path.
TEST(Limits, ScaleToEnvelopeBoundsPureYawAtTheStanceRadius) {
  const auto nominal = g::nominal_stance_from_config();
  const float r_outer = g::outer_stance_radius(nominal);
  const auto caps = g::load_velocity_caps_from_config(r_outer);
  for (const float commanded : {2.0f, 6.0f, 50.0f}) {
    auto [vx, vy, wz] = g::scale_to_envelope(
        0.0f, 0.0f, commanded, nominal, caps.linear_max("tripod"),
        caps.yaw_bias("tripod"));
    EXPECT_NEAR(wz, caps.angular_max("tripod"), 1e-4f)
        << "commanded " << commanded;
    EXPECT_NEAR(vx, 0.0f, 1e-6f);
    EXPECT_NEAR(vy, 0.0f, 1e-6f);
  }
}

// The engine plans the stride at the foot, so a commanded yaw has to come out
// of per_leg_planar_velocity as the foot's tangential speed. Using the mount
// would understate it by |r_mount| / |r_stance|.
TEST(Limits, PerLegYawVelocityUsesTheStanceRadius) {
  const auto contexts = g::build_leg_contexts_from_config();
  const float omega = 0.5f;
  const auto vels = g::per_leg_planar_velocity(contexts, {0.0f, 0.0f}, omega);
  for (const auto& [name, v] : vels) {
    const g::Vec3& stance = contexts.at(name).nominal_stance;
    const float r = std::hypot(stance[0], stance[1]);
    EXPECT_NEAR(std::hypot(v.first, v.second), omega * r, 1e-5f)
        << name << " yaw lever arm is not its stance radius";
  }
}
