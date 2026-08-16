// What the legs do when the command turns under them. Two independent defects,
// measured separately:
//
// **Swing skid.** Through the probe the arc *is* the touchdown ground line, so
// the foot's ground velocity equals the target's own motion one for one, and a
// command ramping through low speed slides the live AEP faster than the robot
// walks. SwingPlanner::retarget bounds it; these measure what is left.
//
// **Stance runway.** A leg that just landed on the old AEP has no excursion left
// in the new direction and pins against the stance ceiling. The reversal ladder
// reflects the phase circle to hand it a fresh one, which is only sound where the
// feet are where the schedule says, so these measure the credit directly.

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "gait/gaits/registry.hpp"

namespace g = hexa::gait;

namespace {

constexpr float kDt = 0.005f;

// The saturating command for a gait — the cap pipeline_config_loader derives and
// scale_to_envelope enforces. Mirrors test_gait_unit's copy; both are standalone
// translation units.
float saturating_speed(const g::EngineConfig& cfg, float duty_factor) {
  const float swing_end = g::swing_end_phase(duty_factor, cfg.swing_phase_margin);
  return cfg.stride_length * swing_end /
         (cfg.min_swing_time * (1.0f - swing_end));
}

// One tick of BodyVelocityLimiter's slew, so what these tests drive is the turn
// the robot actually sees. Single-axis, so the vector slew reduces to a scalar.
float slew_toward(float v, float target, float accel, float dt) {
  const float step = accel * dt;
  const float delta = target - v;
  return std::fabs(delta) <= step ? target : v + std::copysign(step, delta);
}

// Share of the swing counted as the landing. The ride parks the foot over its
// landing point for the last `ride_time`, and where the grant tapers to nothing
// the blend's own tail arrives with three vanishing derivatives instead — so on
// a steady command the foot is world-frame stationary through here either way,
// whatever the speed. Anything it does move is slip.
constexpr float kLandingShare = 0.1f;

struct SkidStats {
  // Worst single landing, in world-frame metres travelled over the ground.
  float peak_landing_slip = 0.0f;
  // Same, over the whole probe band rather than just the landing.
  float peak_probe_slip = 0.0f;
  // Worst single stance, likewise. A planted foot that is tracking the ground is
  // world-frame still; one whose anchor has pinned against the stance ceiling
  // travels at the full ground speed, and this is how far it drags.
  float peak_stance_drag = 0.0f;
  // Worst runway the reflection handed a leg over what its foot position
  // afforded, as a fraction of one stance. Positive means the schedule promised
  // travel the leg does not have and it will plough into its ceiling.
  float worst_over_credit = 0.0f;
  std::string worst_leg;
  std::string worst_stance_leg;
  int landings = 0;
  int mirrors = 0;
  // What the engine did underneath the ladder's hold. The hold is a walk, not a
  // released stick, so the engine must stay in GAIT through it; and once the
  // command has come down onto the knee the clock must run no faster than the
  // walk's own longest cycle, which is the rate the knee is defined by.
  int hold_ticks = 0;
  int hold_ticks_out_of_gait = 0;
  float worst_hold_phase_step = 0.0f;
};

// What the command does once the gait is steady.
enum class Turn {
  kHold,     // the steady-walk residual everything else is judged against
  kReverse,  // straight to the opposite sign
  kStop,     // to zero — the same AEP slide, half as far
};

// Walk along `axis` (0 = fore/aft, 1 = lateral) at the saturating command until
// steady, then turn it; `flip_offset` shifts the turn within the cycle so a sweep
// covers every phase a foot can be caught in. The world frame comes from
// integrating the command, since foot-derived odometry would be polluted by the
// very slip being measured.
SkidStats drive_skid(const std::string& gait, float flip_offset, Turn turn,
                     int axis, bool instant = false, bool ladder = false) {
  const auto cfg = g::engine_config_from_config();
  const auto nominal = g::nominal_stance_from_config();
  auto e = g::make_default_engine(gait);
  const float duty = g::strategies().at(gait)()->duty_factor();
  const float swing_end = g::swing_end_phase(duty, cfg.swing_phase_margin);
  // The stride this axis can actually lay down, and the command derated to
  // match. drive_skid drives the engine directly, so Control::shape is not in
  // the loop to cut the cap; without this the lateral axis is commanded travel
  // the gait has no stride for, and the skid being measured is that mismatch
  // rather than the turn.
  const std::pair<float, float> axis_dir =
      axis == 0 ? std::pair<float, float>{1.0f, 0.0f}
                : std::pair<float, float>{0.0f, 1.0f};
  const float stride = g::effective_stride_length(
      g::build_leg_contexts_from_config(), axis_dir, 0.0f, cfg.stride_length,
      cfg.stride_length_radial);
  const float speed =
      saturating_speed(cfg, duty) * (stride / cfg.stride_length);
  const float accel =
      instant ? std::numeric_limits<float>::max()
              : speed / hexa::config::kControl.vmax_ramp_time_linear;
  const float probe_band = cfg.swing_profile().probe_band(cfg.max_swing_time);

  for (int i = 0; i < 6000 && e->state() != g::EngineState::STAND; ++i) {
    e->update(kDt, {0.0f, 0.0f}, 0.0f);
    e->start_initialize();
  }
  EXPECT_EQ(e->state(), g::EngineState::STAND);

  // The knee this axis holds at — where derive_cycle_time stops stretching the
  // cycle, so the clock is at its slowest and any settle blend can only speed it
  // up. Same formula the engine derives it by, off the same derated stride.
  const float knee =
      stride * swing_end / (cfg.max_swing_time * (1.0f - swing_end));

  float v = 0.0f;
  hexa::Vec3 body_w = hexa::Vec3::Zero();
  std::map<std::string, hexa::Vec3> last_targets;
  bool holding = false;
  const auto tick = [&](float target) {
    std::pair<float, float> request =
        axis == 0 ? std::pair<float, float>{target, 0.0f}
                  : std::pair<float, float>{0.0f, target};
    if (ladder) {
      const auto [rx, ry, rw] = e->shape_reversal(kDt, request, 0.0f);
      // The gate holds by handing back a command other than the one asked for.
      holding = rx != request.first || ry != request.second;
      request = {rx, ry};
      (void)rw;
    }
    // Single-axis, so the shaper's vector slew is a scalar one on that axis.
    v = slew_toward(v, axis == 0 ? request.first : request.second, accel, kDt);
    const std::pair<float, float> cmd =
        axis == 0 ? std::pair<float, float>{v, 0.0f}
                  : std::pair<float, float>{0.0f, v};
    body_w.x += cmd.first * kDt;
    body_w.y += cmd.second * kDt;
    auto out = e->update(kDt, cmd, 0.0f);
    for (const auto& [name, leg] : out) last_targets[name] = leg.foot_target;
    return out;
  };

  for (int i = 0; i < 6000 && e->state() != g::EngineState::GAIT; ++i) {
    tick(speed);
  }
  EXPECT_EQ(e->state(), g::EngineState::GAIT);
  for (int i = 0; i < 600; ++i) tick(speed);
  for (int i = 0; i < static_cast<int>(flip_offset / kDt); ++i) tick(speed);

  struct LegTrace {
    hexa::Vec3 world = hexa::Vec3::Zero();
    bool have = false;
    bool swing = false;
    float landing_slip = 0.0f;
    float probe_slip = 0.0f;
    float stance_drag = 0.0f;
  };
  std::map<std::string, LegTrace> trace;
  float prev_master = e->master_phase();
  g::EngineState prev_state = e->state();

  const float turned = turn == Turn::kReverse  ? -speed
                       : turn == Turn::kStop ? 0.0f
                                             : speed;
  SkidStats s;
  for (int i = 0; i < 1600; ++i) {
    const auto before = last_targets;
    const auto out = tick(turned);

    // The clock only ever advances by dt / cycle_time, so a step this large
    // inside GAIT is the reflection — the one discontinuity the master phase has.
    const float step = std::fabs(e->master_phase() - prev_master);
    const bool walked = prev_state == g::EngineState::GAIT &&
                        e->state() == g::EngineState::GAIT;
    prev_master = e->master_phase();
    prev_state = e->state();

    if (holding) {
      ++s.hold_ticks;
      if (e->state() != g::EngineState::GAIT) ++s.hold_ticks_out_of_gait;
      // Only once the limiter has brought the command down onto the knee: while
      // it is still above, the walk legitimately runs a shorter cycle. The
      // reflection is never the step measured here — the gate hands the request
      // straight back on the tick it fires, so that tick is not a hold.
      if (std::fabs(v) <= knee * 1.05f) {
        s.worst_hold_phase_step =
            std::max(s.worst_hold_phase_step, step > 0.5f ? 1.0f - step : step);
      }
    }

    if (walked && step > 0.1f && step < 0.9f) {
      ++s.mirrors;
      // What the reflection just promised each leg, against what the ground
      // under its foot can actually give: remaining stance, in strides, versus
      // the distance to its PEP in the direction the robot is about to travel.
      for (const auto& [name, leg] : out) {
        const float progress = (leg.phase - swing_end) / (1.0f - swing_end);
        const float remaining = leg.stance ? 1.0f - progress : 0.0f;
        const hexa::Vec3 e_foot = before.at(name) - nominal.at(name);
        // Excursion resolved along the travel being turned to. A planted foot
        // drifts against that travel, from its AEP at +L/2 to its PEP at -L/2,
        // so this is how much of the drift it has left.
        const float along = (axis == 0 ? e_foot.x : e_foot.y) *
                            (turned < 0.0f ? -1.0f : 1.0f);
        const float runway = (along + 0.5f * stride) / stride;
        s.worst_over_credit = std::max(s.worst_over_credit, remaining - runway);
      }
    }

    for (const auto& [name, leg] : out) {
      LegTrace& t = trace[name];
      const hexa::Vec3 world(leg.foot_target.x + body_w.x,
                             leg.foot_target.y + body_w.y, leg.foot_target.z);
      const bool swing = !leg.stance;
      if (swing && t.have && t.swing) {
        const float step = std::hypot(world.x - t.world.x, world.y - t.world.y);
        const float phase_in_swing =
            swing_end > 0.0f ? leg.phase / swing_end : 0.0f;
        if (leg.foot_target.z - nominal.at(name).z <= probe_band) {
          t.probe_slip += step;
          if (phase_in_swing >= 1.0f - kLandingShare) t.landing_slip += step;
        }
      }
      if (!swing && t.have && !t.swing) {
        t.stance_drag +=
            std::hypot(world.x - t.world.x, world.y - t.world.y);
      }
      if (swing && t.have && !t.swing) {  // lift-off edge: bank the stance
        if (t.stance_drag > s.peak_stance_drag) {
          s.peak_stance_drag = t.stance_drag;
          s.worst_stance_leg = name;
        }
        t.stance_drag = 0.0f;
      }
      if (!swing && t.swing) {  // touchdown edge: bank the swing just finished
        ++s.landings;
        if (t.landing_slip > s.peak_landing_slip) {
          s.peak_landing_slip = t.landing_slip;
          s.worst_leg = name;
        }
        s.peak_probe_slip = std::max(s.peak_probe_slip, t.probe_slip);
        t.landing_slip = 0.0f;
        t.probe_slip = 0.0f;
      }
      t.world = world;
      t.swing = swing;
      t.have = true;
    }
  }
  return s;
}

}  // namespace

// The headline regression for the touchdown-target rate bound: sweeping the turn
// across the cycle catches a foot at every swing phase, the probe included, where
// an unbounded retarget drags it the whole way the live AEP moves.
//
// The landing bound is the probe band's own height. The probe bound is coarse by
// comparison — most of what is left there is the ground line's own collapse under
// deceleration, not the target — so it is pinned only well clear of a stride,
// which is where the unbounded version lands.
TEST(Reversal, LandingDoesNotDragAcrossTheGround) {
  const auto cfg = g::engine_config_from_config();
  const float cycle =
      cfg.max_swing_time / g::swing_end_phase(0.5f, cfg.swing_phase_margin);
  const float band = cfg.swing_profile().probe_band(cfg.max_swing_time);

  for (int axis = 0; axis < 2; ++axis) {
    for (int i = 0; i < 12; ++i) {
      const float offset = cycle * static_cast<float>(i) / 12.0f;
      for (const Turn turn : {Turn::kReverse, Turn::kStop}) {
        for (const bool instant : {false, true}) {
          const SkidStats s = drive_skid("tripod", offset, turn, axis, instant);
          ASSERT_GE(s.landings, 6);
          const std::string where =
              "axis " + std::to_string(axis) + " offset " +
              std::to_string(offset) +
              (turn == Turn::kReverse ? " reverse" : " stop") +
              (instant ? " instant: " : " slewed: ") + s.worst_leg;
          EXPECT_LT(s.peak_landing_slip, band)
              << where << " dragged " << s.peak_landing_slip * 1000.0f
              << " mm on landing";
          EXPECT_LT(s.peak_probe_slip, 0.5f * cfg.stride_length)
              << where << " dragged " << s.peak_probe_slip * 1000.0f
              << " mm through the probe";
        }
      }
    }
  }
}

// ── The reversal ladder ──

namespace {

// Worst case over a full sweep of turn offsets and both travel axes.
SkidStats sweep(const std::string& gait, bool ladder) {
  const auto cfg = g::engine_config_from_config();
  const float duty = g::strategies().at(gait)()->duty_factor();
  const float cycle =
      cfg.max_swing_time / g::swing_end_phase(duty, cfg.swing_phase_margin);
  SkidStats w;
  for (int axis = 0; axis < 2; ++axis) {
    for (int i = 0; i < 12; ++i) {
      const SkidStats s = drive_skid(gait, cycle * static_cast<float>(i) / 12.0f,
                                     Turn::kReverse, axis, false, ladder);
      if (s.peak_stance_drag > w.peak_stance_drag) {
        w.peak_stance_drag = s.peak_stance_drag;
        w.worst_stance_leg = s.worst_stance_leg;
      }
      w.worst_over_credit = std::max(w.worst_over_credit, s.worst_over_credit);
      w.mirrors += s.mirrors;
      w.hold_ticks += s.hold_ticks;
      w.hold_ticks_out_of_gait += s.hold_ticks_out_of_gait;
      w.worst_hold_phase_step =
          std::max(w.worst_hold_phase_step, s.worst_hold_phase_step);
    }
  }
  return w;
}

}  // namespace

// The reflection is only as good as its premise: that a foot is where its stance
// progress says. Reflecting a walk bled to a standstill breaks exactly that —
// below the knee the clock outruns the travel and the feet bunch toward nominal
// (24-29 mm off at 40% of knee speed), so every leg is handed runway it does not
// have. The load-bearing test, not the drag one below: it asserts the credit at
// the instant the reflection happens.
TEST(Reversal, MirrorNeverCreditsALegMoreRunwayThanItHas) {
  for (const char* gait : {"tripod", "tetrapod", "ripple"}) {
    const SkidStats s = sweep(gait, /*ladder=*/true);
    EXPECT_GT(s.mirrors, 0) << gait << " never reflected, so nothing was tested";
    // One tick of stance travel is the floor any measurement like this has.
    EXPECT_LT(s.worst_over_credit, 0.02f)
        << gait << " was credited " << s.worst_over_credit * 100.0f
        << "% of a stance more than its foot position affords";
  }
}

// The mechanism the test above rests on, asserted directly. On a slow gait's
// derated axis the ladder's hold is *slower* than cmd_zero_tol (ripple's lateral
// knee is 0.0183 m/s against a 0.02 tolerance), and read as a release it would
// decay cmd_gain_ — feet slow, clock speeds up, feet bunch toward nominal — and
// eventually drop the engine out of GAIT mid-ladder without ever reflecting.
//
// So: through the hold the engine keeps walking, and on the knee the clock runs
// no faster than max_cycle_time. Both bounds are read off the tuning, so a retune
// that moves the knee across the tolerance fails here instead of quietly bunching
// the feet.
TEST(Reversal, TheLaddersHoldIsNotReadAsAReleasedStick) {
  const auto cfg = g::engine_config_from_config();
  for (const char* gait : {"tripod", "tetrapod", "ripple"}) {
    const SkidStats s = sweep(gait, /*ladder=*/true);
    const float swing_end = g::swing_end_phase(
        g::strategies().at(gait)()->duty_factor(), cfg.swing_phase_margin);
    const float walk_step = kDt / (cfg.max_swing_time / swing_end);

    ASSERT_GT(s.hold_ticks, 0) << gait << " never held, so nothing was tested";
    EXPECT_EQ(s.hold_ticks_out_of_gait, 0)
        << gait << " stopped walking under the ladder's hold";
    // One tick's worth of slack: the limiter lands on the knee asymptotically,
    // so the first tick inside the window can still be a hair above it.
    EXPECT_LT(s.worst_hold_phase_step, walk_step * 1.05f)
        << gait << " ran its clock " << s.worst_hold_phase_step / walk_step
        << "x the walk's own rate while holding at the knee";
  }
}

// What the reflection buys: the legs that landed most recently get their runway
// back instead of pinning against the ceiling. Worst stance drag over 12 turn
// offsets on both axes, on the baked config:
//
//   tripod    48.1 mm -> 31.9 mm
//   tetrapod  64.5 mm -> 33.0 mm
//   ripple    75.0 mm -> 36.7 mm
//
// What is left is the stopping distance from the knee (~22 mm here) against the
// stance band's 12.5 mm of grace: no reflection can give back ground the robot
// has yet to stop covering.
TEST(Reversal, LadderHandsTheStanceLegsTheirRunwayBack) {
  for (const char* gait : {"tripod", "tetrapod", "ripple"}) {
    const SkidStats bare = sweep(gait, /*ladder=*/false);
    const SkidStats led = sweep(gait, /*ladder=*/true);
    EXPECT_GT(bare.peak_stance_drag, 0.02f)
        << gait << ": nothing to fix, the measurement must be wrong";
    EXPECT_LT(led.peak_stance_drag, 0.75f * bare.peak_stance_drag)
        << gait << ": " << led.worst_stance_leg << " still dragged "
        << led.peak_stance_drag * 1000.0f << " mm, against "
        << bare.peak_stance_drag * 1000.0f << " mm without the ladder";
  }
}

// Crawl and surf swing end to end and never have all six feet down, so there is
// no instant at which the reflection is valid and the ladder must not hold them
// up pretending otherwise: they reverse exactly as they did before.
TEST(Reversal, GaitsWithoutAnAllDownWindowAreLeftAlone) {
  for (const char* gait : {"crawl", "surf"}) {
    const SkidStats bare = sweep(gait, /*ladder=*/false);
    const SkidStats led = sweep(gait, /*ladder=*/true);
    EXPECT_EQ(led.mirrors, 0) << gait << " reflected without an all-down window";
    EXPECT_FLOAT_EQ(led.peak_stance_drag, bare.peak_stance_drag)
        << gait << " was disturbed by a ladder that cannot help it";
  }
}

// ── The reflection itself ──

TEST(Reversal, MirrorInvertsEveryStanceLegsProgress) {
  const auto cfg = g::engine_config_from_config();
  for (const auto& [gait, make] : g::strategies()) {
    const auto strategy = make();
    const float end =
        g::swing_end_phase(strategy->duty_factor(), cfg.swing_phase_margin);
    for (int i = 0; i < 200; ++i) {
      const float master = static_cast<float>(i) / 200.0f;
      g::GaitClock clock(strategy->phase_offsets());
      clock.reset(master);
      const auto before = clock.phases();
      clock.mirror(end);
      const auto after = clock.phases();
      for (const auto& [leg, was] : before) {
        const float now = after.at(leg);
        // Compared the long way round: phase 0 and phase 1 are the same place,
        // and the reflection of a phase a hair below zero lands on that seam.
        const float gap = std::fabs(now - g::pymod(end - was, 1.0f));
        EXPECT_LT(std::min(gap, 1.0f - gap), 1.0e-5f)
            << gait << " " << leg << " at master " << master;
        if (was <= end + 1.0e-4f) continue;  // airborne: no stance progress
        EXPECT_GT(now, end) << gait << " " << leg << " left stance on a mirror";
        const float progress = (was - end) / (1.0f - end);
        EXPECT_NEAR((now - end) / (1.0f - end), 1.0f - progress, 1.0e-4f)
            << gait << " " << leg;
      }
    }
  }
}

TEST(Reversal, MirroringTwiceRestoresTheClock) {
  for (const auto& [gait, make] : g::strategies()) {
    const auto strategy = make();
    g::GaitClock clock(strategy->phase_offsets());
    clock.reset(0.37f);
    const auto before = clock.phases();
    clock.mirror(0.44f);
    clock.mirror(0.44f);
    for (const auto& [leg, was] : before) {
      EXPECT_NEAR(clock.phases().at(leg), was, 1.0e-5f) << gait << " " << leg;
    }
  }
}

// A tripod's table is its own negation; the gaits that carry a wave down the
// body send it the other way, which is what walking the other way wants.
TEST(Reversal, MirrorReversesTheMetachronalWave) {
  const auto tripod = g::strategies().at("tripod")()->phase_offsets();
  for (const auto& [leg, value] : tripod.offsets()) {
    EXPECT_FLOAT_EQ(tripod.reversed().at(leg), value) << leg;
  }

  const auto crawl = g::strategies().at("crawl")()->phase_offsets();
  const auto reversed = crawl.reversed();
  int differing = 0;
  for (const auto& [leg, value] : crawl.offsets()) {
    // Every offset is a valid cycle start, and the wave is not the same one.
    EXPECT_GE(reversed.at(leg), 0.0f);
    EXPECT_LT(reversed.at(leg), 1.0f);
    if (std::fabs(reversed.at(leg) - value) > 1.0e-6f) ++differing;
  }
  EXPECT_GE(differing, 4) << "crawl's wave did not turn around";
}

// A steady command holds its AEP still, so the bound has nothing to bound and
// the ride does what it was built for: the foot is parked over its landing spot,
// world-frame stationary, for the whole approach.
TEST(Reversal, SteadyWalkLandsWithoutSlip) {
  for (int axis = 0; axis < 2; ++axis) {
    const SkidStats steady = drive_skid("tripod", 0.0f, Turn::kHold, axis);
    ASSERT_GE(steady.landings, 6);
    EXPECT_LT(steady.peak_landing_slip, 1.0e-4f)
        << steady.worst_leg << " slips on a steady walk";
  }
}
