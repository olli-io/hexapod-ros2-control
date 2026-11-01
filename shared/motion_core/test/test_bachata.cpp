// The bachata routine: the pure sample (feet, beats, sway) and the engine's
// DANCE state on a preset that names it.

#include <cmath>
#include <map>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "gait/bachata.hpp"
#include "gait/engine.hpp"

namespace g = hexa::gait;

namespace {

constexpr float kDt = 0.005f;
constexpr float kTol = 1e-5f;

g::BachataParams params() {
  g::BachataParams p;
  p.beat_time = 0.6f;
  p.stride = 0.05f;
  p.planted_share = 0.1f;
  p.sway_y = 0.02f;
  p.sway_z = 0.01f;
  p.sway_roll = 0.08f;
  p.swing.clearance = 0.03f;
  p.swing.apex_time = 0.35f;
  return p;
}

bool in_a(const std::string& n) {
  return n == "l_front" || n == "r_middle" || n == "l_rear";
}

float y_offset(const g::BachataFrame& f, const std::map<std::string, g::Vec3>& nominal,
               const std::string& leg) {
  return f.legs.at(leg).foot_target.y - nominal.at(leg).y;
}

// The touchdown that ends `beat`.
float touchdown(int beat, const g::BachataParams& p) {
  return (static_cast<float>(beat) + 1.0f - p.planted_share) * p.beat_time;
}

}  // namespace

TEST(Bachata, EveryHalfBarStartsAndEndsOnNominal) {
  const auto p = params();
  const auto nominal = g::nominal_stance_from_config();
  for (int half = 0; half <= 4; ++half) {
    const float t = static_cast<float>(half * g::kBachataBeatsPerHalf) * p.beat_time;
    const auto f = g::bachata_sample(t, std::nullopt, p, nominal);
    for (const auto& [leg, home] : nominal) {
      EXPECT_NEAR((f.legs.at(leg).foot_target - home).norm(), 0.0f, kTol)
          << leg << " half " << half;
      EXPECT_TRUE(f.legs.at(leg).stance) << leg;
    }
  }
}

TEST(Bachata, OneTripodInTheAirAndSixDownAtEachHandover) {
  const auto p = params();
  const auto nominal = g::nominal_stance_from_config();
  for (float t = 0.0f; t < 16.0f * p.beat_time; t += kDt) {
    const auto f = g::bachata_sample(t, std::nullopt, p, nominal);
    int up_a = 0, up_b = 0;
    for (const auto& [leg, out] : f.legs) {
      if (!out.stance) (in_a(leg) ? up_a : up_b)++;
    }
    EXPECT_TRUE((up_a == 0 || up_a == 3) && (up_b == 0 || up_b == 3)) << t;
    EXPECT_FALSE(up_a > 0 && up_b > 0) << t;
    const float u = std::fmod(t / p.beat_time, 1.0f);
    if (u < p.planted_share - 1e-3f || u > 1.0f - p.planted_share + 1e-3f) {
      EXPECT_EQ(up_a + up_b, 0) << t;
    }
  }
}

TEST(Bachata, StepsTravelLeftThenTapsThenMirrorsRight) {
  const auto p = params();
  const auto nominal = g::nominal_stance_from_config();
  const float h = 0.5f * p.stride;
  // Mid-swing: who is up, and where each half bar's tripods end their beats.
  const auto mid = [&](int beat) {
    return g::bachata_sample((static_cast<float>(beat) + 0.5f) * p.beat_time,
                             std::nullopt, p, nominal);
  };
  const auto end = [&](int beat) {
    return g::bachata_sample((static_cast<float>(beat) + 1.0f) * p.beat_time - 1e-4f,
                             std::nullopt, p, nominal);
  };
  // Swinging tripod per beat, led by the travel-side middle leg:
  // B A B A(tap) | A B A B(tap).
  const bool a_swings[8] = {false, true, false, true, true, false, true, false};
  for (int beat = 0; beat < 8; ++beat) {
    EXPECT_FALSE(mid(beat).legs.at(a_swings[beat] ? "l_front" : "r_front").stance)
        << beat;
    EXPECT_TRUE(mid(beat).legs.at(a_swings[beat] ? "r_front" : "l_front").stance)
        << beat;
  }
  // A's lateral offset at each beat end, left half then the mirror.
  const float a_end[8] = {-h, h, 0.0f, 0.0f, -h, h, 0.0f, 0.0f};
  for (int beat = 0; beat < 8; ++beat) {
    // Left half: B leads left, so A stands first and is carried right in the
    // body frame. Right half: A leads right and ends its first step at -h.
    EXPECT_NEAR(y_offset(end(beat), nominal, "l_front"), a_end[beat], 1e-4f)
        << beat;
  }
  // The right half is the left half with A and B swapped and y mirrored.
  for (int beat = 0; beat < 4; ++beat) {
    const auto l = end(beat);
    const auto r = end(beat + 4);
    EXPECT_NEAR(y_offset(l, nominal, "l_front"), -y_offset(r, nominal, "r_front"), 1e-4f);
    EXPECT_NEAR(y_offset(l, nominal, "r_front"), -y_offset(r, nominal, "l_front"), 1e-4f);
  }
}

TEST(Bachata, TapLandsWhereItLiftedOff) {
  const auto p = params();
  const auto nominal = g::nominal_stance_from_config();
  float peak = 0.0f;
  for (float u = 0.0f; u < 1.0f; u += 0.01f) {
    const auto f = g::bachata_sample((3.0f + u) * p.beat_time, std::nullopt, p, nominal);
    for (const auto& [leg, out] : f.legs) {
      EXPECT_NEAR(out.foot_target.y, nominal.at(leg).y, 1e-4f) << leg << " " << u;
    }
    peak = std::max(peak, f.legs.at("l_front").foot_target.z - nominal.at("l_front").z);
  }
  EXPECT_NEAR(peak, 0.5f * p.swing.clearance, 1e-3f) << "the tap lifts half as high";
}

TEST(Bachata, SwayPeaksAtTouchdownTowardTheSwingingTripod) {
  const auto p = params();
  const auto nominal = g::nominal_stance_from_config();
  // +1: shift right, away from A's two left legs; tilt down right. The tap holds
  // its side.
  const float side[8] = {-1, 1, -1, -1, 1, -1, 1, 1};
  for (int beat = 0; beat < 8; ++beat) {
    const auto f = g::bachata_sample(touchdown(beat, p), std::nullopt, p, nominal);
    EXPECT_NEAR(f.body.y, -side[beat] * p.sway_y, 1e-5f) << beat;
    EXPECT_NEAR(f.body.roll, side[beat] * p.sway_roll, 1e-5f) << beat;
    EXPECT_NEAR(f.body.z, p.sway_z, 1e-5f) << beat;
    // Not exceeded anywhere in the segment that ends here.
    const float from = beat == 0 ? 0.0f : touchdown(beat - 1, p);
    for (float t = from; t < touchdown(beat, p); t += kDt) {
      const auto s = g::bachata_sample(t, std::nullopt, p, nominal);
      EXPECT_LE(std::fabs(s.body.y), p.sway_y + 1e-6f) << t;
      EXPECT_LE(s.body.z, p.sway_z + 1e-6f) << t;
      EXPECT_GE(s.body.z, 0.0f) << t;
    }
  }
  // The tap is a half sway: side, centre, same side.
  for (int beat : {3, 7}) {
    const float t = 0.5f * (touchdown(beat - 1, p) + touchdown(beat, p));
    const auto f = g::bachata_sample(t, std::nullopt, p, nominal);
    EXPECT_NEAR(f.body.y, 0.0f, 1e-5f) << beat;
    EXPECT_NEAR(f.body.z, 0.0f, 1e-5f) << beat;
  }
}

TEST(Bachata, StopFinishesTheHalfBarThenEasesOut) {
  const auto p = params();
  const auto nominal = g::nominal_stance_from_config();
  const int halves = 2;
  const float end = g::bachata_duration(halves, p);
  const float bar_end = static_cast<float>(halves * g::kBachataBeatsPerHalf) * p.beat_time;
  for (float t = bar_end; t < end + 0.5f; t += kDt) {
    const auto f = g::bachata_sample(t, halves, p, nominal);
    for (const auto& [leg, out] : f.legs) {
      EXPECT_NEAR((out.foot_target - nominal.at(leg)).norm(), 0.0f, kTol) << leg;
      EXPECT_TRUE(out.stance);
    }
  }
  EXPECT_NEAR(g::bachata_sample(touchdown(7, p), halves, p, nominal).body.y, -p.sway_y, 1e-5f)
      << "the right half's tap holds its shift right";
  EXPECT_NEAR(g::bachata_sample(end, halves, p, nominal).body.y, 0.0f, 1e-6f);
  EXPECT_NEAR(g::bachata_sample(end, halves, p, nominal).body.z, 0.0f, 1e-6f);
}

TEST(Engine, BachataPresetDancesAndHandsBackAStand) {
  auto e = g::make_default_engine("tripod");
  ASSERT_TRUE(e->start_initialize());
  for (int i = 0; i < 2000 && e->state() != g::EngineState::STAND; ++i) {
    e->update(kDt, {0.0f, 0.0f}, 0.0f);
  }
  ASSERT_EQ(e->state(), g::EngineState::STAND);
  ASSERT_TRUE(e->request_preset("bachata"));
  for (int i = 0; i < 4000; ++i) {
    e->update(kDt, {0.0f, 0.0f}, 0.0f);
    if (e->preset_id() == "bachata" && e->state() == g::EngineState::STAND) break;
  }
  ASSERT_EQ(e->preset_id(), "bachata");
  ASSERT_EQ(e->state(), g::EngineState::STAND);
  EXPECT_FALSE(e->dance().has_value());

  e->update(kDt, {0.0f, 0.05f}, 0.0f);
  ASSERT_EQ(e->state(), g::EngineState::DANCE);
  EXPECT_EQ(g::state_value(g::EngineState::DANCE), "dance");
  float max_y = 0.0f;
  for (int i = 0; i < 400; ++i) {
    e->update(kDt, {0.0f, 0.05f}, 0.0f);
    ASSERT_EQ(e->state(), g::EngineState::DANCE);
    max_y = std::max(max_y, std::fabs(e->dance()->y));
  }
  EXPECT_GT(max_y, 0.0f);

  std::map<std::string, g::LegOutput> out;
  int ticks = 0;
  for (; ticks < 2000 && e->state() == g::EngineState::DANCE; ++ticks) {
    out = e->update(kDt, {0.0f, 0.0f}, 0.0f);
  }
  ASSERT_EQ(e->state(), g::EngineState::STAND);
  EXPECT_FALSE(e->dance().has_value());
  for (const auto& leg : g::LEG_NAMES) {
    EXPECT_TRUE(out.at(leg).stance) << leg;
  }
}
