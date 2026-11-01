#include "gait/bachata.hpp"

#include <algorithm>
#include <cmath>

namespace hexa::gait {

namespace {

constexpr float kPi = 3.14159265358979f;

// A tap lifts the feet this share of a step's swing height.
constexpr float kTapHeightShare = 0.5f;

// Tripod A: two left legs and the right middle, so it leads the right half.
bool in_tripod_a(const std::string& name) {
  return name == "l_front" || name == "r_middle" || name == "l_rear";
}

// One beat of a half bar, as lateral foot offsets in half strides along the
// travel. The swinging tripod is the lead one, then the other, then the lead,
// then the other taps.
struct BeatShape {
  bool lead_swings;
  float swing_from, swing_to;
  float stance_from, stance_to;
};

constexpr BeatShape kHalfBar[kBachataBeatsPerHalf] = {
    {true, 0.0f, 1.0f, 0.0f, -1.0f},
    {false, -1.0f, 1.0f, 1.0f, -1.0f},
    {true, -1.0f, 0.0f, 1.0f, 0.0f},
    {false, 0.0f, 0.0f, 0.0f, 0.0f},
};

bool is_outro(int beat, std::optional<int> halves) {
  return halves.has_value() && beat >= kBachataBeatsPerHalf * *halves;
}

bool lead_is_a(int beat) {
  return (beat / kBachataBeatsPerHalf) % 2 == 1;
}

// The sway target at the touchdown that ends `beat`: +1 shifts the body right,
// away from tripod A's two left legs, and tilts it down on the right. A tap
// holds the side of the step before.
float sway_target(int beat, std::optional<int> halves) {
  if (beat < 0 || is_outro(beat, halves)) {
    return 0.0f;
  }
  const BeatShape& shape = kHalfBar[beat % kBachataBeatsPerHalf];
  const bool tap = beat % kBachataBeatsPerHalf == kBachataBeatsPerHalf - 1;
  const bool a_swings = (tap || shape.lead_swings) == lead_is_a(beat);
  return a_swings ? 1.0f : -1.0f;
}

float sway(float t, std::optional<int> halves, const BachataParams& p) {
  const float a = p.planted_share;
  const int beat = static_cast<int>(std::floor(t / p.beat_time + a));
  if (is_outro(beat - 1, halves)) {
    return 0.0f;
  }
  const float start = beat == 0 ? 0.0f : (static_cast<float>(beat) - a) * p.beat_time;
  const float end = (static_cast<float>(beat) + 1.0f - a) * p.beat_time;
  const float x = std::clamp((t - start) / (end - start), 0.0f, 1.0f);
  const float target = sway_target(beat, halves);
  const bool tap = !is_outro(beat, halves) &&
                   beat % kBachataBeatsPerHalf == kBachataBeatsPerHalf - 1;
  if (tap) {
    return target * 0.5f * (1.0f + std::cos(2.0f * kPi * x));
  }
  const float from = sway_target(beat - 1, halves);
  return from + (target - from) * 0.5f * (1.0f - std::cos(kPi * x));
}

}  // namespace

float bachata_duration(int halves, const BachataParams& params) {
  return static_cast<float>(kBachataBeatsPerHalf * halves + 1) *
         params.beat_time;
}

BachataFrame bachata_sample(float t, std::optional<int> halves,
                            const BachataParams& params,
                            const std::map<std::string, Vec3>& nominal) {
  BachataFrame out;
  const float s = sway(t, halves, params);
  out.body.y = -s * params.sway_y;
  out.body.z = s * s * params.sway_z;
  out.body.roll = s * params.sway_roll;

  const int beat = static_cast<int>(std::floor(t / params.beat_time));
  if (t < 0.0f || is_outro(beat, halves)) {
    for (const auto& [name, home] : nominal) {
      out.legs[name] = LegOutput{home, 0.0f, true};
    }
    return out;
  }

  const BeatShape& shape = kHalfBar[beat % kBachataBeatsPerHalf];
  const float travel = lead_is_a(beat) ? -1.0f : 1.0f;
  const float half_stride = 0.5f * params.stride * travel;
  const float a = params.planted_share;
  const float swing_share = 1.0f - 2.0f * a;
  const float u = t / params.beat_time - static_cast<float>(beat);
  const float x = std::clamp((u - a) / swing_share, 0.0f, 1.0f);
  const bool airborne = u > a && u < 1.0f - a;
  SwingProfile swing = params.swing;
  if (beat % kBachataBeatsPerHalf == kBachataBeatsPerHalf - 1) {
    swing.clearance *= kTapHeightShare;
    swing.lift_height *= kTapHeightShare;
    swing.land_height *= kTapHeightShare;
  }

  for (const auto& [name, home] : nominal) {
    const bool swings = in_tripod_a(name) == (shape.lead_swings == lead_is_a(beat));
    const auto at = [&](float offset) {
      return home + Vec3(0.0f, offset * half_stride, 0.0f);
    };
    if (!swings) {
      const float o = shape.stance_from +
                      (shape.stance_to - shape.stance_from) * ease5(x);
      out.legs[name] = LegOutput{at(o), 0.0f, true};
      continue;
    }
    if (!airborne) {
      const Vec3 p = at(u <= a ? shape.swing_from : shape.swing_to);
      out.legs[name] = LegOutput{p, 0.0f, true};
      continue;
    }
    const Vec3 p = swing_arc(x, at(shape.swing_from), at(shape.swing_to),
                             identity_y_sign(home),
                             swing_share * params.beat_time, swing,
                             Vec3::Zero(), Vec3::Zero());
    out.legs[name] = LegOutput{p, x, false};
  }
  return out;
}

}  // namespace hexa::gait
