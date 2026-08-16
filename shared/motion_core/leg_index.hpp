// Canonical leg enumeration. The enum order is the ordering every per-leg loop,
// the config generator and the servo/pin mapping agree on, so a Leg value is a
// stable index into any of them.
#pragma once

#include <array>
#include <cstddef>
#include <string_view>

namespace hexa {

enum class Leg {
  L_FRONT,
  L_MIDDLE,
  L_REAR,
  R_FRONT,
  R_MIDDLE,
  R_REAR,
};

inline constexpr int kNumLegs = 6;

// Legs pair up left/right into three groups; the standing pose is configured per
// group (tuning.yaml default_standing_pose).
enum class LegGroup {
  FRONT,
  MIDDLE,
  REAR,
};

inline constexpr int kNumLegGroups = 3;

inline constexpr std::array<std::string_view, kNumLegs> LEG_NAMES = {
    "l_front", "l_middle", "l_rear", "r_front", "r_middle", "r_rear",
};

constexpr int leg_index(Leg leg) { return static_cast<int>(leg); }

constexpr Leg leg_from_index(int i) { return static_cast<Leg>(i); }

constexpr std::string_view leg_name(Leg leg) {
  return LEG_NAMES[static_cast<std::size_t>(leg)];
}

constexpr LegGroup leg_group(Leg leg) {
  switch (leg) {
    case Leg::L_FRONT:
    case Leg::R_FRONT:
      return LegGroup::FRONT;
    case Leg::L_MIDDLE:
    case Leg::R_MIDDLE:
      return LegGroup::MIDDLE;
    default:
      return LegGroup::REAR;
  }
}

constexpr bool leg_is_right(Leg leg) {
  return leg == Leg::R_FRONT || leg == Leg::R_MIDDLE || leg == Leg::R_REAR;
}

constexpr int group_index(LegGroup group) { return static_cast<int>(group); }

// Matching the tuning.yaml sub-block keys.
inline constexpr std::array<std::string_view, kNumLegGroups> LEG_GROUP_NAMES = {
    "front", "middle", "rear",
};

constexpr std::string_view leg_group_name(LegGroup group) {
  return LEG_GROUP_NAMES[static_cast<std::size_t>(group)];
}

// Returns Leg::L_FRONT with found=false on an unknown name, to stay
// constexpr/exception-free.
constexpr Leg leg_from_name(std::string_view name, bool& found) {
  for (std::size_t i = 0; i < LEG_NAMES.size(); ++i) {
    if (LEG_NAMES[i] == name) {
      found = true;
      return static_cast<Leg>(i);
    }
  }
  found = false;
  return Leg::L_FRONT;
}

}  // namespace hexa
