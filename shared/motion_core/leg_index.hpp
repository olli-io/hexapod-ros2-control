// Canonical leg enumeration and name tables (plan part 04).
//
// Leg count is fixed at six (CLAUDE.md). The enum order matches LEG_NAMES in the
// forked libs (hexa_kinematics.leg_specs.LEG_NAMES / hexa_gait::LEG_NAMES):
// l_front, l_middle, l_rear, r_front, r_middle, r_rear. That fixed ordering is
// what every per-leg loop, the config generator, and the servo/pin mapping all
// agree on, so a Leg value is a stable index into any of them.
//
// Upstream the engine keys legs by std::string in std::map (allocating per
// tick); this enum + the index<->name helpers here are the seam for the later
// std::map<std::string,T> -> std::array<T,6> optimization (overview part 00),
// letting call sites migrate to O(1) indexed lookups without touching behaviour.
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

// Legs pair up left/right into three groups. The standing pose is configured per
// group (tuning.yaml default_standing_pose), so this is the index into that
// table; the left/right half of the pair is the mirroring in leg_is_right.
enum class LegGroup {
  FRONT,
  MIDDLE,
  REAR,
};

inline constexpr int kNumLegGroups = 3;

// Ordered by the Leg enum. Same strings and order as the ROS2 libs' LEG_NAMES.
inline constexpr std::array<std::string_view, kNumLegs> LEG_NAMES = {
    "l_front", "l_middle", "l_rear", "r_front", "r_middle", "r_rear",
};

// Leg -> dense index [0, 6).
constexpr int leg_index(Leg leg) { return static_cast<int>(leg); }

// index [0, 6) -> Leg (unchecked; callers stay within range).
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

// LegGroup -> dense index [0, 3), matching the group table's order.
constexpr int group_index(LegGroup group) { return static_cast<int>(group); }

// Names in LegGroup order, matching the tuning.yaml sub-block keys.
inline constexpr std::array<std::string_view, kNumLegGroups> LEG_GROUP_NAMES = {
    "front", "middle", "rear",
};

constexpr std::string_view leg_group_name(LegGroup group) {
  return LEG_GROUP_NAMES[static_cast<std::size_t>(group)];
}

// name -> Leg. Returns Leg::L_FRONT with found=false on an unknown name so the
// helper stays constexpr/exception-free (the forked config is closed-world).
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
