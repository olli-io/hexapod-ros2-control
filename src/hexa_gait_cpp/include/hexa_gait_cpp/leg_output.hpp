// One leg's contribution to a LegTargets message — the shared currency every
// trajectory controller in this package emits. Port of pause.py's LegOutput.
#pragma once

#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {

// stance=true means the foot is on the ground this tick. During a descent/swing
// the phase value is informational (fractional progress through the curve).
struct LegOutput {
  Vec3 foot_target = Vec3::Zero();
  double phase = 0.0;
  bool stance = true;
};

}  // namespace hexa_gait
