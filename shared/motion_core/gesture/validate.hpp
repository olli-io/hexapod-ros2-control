// Construction-time check of a gesture table against the stance it will run
// on. Exhaustive because a gesture runs only on the default preset at a zero
// height offset, so the stance sampled here is the stance it plays on.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "gait/kinematics.hpp"
#include "gesture/player.hpp"
#include "posture/pose.hpp"
#include "vec3.hpp"

namespace hexa::gesture {

// Sample every gesture at 100 Hz from `nominal_stance` (the default preset's,
// body frame) and solve each tracked and planted foot through the body track,
// exactly as the pipeline composes it. Throws std::invalid_argument naming the
// gesture, leg and time on an unreachable sample, and on a body keyframe
// outside `limits` — the pose clamp would otherwise bend the track silently.
void validate_gestures(
    const std::vector<GestureSpec>& specs,
    const std::map<std::string, gait::kin::LegSpec>& leg_specs,
    const std::map<std::string, Vec3>& nominal_stance,
    const posture::PoseLimits& limits);

}  // namespace hexa::gesture
