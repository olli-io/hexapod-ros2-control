// Construction-time check of a gesture table against the joint limits and the
// stance it will run on. Per knot only: each joint interpolates inside the
// range its neighbouring knots span, and a tracked leg is commanded in joint
// space with no body pose on top, so the knots bound the whole path.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "gait/kinematics.hpp"
#include "gesture/player.hpp"
#include "posture/pose.hpp"
#include "vec3.hpp"

namespace hexa::gesture {

// Throws std::invalid_argument naming the gesture, leg, joint and time on a
// leg knot outside kJointLimits, naming the gesture, leg and time on a knot
// whose foot sits below the default preset's ground plane (`nominal_stance`,
// body frame), and on a body keyframe outside `limits` — the pose clamp would
// otherwise bend the track silently.
void validate_gestures(
    const std::vector<GestureSpec>& specs,
    const std::map<std::string, gait::kin::LegSpec>& leg_specs,
    const std::map<std::string, Vec3>& nominal_stance,
    const posture::PoseLimits& limits);

}  // namespace hexa::gesture
