// Construction-time check of a gesture table against the joint limits and the
// stance it will run on. Per knot only: each joint interpolates inside the
// range its neighbouring knots span, and a tracked leg is commanded in joint
// space with no body pose on top, so the knots bound the whole path. Next to
// a live knot the other bound is the stand under the body pose, which the
// posture clamp keeps reachable.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "gait/kinematics.hpp"
#include "gesture/player.hpp"
#include "posture/pose.hpp"
#include "vec3.hpp"

namespace hexa::gesture {

// The structural rules one leg's table must meet, thrown as
// std::invalid_argument prefixed with `where`. A start and a home knot are
// live: their value is the standing leg under whatever body pose is on at
// that moment, which only the pipeline knows, and a hold after one is live
// too. So a start must come before the track's first pose, a
// continuous keyframe needs a fixed knot before it to draw a slope from, and
// the track must end on a home. The YAML loaders call this too.
void check_track_shape(const std::vector<LegKeyframe>& keys,
                       const std::string& where);

// Throws std::invalid_argument naming the gesture, leg, joint and time on a
// leg knot outside kJointLimits, naming the gesture, leg and time on a knot
// whose foot sits below the default preset's ground plane (`nominal_stance`,
// body frame), on a body keyframe outside `limits` — the pose clamp would
// otherwise bend the track silently — on a leg track that breaks
// check_track_shape, and on a body track that does not end with a home knot,
// since the engine hands back a stand when the gesture is done.
void validate_gestures(
    const std::vector<GestureSpec>& specs,
    const std::map<std::string, gait::kin::LegSpec>& leg_specs,
    const std::map<std::string, Vec3>& nominal_stance,
    const posture::PoseLimits& limits);

}  // namespace hexa::gesture
