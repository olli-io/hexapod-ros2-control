#include "gesture/validate.hpp"

#include <cmath>
#include <stdexcept>

#include "kinematics/leg_ik.hpp"

namespace hexa::gesture {

namespace {

constexpr float kSampleDt = 0.01f;

void check_body_keyframe(const std::string& id, const BodyKeyframe& k,
                         const posture::PoseLimits& limits) {
  const auto fail = [&](const char* axis) {
    throw std::invalid_argument("gesture '" + id + "' body keyframe at t=" +
                                std::to_string(k.t) + " s: " + axis +
                                " is outside the posture pose limits");
  };
  if (k.preserve) {
    return;
  }
  if (std::fabs(k.x) > limits.x) fail("x");
  if (std::fabs(k.y) > limits.y) fail("y");
  if (k.z > limits.z_max || k.z < limits.z_min) fail("z");
  if (std::fabs(k.roll) > limits.roll) fail("roll");
  if (std::fabs(k.pitch) > limits.pitch) fail("pitch");
  if (std::fabs(k.yaw) > limits.yaw) fail("yaw");
}

}  // namespace

void validate_gestures(
    const std::vector<GestureSpec>& specs,
    const std::map<std::string, gait::kin::LegSpec>& leg_specs,
    const std::map<std::string, Vec3>& nominal_stance,
    const posture::PoseLimits& limits) {
  for (const GestureSpec& spec : specs) {
    for (const BodyKeyframe& k : spec.body) {
      check_body_keyframe(spec.id, k, limits);
    }
    GesturePlayer player(spec, nominal_stance, nominal_stance, leg_specs);
    while (!player.done()) {
      const auto out = player.update(kSampleDt);
      const BodyPose body = player.body();
      for (const auto& [name, leg] : out) {
        const gait::kin::LegSpec& ls = leg_specs.at(name);
        const Vec3 in_leg =
            body_to_leg(apply_body_pose(leg.foot_target, body), ls);
        try {
          inverse_kinematics(in_leg, ls);
        } catch (const UnreachableTarget&) {
          throw std::invalid_argument(
              "gesture '" + spec.id + "': " + name + " is unreachable at t=" +
              std::to_string(player.t()) + " s");
        }
      }
    }
  }
}

}  // namespace hexa::gesture
