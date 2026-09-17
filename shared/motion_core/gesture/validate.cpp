#include "gesture/validate.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

#include "kinematics/leg_ik.hpp"

namespace hexa::gesture {

namespace {

void check_body_keyframe(const std::string& id, const BodyKeyframe& k,
                         const posture::PoseLimits& limits) {
  const auto fail = [&](const char* axis) {
    throw std::invalid_argument("gesture '" + id + "' body keyframe at t=" +
                                std::to_string(k.t) + " s: " + axis +
                                " is outside the posture pose limits");
  };
  if (k.hold || k.home) {
    return;
  }
  if (std::fabs(k.x) > limits.x) fail("x");
  if (std::fabs(k.y) > limits.y) fail("y");
  if (k.z > limits.z_max || k.z < limits.z_min) fail("z");
  if (std::fabs(k.roll) > limits.roll) fail("roll");
  if (std::fabs(k.pitch) > limits.pitch) fail("pitch");
  if (std::fabs(k.yaw) > limits.yaw) fail("yaw");
}

void check_leg_keyframe(const std::string& id, const std::string& leg,
                        const LegKeyframe& k, const gait::kin::LegSpec& spec,
                        float ground_z) {
  if (k.hold || k.home) {
    return;
  }
  static constexpr std::array<const char*, 3> kJointNames = {"coxa", "femur",
                                                              "tibia"};
  const JointAngles a = {k.coxa, k.femur, k.tibia};
  for (std::size_t j = 0; j < 3; ++j) {
    const auto& lim = ::hexa::config::kJointLimits[j];
    if (a[j] < lim.lower || a[j] > lim.upper) {
      throw std::invalid_argument(
          "gesture '" + id + "': " + leg + " " + kJointNames[j] + " at t=" +
          std::to_string(k.t) + " s is " + std::to_string(a[j]) +
          " rad, outside the joint limit window [" + std::to_string(lim.lower) +
          ", " + std::to_string(lim.upper) + "] rad");
    }
  }
  const float height = gait::kin::forward_kinematics(a, spec).z - ground_z;
  if (height < -kPlantedHeight) {
    throw std::invalid_argument("gesture '" + id + "': " + leg + " at t=" +
                                std::to_string(k.t) + " s is " +
                                std::to_string(-height) +
                                " m below the ground plane");
  }
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
    if (!spec.body.empty() && !spec.body.back().home) {
      throw std::invalid_argument("gesture '" + spec.id +
                                  "': the body track must end with a home "
                                  "keyframe");
    }
    for (const LegTrack& track : spec.legs) {
      const std::string leg(leg_name(track.leg));
      if (track.keys.empty() || !track.keys.back().home) {
        throw std::invalid_argument("gesture '" + spec.id + "': " + leg +
                                    " must end with a home keyframe");
      }
      const gait::kin::LegSpec& ls = leg_specs.at(leg);
      const float ground_z = body_to_leg(nominal_stance.at(leg), ls).z;
      for (const LegKeyframe& k : track.keys) {
        check_leg_keyframe(spec.id, leg, k, ls, ground_z);
      }
    }
  }
}

}  // namespace hexa::gesture
