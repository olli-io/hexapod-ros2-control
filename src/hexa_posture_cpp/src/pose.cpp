// Port of hexa_posture/pose.py — component-wise pose composition and clamping.
#include "hexa_posture_cpp/pose.hpp"

#include <algorithm>

namespace hexa_posture {

BodyPose add(const BodyPose& a, const BodyPose& b) {
  return BodyPose{
      a.x + b.x, a.y + b.y,       a.z + b.z,
      a.roll + b.roll, a.pitch + b.pitch, a.yaw + b.yaw,
  };
}

BodyPose scale(const BodyPose& p, double k) {
  return BodyPose{
      p.x * k, p.y * k, p.z * k, p.roll * k, p.pitch * k, p.yaw * k,
  };
}

BodyPose lerp(const BodyPose& a, const BodyPose& b, double t) {
  return add(a, scale(add(b, scale(a, -1.0)), t));
}

BodyPose clamp(const BodyPose& pose, const PoseLimits& limits) {
  const auto c = [](double v, double lo_hi) {
    return std::max(-lo_hi, std::min(lo_hi, v));
  };
  return BodyPose{
      c(pose.x, limits.x),       c(pose.y, limits.y),
      c(pose.z, limits.z),       c(pose.roll, limits.roll),
      c(pose.pitch, limits.pitch), c(pose.yaw, limits.yaw),
  };
}

BodyPose compose_layered(const BodyPose& user, const BodyPose& animated,
                         const PoseLimits& limits,
                         const PoseLimits& anim_reserve) {
  const PoseLimits user_env{
      std::max(0.0, limits.x - anim_reserve.x),
      std::max(0.0, limits.y - anim_reserve.y),
      std::max(0.0, limits.z - anim_reserve.z),
      std::max(0.0, limits.roll - anim_reserve.roll),
      std::max(0.0, limits.pitch - anim_reserve.pitch),
      std::max(0.0, limits.yaw - anim_reserve.yaw),
  };
  const BodyPose summed =
      add(clamp(user, user_env), clamp(animated, anim_reserve));
  return clamp(summed, limits);
}

}  // namespace hexa_posture
