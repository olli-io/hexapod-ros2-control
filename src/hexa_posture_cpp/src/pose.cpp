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

}  // namespace hexa_posture
