// Body-pose small-offset algebra — float fork of hexa_posture/pose.py.
#include "posture/pose.hpp"

#include <algorithm>

namespace hexa::posture {

BodyPose add(const BodyPose& a, const BodyPose& b) {
  return BodyPose{a.x + b.x,       a.y + b.y,         a.z + b.z,
                  a.roll + b.roll, a.pitch + b.pitch, a.yaw + b.yaw};
}

BodyPose scale(const BodyPose& p, float k) {
  return BodyPose{p.x * k, p.y * k, p.z * k, p.roll * k, p.pitch * k, p.yaw * k};
}

BodyPose lerp(const BodyPose& a, const BodyPose& b, float t) {
  return BodyPose{a.x + t * (b.x - a.x),         a.y + t * (b.y - a.y),
                  a.z + t * (b.z - a.z),         a.roll + t * (b.roll - a.roll),
                  a.pitch + t * (b.pitch - a.pitch), a.yaw + t * (b.yaw - a.yaw)};
}

namespace {
float clamp_axis(float v, float lo_hi) {
  if (v < -lo_hi) {
    return -lo_hi;
  }
  if (v > lo_hi) {
    return lo_hi;
  }
  return v;
}

// Asymmetric form, for z.
float clamp_axis(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}
}  // namespace

BodyPose clamp(const BodyPose& pose, const PoseLimits& limits) {
  return BodyPose{clamp_axis(pose.x, limits.x),
                  clamp_axis(pose.y, limits.y),
                  clamp_axis(pose.z, limits.z_min, limits.z_max),
                  clamp_axis(pose.roll, limits.roll),
                  clamp_axis(pose.pitch, limits.pitch),
                  clamp_axis(pose.yaw, limits.yaw)};
}

BodyPose compose_layered(const BodyPose& user, const BodyPose& animated,
                         const PoseLimits& limits,
                         const PoseLimits& anim_reserve) {
  // The user's z envelope shrinks from BOTH ends by the animation reserve; the
  // floor/ceiling are clamped toward 0 so an oversized reserve collapses the
  // user's range rather than inverting it (same intent as the max(0, ...) on
  // the symmetric axes).
  const PoseLimits user_env{
      std::max(0.0f, limits.x - anim_reserve.x),
      std::max(0.0f, limits.y - anim_reserve.y),
      std::max(0.0f, limits.z_max - anim_reserve.z_max),
      std::min(0.0f, limits.z_min - anim_reserve.z_min),
      std::max(0.0f, limits.roll - anim_reserve.roll),
      std::max(0.0f, limits.pitch - anim_reserve.pitch),
      std::max(0.0f, limits.yaw - anim_reserve.yaw)};
  const BodyPose summed =
      add(clamp(user, user_env), clamp(animated, anim_reserve));
  return clamp(summed, limits);
}

}  // namespace hexa::posture
