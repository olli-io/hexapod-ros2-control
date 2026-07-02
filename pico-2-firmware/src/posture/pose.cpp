// Body-pose small-offset algebra — float fork of hexa_posture/pose.py.
#include "posture/pose.hpp"

namespace hexa::posture {

BodyPose add(const BodyPose& a, const BodyPose& b) {
  return BodyPose{a.x + b.x,       a.y + b.y,         a.z + b.z,
                  a.roll + b.roll, a.pitch + b.pitch, a.yaw + b.yaw};
}

BodyPose scale(const BodyPose& p, float k) {
  return BodyPose{p.x * k, p.y * k, p.z * k, p.roll * k, p.pitch * k, p.yaw * k};
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
}  // namespace

BodyPose clamp(const BodyPose& pose, const PoseLimits& limits) {
  return BodyPose{
      clamp_axis(pose.x, limits.x),         clamp_axis(pose.y, limits.y),
      clamp_axis(pose.z, limits.z),         clamp_axis(pose.roll, limits.roll),
      clamp_axis(pose.pitch, limits.pitch), clamp_axis(pose.yaw, limits.yaw)};
}

}  // namespace hexa::posture
