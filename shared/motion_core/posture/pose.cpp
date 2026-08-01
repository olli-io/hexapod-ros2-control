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

PoseLimits user_envelope(const PoseLimits& limits,
                         const PoseLimits& anim_reserve) {
  // The user's z envelope shrinks from BOTH ends by the animation reserve; the
  // floor/ceiling are clamped toward 0 so an oversized reserve collapses the
  // user's range rather than inverting it (same intent as the max(0, ...) on
  // the symmetric axes).
  return PoseLimits{std::max(0.0f, limits.x - anim_reserve.x),
                    std::max(0.0f, limits.y - anim_reserve.y),
                    std::max(0.0f, limits.z_max - anim_reserve.z_max),
                    std::min(0.0f, limits.z_min - anim_reserve.z_min),
                    std::max(0.0f, limits.roll - anim_reserve.roll),
                    std::max(0.0f, limits.pitch - anim_reserve.pitch),
                    std::max(0.0f, limits.yaw - anim_reserve.yaw)};
}

BodyPose compose_layered(const BodyPose& user, const BodyPose& animated,
                         const PoseLimits& limits,
                         const PoseLimits& anim_reserve) {
  const PoseLimits user_env = user_envelope(limits, anim_reserve);
  const BodyPose summed =
      add(clamp(user, user_env), clamp(animated, anim_reserve));
  return clamp(summed, limits);
}

namespace {
// One semi-implicit Euler step of a damped spring on a single axis, with the
// saturation clamp folded in so a pinned axis also drops its velocity.
//
// w is pre-clamped by the caller; a non-positive w means "bypass" and snaps.
void step_axis(float& pos, float& vel, float target, float lo, float hi, float w,
               float zeta, float dt) {
  if (w <= 0.0f) {
    pos = clamp_axis(target, lo, hi);
    vel = 0.0f;
    return;
  }
  vel += (w * w * (target - pos) - 2.0f * zeta * w * vel) * dt;
  pos += vel * dt;
  if (pos < lo) {
    pos = lo;
    vel = 0.0f;
  } else if (pos > hi) {
    pos = hi;
    vel = 0.0f;
  }
}

// Semi-implicit Euler on an oscillator diverges around w*dt > 2; capping at 0.5
// keeps a mistyped tau at the fastest well-behaved response instead of shaking
// the servos apart. Inert at the shipped tau, where w*dt ~ 0.037.
float omega_for(float tau, float dt) {
  if (tau <= 0.0f) {
    return 0.0f;  // bypass
  }
  const float w = 1.0f / tau;
  const float w_max = 0.5f / dt;
  return w > w_max ? w_max : w;
}
}  // namespace

BodyPose PoseSmoother::step(const BodyPose& target, const PoseLimits& envelope,
                            float dt) {
  if (dt <= 0.0f) {
    return pose_;
  }
  const float wt = omega_for(cfg_.tau_translation, dt);
  const float wr = omega_for(cfg_.tau_rotation, dt);
  const float z = cfg_.damping_ratio;

  step_axis(pose_.x, vel_.x, target.x, -envelope.x, envelope.x, wt, z, dt);
  step_axis(pose_.y, vel_.y, target.y, -envelope.y, envelope.y, wt, z, dt);
  step_axis(pose_.z, vel_.z, target.z, envelope.z_min, envelope.z_max, wt, z,
            dt);
  step_axis(pose_.roll, vel_.roll, target.roll, -envelope.roll, envelope.roll,
            wr, z, dt);
  step_axis(pose_.pitch, vel_.pitch, target.pitch, -envelope.pitch,
            envelope.pitch, wr, z, dt);
  step_axis(pose_.yaw, vel_.yaw, target.yaw, -envelope.yaw, envelope.yaw, wr, z,
            dt);
  return pose_;
}

}  // namespace hexa::posture
