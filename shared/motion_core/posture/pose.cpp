// Body-pose small-offset algebra — float fork of hexa_posture/pose.py.
#include "posture/pose.hpp"

#include <cmath>

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

// Both a length (m) and an angle (rad) floor. One value serves: 1e-6 of either
// is far below what a servo can express, and far above the float noise at the
// centimetre / third-of-a-radian magnitudes the envelope allows.
constexpr float kPolarEps = 1e-6f;
constexpr float kTwoPi = 6.283185307179586f;

// Scale a pair onto the disc of radius r_max, direction preserved.
void clamp_disc(float& a, float& b, float r_max) {
  const float r = std::hypot(a, b);
  if (r > r_max) {
    const float k = r_max / r;
    a *= k;
    b *= k;
  }
}

// The short-way-around representative of an angle error, in [-pi, pi].
// std::remainder is the exact form (a - 2pi*round(a/2pi)); std::fmod is not —
// it keeps the sign of the dividend.
float wrap_pi(float a) { return std::remainder(a, kTwoPi); }
}  // namespace

BodyPose clamp(const BodyPose& pose, const PoseLimits& limits) {
  BodyPose out{pose.x,
               pose.y,
               clamp_axis(pose.z, limits.z_min, limits.z_max),
               pose.roll,
               pose.pitch,
               clamp_axis(pose.yaw, limits.yaw)};
  clamp_disc(out.x, out.y, limits.xy_radius());
  clamp_disc(out.roll, out.pitch, limits.tilt_radius());
  return out;
}

PolarState to_polar(float a, float b) {
  PolarState s;
  s.radius = std::hypot(a, b);
  s.angle = s.radius > kPolarEps ? std::atan2(b, a) : 0.0f;
  return s;
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

// One step of the same spring on a pair carried in polar, so a direction sweep
// holds its reach instead of cutting the chord. Magnitude and direction get
// their own w; both integrate exactly as step_axis does, with the saturation
// clamp folded in the same way.
void step_polar(PolarState& s, float& out_a, float& out_b, float target_a,
                float target_b, float r_max, float w_radius, float w_angle,
                float zeta, float dt) {
  if (w_radius <= 0.0f) {
    // Bypass the whole pair. Snap in Cartesian rather than round-tripping
    // through polar, so the result is bit-identical to the old per-axis snap.
    out_a = target_a;
    out_b = target_b;
    clamp_disc(out_a, out_b, r_max);
    s = to_polar(out_a, out_b);
    return;
  }

  float target_r = std::hypot(target_a, target_b);
  if (target_r > r_max) {
    target_r = r_max;
  }

  // A pair sitting at the origin has no direction of its own, so adopt the
  // target's outright instead of easing off a stale one — which would fling
  // the body out along the old heading first and only then curve. Free: at
  // radius 0 there is nothing to see move.
  if (s.radius <= kPolarEps && target_r > kPolarEps) {
    s.angle = std::atan2(target_b, target_a);
    s.angle_rate = 0.0f;
  }
  // A target at the origin has no direction either. Freeze the heading and let
  // the magnitude ease in, so a withdrawn pose retracts along its own line
  // rather than spinning on the way to centre.
  const float target_angle =
      target_r > kPolarEps ? std::atan2(target_b, target_a) : s.angle;

  if (w_angle <= 0.0f) {
    s.angle = target_angle;
    s.angle_rate = 0.0f;
  } else {
    const float err = wrap_pi(target_angle - s.angle);
    s.angle_rate +=
        (w_angle * w_angle * err - 2.0f * zeta * w_angle * s.angle_rate) * dt;
    // Wrapping the state and not just the error keeps `angle` bounded however
    // many turns it accumulates.
    s.angle = wrap_pi(s.angle + s.angle_rate * dt);
  }

  s.radius_rate += (w_radius * w_radius * (target_r - s.radius) -
                    2.0f * zeta * w_radius * s.radius_rate) *
                   dt;
  s.radius += s.radius_rate * dt;
  if (s.radius < 0.0f) {
    // The floor is what stops an undershoot swinging the pair through the
    // origin and out the far side. The heading rate goes with it: a spin
    // banked while the pair sat at the origin would otherwise be released the
    // instant the magnitude came off zero.
    s.radius = 0.0f;
    s.radius_rate = 0.0f;
    s.angle_rate = 0.0f;
  } else if (s.radius > r_max) {
    s.radius = r_max;
    s.radius_rate = 0.0f;
  }

  out_a = s.radius * std::cos(s.angle);
  out_b = s.radius * std::sin(s.angle);
}
}  // namespace

BodyPose PoseSmoother::step(const BodyPose& target, const PoseLimits& envelope,
                            float dt) {
  if (dt <= 0.0f) {
    return pose_;
  }
  const float wt = omega_for(cfg_.tau_translation, dt);
  const float wr = omega_for(cfg_.tau_rotation, dt);
  const float wxa = omega_for(cfg_.tau_xy_angle, dt);
  const float wta = omega_for(cfg_.tau_tilt_angle, dt);
  const float z = cfg_.damping_ratio;

  step_polar(xy_, pose_.x, pose_.y, target.x, target.y, envelope.xy_radius(),
             wt, wxa, z, dt);
  step_polar(tilt_, pose_.roll, pose_.pitch, target.roll, target.pitch,
             envelope.tilt_radius(), wr, wta, z, dt);
  step_axis(pose_.z, vel_z_, target.z, envelope.z_min, envelope.z_max, wt, z,
            dt);
  step_axis(pose_.yaw, vel_yaw_, target.yaw, -envelope.yaw, envelope.yaw, wr, z,
            dt);
  return pose_;
}

}  // namespace hexa::posture
