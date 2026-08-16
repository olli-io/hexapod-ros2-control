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

// Serves as both a length (m) and an angle (rad) floor: 1e-6 of either is below
// what a servo can express and above the float noise at these magnitudes.
constexpr float kPolarEps = 1e-6f;
constexpr float kPi = 3.141592653589793f;
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

// Short way round, in [-pi, pi]. std::remainder, not std::fmod, which keeps the
// sign of the dividend.
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
// One semi-implicit Euler step of a damped spring, with the saturation clamp
// folded in so a pinned axis also drops its velocity. A non-positive w snaps.
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

// Semi-implicit Euler diverges around w*dt > 2, so a mistyped tau caps at the
// fastest well-behaved response. Inert at the shipped tau (w*dt ~ 0.037).
float omega_for(float tau, float dt) {
  if (tau <= 0.0f) {
    return 0.0f;  // bypass
  }
  const float w = 1.0f / tau;
  const float w_max = 0.5f / dt;
  return w > w_max ? w_max : w;
}

// One step of the same spring on a pair carried in polar, so a direction sweep
// holds its reach instead of cutting the chord. Magnitude and direction keep
// separate state but share one w, which is a parameter so the direction can be
// given its own tau again by passing a second one in.
void step_polar(PolarState& s, float& out_a, float& out_b, float target_a,
                float target_b, float r_max, float w, float zeta, float dt) {
  if (w <= 0.0f) {
    // Snap in Cartesian rather than round-tripping through polar, so the result
    // is bit-identical to the old per-axis snap.
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

  // The magnitude is SIGNED between ticks. Canonicalise before anything reads
  // the heading, or a pair caught mid-crossing is re-commanded against a heading
  // a half-turn from its own position and sweeps the long way round to a point
  // it already stands on. Exact: negating the rate with the radius leaves the
  // emitted point and its velocity untouched.
  if (s.radius < 0.0f) {
    s.radius = -s.radius;
    s.radius_rate = -s.radius_rate;
    s.angle = wrap_pi(s.angle + kPi);
  }

  // A pair at the origin has no direction, so adopt the target's outright rather
  // than easing off a stale one and flinging the body along the old heading
  // first. Free: at radius 0 there is nothing to see move.
  if (s.radius <= kPolarEps && target_r > kPolarEps) {
    s.angle = std::atan2(target_b, target_a);
    s.angle_rate = 0.0f;
  }
  // Nor has a target at the origin: freeze the heading so a withdrawn pose
  // retracts along its own line rather than spinning on the way to centre.
  const float target_angle =
      target_r > kPolarEps ? std::atan2(target_b, target_a) : s.angle;

  const float err = wrap_pi(target_angle - s.angle);
  s.angle_rate += (w * w * err - 2.0f * zeta * w * s.angle_rate) * dt;
  // Wrapping the state, not just the error, keeps `angle` bounded.
  s.angle = wrap_pi(s.angle + s.angle_rate * dt);

  s.radius_rate +=
      (w * w * (target_r - s.radius) - 2.0f * zeta * w * s.radius_rate) * dt;
  s.radius += s.radius_rate * dt;
  // The magnitude runs SIGNED and may cross the origin. A floor at zero would pin
  // the pair where a withdrawal moves fastest and stop the body dead; damping
  // bounds the excursion out the far side instead, at
  // exp(-pi*zeta/sqrt(1-zeta^2)) of the reach withdrawn. The derivation below
  // handles a negative magnitude unaided, and the next tick folds the sign back
  // into the heading.
  if (s.radius > r_max) {
    s.radius = r_max;
    s.radius_rate = 0.0f;
  } else if (s.radius < -r_max) {
    // The bound is on the absolute magnitude, so a rebound is held to the same
    // envelope as a reach.
    s.radius = -r_max;
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
  const float w = omega_for(cfg_.tau, dt);
  const float z = cfg_.damping_ratio;

  step_polar(xy_, pose_.x, pose_.y, target.x, target.y, envelope.xy_radius(), w,
             z, dt);
  step_polar(tilt_, pose_.roll, pose_.pitch, target.roll, target.pitch,
             envelope.tilt_radius(), w, z, dt);
  step_axis(pose_.z, vel_z_, target.z, envelope.z_min, envelope.z_max, w, z, dt);
  step_axis(pose_.yaw, vel_yaw_, target.yaw, -envelope.yaw, envelope.yaw, w, z,
            dt);
  return pose_;
}

}  // namespace hexa::posture
