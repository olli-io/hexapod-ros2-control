// Body-pose value type + small-offset algebra for the posture stack (plan
// part 08).
//
// Float fork of hexa_posture/pose.py. The Python module carries its own
// ROS-free BodyPose analogue so the animation / clamp / composition code stays
// importable without rclpy; here the equivalent role is played by the
// kinematics BodyPose (kinematics/body_transform.hpp) — the two structs are
// field-for-field identical (x, y, z, roll, pitch, yaw floats), and the whole
// point of the posture stack is to produce the BodyPose the compose step then
// feeds to apply_body_pose. Reusing the one struct avoids a redundant twin and
// a conversion at the seam.
//
// The add()/scale()/clamp() helpers live in namespace hexa::posture to mirror
// the Python pose module surface without colliding with the kinematics free
// functions.
#pragma once

#include "kinematics/body_transform.hpp"  // hexa::BodyPose, IDENTITY_BODY_POSE

namespace hexa::posture {

// The posture stack's pose currency IS the kinematics body-pose offset.
using BodyPose = ::hexa::BodyPose;

inline constexpr BodyPose IDENTITY{};

// Component-wise sum. Valid only for small offsets: full SE(3) composition
// doesn't commute and intrinsic-XYZ Euler angles don't add, but the posture
// stack stays in the small-angle regime (centimetre / single-digit-degree
// amplitudes), so additive composition is good enough. Mirrors pose.add.
BodyPose add(const BodyPose& a, const BodyPose& b);

// Uniform per-component scale. Mirrors pose.scale.
BodyPose scale(const BodyPose& p, float k);

// Component-wise linear interpolation: a + t*(b - a). t is not clamped, but the
// posture tick only ever passes t in [0, 1] (the gait-animation activation
// crossfade). Mirrors pose.lerp.
BodyPose lerp(const BodyPose& a, const BodyPose& b, float t);

// Per-axis clamp envelope for the final pose target. A blunt safety first cut
// (the real reachable envelope is geometry-dependent); a cheap upstream guard
// against runaway animation / teleop inputs.
//
// Every axis but z is symmetric (+/- the single value). z is asymmetric because
// lift and drop are bounded by different things — the femur's upper travel on
// the way up, the leg folding under itself on the way down. z_max/z_min stay
// OFFSETS from the nominal stance (BodyPose::z is itself an offset; see
// kinematics/body_transform.cpp), derived once in the PostureController ctor
// from the absolute body_height_{max,min}_m tuning values.
struct PoseLimits {
  float x = 0.05f;       // m
  float y = 0.05f;       // m
  float z_max = 0.04f;   // m, max body lift above nominal (positive)
  float z_min = -0.04f;  // m, max body drop below nominal (negative)
  float roll = 0.30f;    // rad (~17 deg)
  float pitch = 0.30f;   // rad
  float yaw = 0.50f;     // rad (~29 deg)
};

// Clamp each axis to [-limit, +limit], and z to [z_min, z_max]. Mirrors
// pose.clamp.
BodyPose clamp(const BodyPose& pose, const PoseLimits& limits);

// The user pose's share of the envelope: (limits - anim_reserve) per axis,
// floored at 0 (ceilinged at 0 for z_min) so an oversized reserve collapses the
// user's range rather than inverting it. Both compose_layered and the pose
// smoother clamp against this, so they must agree on it.
PoseLimits user_envelope(const PoseLimits& limits, const PoseLimits& anim_reserve);

// Layered clamp: give the static user pose and the animation offset each their
// own budget so a dialed-in posture can never asymmetrically clip the animation.
// The user pose is clamped to user_envelope(limits, anim_reserve), the animation
// to +/-anim_reserve, then summed and clamped to limits as a final guard (inert
// while user_env + anim_reserve <= limits). Trade-off: the user's static range
// shrinks by anim_reserve per axis. Mirrors pose.compose_layered.
BodyPose compose_layered(const BodyPose& user, const BodyPose& animated,
                         const PoseLimits& limits,
                         const PoseLimits& anim_reserve);

// Tuning for PoseSmoother, one (tau, zeta) pair per axis group.
//
// Parameterised by omega_n and zeta rather than by the reference
// implementation's raw spring/friction constants, because those constants are
// frame-rate artifacts: it retains 0.92 of the velocity per step and advances
// position by the per-TICK displacement, which works out to omega_n =
// sqrt(0.92/dt) and zeta = 0.0417/sqrt(dt). Its ~60 Hz gives 7.4 rad/s and
// zeta 0.32; the same literals at our 200 Hz would give 13.6 and 0.59. The
// defaults below reproduce the reference feel at any tick rate.
struct PoseSmootherConfig {
  float tau_translation = 0.135f;  // s, applies to x/y/z
  float tau_rotation = 0.135f;     // s, applies to roll/pitch/yaw
  float damping_ratio = 0.32f;     // zeta, shared; < 1 overshoots
};

// Damped second-order smoother on the commanded body pose — a spring/inertia
// filter, not a keyframe easing curve, so a stepped pose command accelerates,
// coasts, and settles instead of snapping to the servos in one tick.
//
// Integrated with semi-implicit (symplectic) Euler per axis:
//   v += (w*w*(target - pos) - 2*zeta*w*v) * dt;  pos += v * dt
// Six mul-adds per axis and no transcendentals, which is what makes it viable
// inside the Pico's 200 Hz loop.
class PoseSmoother {
 public:
  PoseSmoother() = default;
  explicit PoseSmoother(const PoseSmootherConfig& cfg) : cfg_(cfg) {}

  // Advance one tick toward `target` and return the smoothed pose.
  //
  // `envelope` is applied INSIDE the integrator: a saturated axis is pinned and
  // its velocity zeroed, so a target parked outside the limits cannot build up
  // momentum that has to unwind before the pose responds again. Pass the target
  // already clamped to the same envelope — integrating toward an unreachable
  // value is the same wind-up from the other side.
  //
  // tau <= 0 bypasses the filter for that axis group.
  BodyPose step(const BodyPose& target, const PoseLimits& envelope, float dt);

  // Snap to `pose` and drop the stored velocity. The caller must do this
  // whenever the pose stops being applied (posture inactive), or the body ramps
  // from a stale offset when it re-activates.
  void reset(const BodyPose& pose = IDENTITY) {
    pose_ = pose;
    vel_ = IDENTITY;
  }

  const BodyPose& value() const { return pose_; }

 private:
  PoseSmootherConfig cfg_{};
  BodyPose pose_ = IDENTITY;
  // Per-second rates, not a pose — it borrows the struct for its six floats.
  BodyPose vel_ = IDENTITY;
};

}  // namespace hexa::posture
