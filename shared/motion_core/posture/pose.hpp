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

  // x-y and roll-pitch are eased as PAIRS (see PoseSmoother), so their bound
  // is one radius rather than two half-widths: the disc inscribed in the box.
  // The corners are given up deliberately — a reach that is the same in every
  // direction is worth more than the extra 41% the diagonals alone would have.
  float xy_radius() const { return x < y ? x : y; }
  float tilt_radius() const { return roll < pitch ? roll : pitch; }
};

// Clamp z and yaw to their axis limits, and each eased pair onto its disc
// (direction preserved, magnitude scaled down). Mirrors pose.clamp.
BodyPose clamp(const BodyPose& pose, const PoseLimits& limits);

// Persistent polar state for one eased Cartesian pair: (a, b) carried as
// radius = hypot(a, b) and angle = atan2(b, a).
//
// The smoother integrates THIS and derives the Cartesian pair from it, rather
// than converting a Cartesian velocity into polar rates each tick — that map
// (r' = (a*a' + b*b')/r, angle' = (a*b' - b*a')/r^2) is singular at the origin,
// and carrying the angle across a pass through the origin is exactly what keeps
// the body from spinning when the pair is withdrawn.
struct PolarState {
  float radius = 0.0f;       // m for (x, y), rad for (roll, pitch)
  float angle = 0.0f;        // rad, wrapped to [-pi, pi]
  float radius_rate = 0.0f;  // per s
  float angle_rate = 0.0f;   // rad/s
};

// Seed a PolarState from a Cartesian pair, rates zeroed. At the origin the
// direction is undefined and comes back 0; the first target that has one snaps
// it, which costs no visible motion at radius 0.
PolarState to_polar(float a, float b);

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
  float tau_translation = 0.135f;  // s, magnitude of the x-y pair, and z
  float tau_rotation = 0.135f;     // s, magnitude of the roll-pitch pair, and yaw
  float tau_xy_angle = 0.135f;     // s, direction of the x-y pair
  float tau_tilt_angle = 0.135f;   // s, direction of the roll-pitch pair
  float damping_ratio = 0.32f;     // zeta, shared; < 1 overshoots
};

// Damped second-order smoother on the commanded body pose — a spring/inertia
// filter, not a keyframe easing curve, so a stepped pose command accelerates,
// coasts, and settles instead of snapping to the servos in one tick.
//
// Integrated with semi-implicit (symplectic) Euler:
//   v += (w*w*(target - pos) - 2*zeta*w*v) * dt;  pos += v * dt
//
// z and yaw are lone axes and run that directly — six mul-adds, no
// transcendentals. x-y and roll-pitch instead ease as PAIRS, in polar: the
// magnitude and the direction each get their own spring and the Cartesian pair
// is derived from the result. Per-axis easing is isotropic, so its response to
// any step is a straight line; a command that swings the body sideways
// therefore cut the chord, collapsing the magnitude to 0.707 of the reach at
// the crossing and arriving as a sequence of angled segments. In polar a
// direction sweep holds its reach and the body arcs.
//
// That costs an atan2f + hypotf + sinf/cosf per pair per tick — order 10 us on
// the Pico, ~0.2% of the 5 ms tick, but no longer the nothing the scalar
// version was.
//
// The (roll, pitch) pair is treated as a plain 2-vector with roll on the cosine
// axis. Since roll is about +x and pitch about +y, that vector is the lean
// direction turned 90 degrees — irrelevant to the easing (any consistent
// pairing sweeps the same arc), noted so nobody "fixes" it.
class PoseSmoother {
 public:
  PoseSmoother() = default;
  explicit PoseSmoother(const PoseSmootherConfig& cfg) : cfg_(cfg) {}

  // Advance one tick toward `target` and return the smoothed pose.
  //
  // `envelope` is applied INSIDE the integrator: a saturated axis — or a pair
  // pressed against its disc — is pinned and its velocity zeroed, so a target
  // parked outside the limits cannot build up momentum that has to unwind
  // before the pose responds again. Pass the target already clamped to the same
  // envelope; integrating toward an unreachable value is the same wind-up from
  // the other side.
  //
  // A pair's magnitude is also floored at zero, which is what keeps an
  // undershoot from swinging it through the origin and out the far side. Note
  // the consequence: withdrawing an x-y or tilt command no longer overshoots
  // past centre the way the old per-axis filter did at zeta < 1.
  //
  // tau <= 0 on a magnitude bypasses that whole group; tau <= 0 on a pair's
  // angle snaps the direction and eases only the magnitude.
  BodyPose step(const BodyPose& target, const PoseLimits& envelope, float dt);

  // Snap to `pose` and drop the stored velocity. The caller must do this
  // whenever the pose stops being applied (posture inactive), or the body ramps
  // from a stale offset when it re-activates.
  void reset(const BodyPose& pose = IDENTITY) {
    pose_ = pose;
    xy_ = to_polar(pose.x, pose.y);
    tilt_ = to_polar(pose.roll, pose.pitch);
    vel_z_ = 0.0f;
    vel_yaw_ = 0.0f;
  }

  const BodyPose& value() const { return pose_; }

 private:
  PoseSmootherConfig cfg_{};
  // The emitted pose. x/y/roll/pitch are DERIVED from the two polar states
  // every tick; only z and yaw are integrated in place here.
  BodyPose pose_ = IDENTITY;
  PolarState xy_{};
  PolarState tilt_{};
  float vel_z_ = 0.0f;    // m/s
  float vel_yaw_ = 0.0f;  // rad/s
};

}  // namespace hexa::posture
