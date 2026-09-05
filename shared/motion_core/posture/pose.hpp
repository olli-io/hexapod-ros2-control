// Body-pose value type + small-offset algebra for the posture stack.
#pragma once

#include "kinematics/body_transform.hpp"

namespace hexa::posture {

// The posture stack's pose currency IS the kinematics body-pose offset, which is
// what the compose step feeds to apply_body_pose.
using BodyPose = ::hexa::BodyPose;

inline constexpr BodyPose IDENTITY{};

// Component-wise sum. Valid only for small offsets — SE(3) composition does not
// commute and intrinsic-XYZ Euler angles do not add — but the posture stack
// stays in the small-angle regime.
BodyPose add(const BodyPose& a, const BodyPose& b);

BodyPose scale(const BodyPose& p, float k);

// a + t*(b - a). t is not clamped; the posture tick only passes [0, 1].
BodyPose lerp(const BodyPose& a, const BodyPose& b, float t);

// Per-axis clamp envelope for the final pose target: a blunt guard against
// runaway animation / teleop inputs, not the real (geometry-dependent) envelope.
//
// Every axis but z is symmetric. z is asymmetric because lift and drop are
// bounded by different things — the femur's upper travel going up, the leg
// folding under itself coming down. z_max/z_min stay OFFSETS from the nominal
// stance, derived in the PostureController ctor from the absolute
// body_height_{max,min}_m tuning values.
struct PoseLimits {
  float x = 0.05f;       // m
  float y = 0.05f;       // m
  float z_max = 0.04f;   // m, max body lift above nominal (positive)
  float z_min = -0.04f;  // m, max body drop below nominal (negative)
  float roll = 0.30f;    // rad (~17 deg)
  float pitch = 0.30f;   // rad
  float yaw = 0.50f;     // rad (~29 deg)

  // x-y and roll-pitch are eased as PAIRS (see PoseSmoother), so their bound is
  // one radius rather than two half-widths: the disc inscribed in the box. The
  // corners are given up so the reach is the same in every direction.
  float xy_radius() const { return x < y ? x : y; }
  float tilt_radius() const { return roll < pitch ? roll : pitch; }
};

// Clamp z and yaw to their axis limits, and each eased pair onto its disc
// (direction preserved, magnitude scaled down).
BodyPose clamp(const BodyPose& pose, const PoseLimits& limits);

// Persistent polar state for one eased Cartesian pair: (a, b) carried as
// radius = hypot(a, b) and angle = atan2(b, a). The smoother integrates THIS and
// derives the pair from it, rather than mapping a Cartesian velocity into polar
// rates each tick — that map is singular at the origin, and carrying the angle
// through the origin is what keeps the body from spinning on a withdrawal.
//
// `radius` is SIGNED, left negative for the tick a withdrawal eases through the
// origin ((-r, angle) being (r, angle+pi), which the Cartesian derivation handles
// unaided) and folded back into `angle` on the next step. Between steps it is
// canonical; read mid-step as a distance it wants std::fabs.
struct PolarState {
  float radius = 0.0f;       // m for (x, y), rad for (roll, pitch); signed
  float angle = 0.0f;        // rad, wrapped to [-pi, pi]
  float radius_rate = 0.0f;  // per s
  float angle_rate = 0.0f;   // rad/s
};

// Seed a PolarState from a Cartesian pair, rates zeroed. At the origin the
// direction comes back 0 and the first target that has one snaps it, which costs
// no visible motion at radius 0.
PolarState to_polar(float a, float b);

// Tuning for PoseSmoother: one spring for every axis group — both pair
// magnitudes, both pair directions, and the lone z and yaw axes.
//
// Parameterised by omega_n and zeta rather than the reference implementation's
// spring/friction constants, which are frame-rate artifacts (its ~60 Hz works
// out to 7.4 rad/s and zeta 0.32; the same literals at 200 Hz give 13.6 and
// 0.59). The defaults reproduce the reference feel at any tick rate.
//
// One tau means a direction sweep and a radial move of equal size take equal
// TIME, not equal path — the body's response to a stick is one time constant
// whatever the stick did. Splitting the direction back out means restoring the
// second omega_for() call in step(); the pair kernel already takes its own w.
struct PoseSmootherConfig {
  float tau = 0.135f;           // s, 1/omega_n; <= 0 bypasses the filter
  float damping_ratio = 0.32f;  // zeta; < 1 overshoots

  // Settle deadband. A spring's tail is asymptotic, so a released stick would
  // otherwise leave the body creeping through fractions of a servo count long
  // after it has visibly arrived. Inside this of zero — command, position, and
  // velocity scaled by tau -- the axis snaps to exactly 0 and drops its
  // velocity. Two tolerances because the pose carries both metres and radians,
  // as control's snap_tol_* do. 0 disables.
  float snap_tol_linear = 1.0e-4f;   // m, for x-y and z
  float snap_tol_angular = 1.0e-4f;  // rad, for roll-pitch and yaw
};

// Damped second-order smoother on the commanded body pose — a spring/inertia
// filter, not a keyframe easing curve — integrated with semi-implicit Euler:
//   v += (w*w*(target - pos) - 2*zeta*w*v) * dt;  pos += v * dt
//
// z and yaw run that directly. x-y and roll-pitch ease as PAIRS in polar,
// magnitude and direction each with their own spring, because per-axis easing
// cuts the chord on a direction change: it collapsed the magnitude to 0.707 of
// the reach at the crossing and arrived as angled segments. In polar the reach
// holds and the body arcs. That costs an atan2f + hypotf + sinf/cosf per pair per
// tick, ~0.2% of the Pico's 5 ms tick.
//
// (roll, pitch) is paired as a plain 2-vector with roll on the cosine axis, which
// is the lean direction turned 90 degrees — irrelevant to the easing, noted so
// nobody "fixes" it.
class PoseSmoother {
 public:
  PoseSmoother() = default;
  explicit PoseSmoother(const PoseSmootherConfig& cfg) : cfg_(cfg) {}

  // Advance one tick toward `target` and return the smoothed pose.
  //
  // `envelope` is applied INSIDE the integrator: a saturated axis, or a pair
  // pressed against its disc, is pinned and its velocity zeroed, so a target
  // outside the limits cannot build momentum that has to unwind before the pose
  // responds again. Pass the target already clamped to the same envelope.
  //
  // A pair's magnitude is NOT floored at zero: a withdrawal eases through the
  // origin and rings down about it as the lone axes do, and a floor would stop
  // the body dead at its fastest point. The rebound past centre is
  // exp(-pi*zeta/sqrt(1-zeta^2)) of the reach withdrawn, so damping_ratio is the
  // knob that governs it. The snap_tol_* deadband ends that ring-down rather
  // than bounding it: it needs the axis SLOW as well as near zero, so it fires
  // on the arrival and never on the crossing.
  //
  // tau <= 0 bypasses the filter — every axis snaps to the clamped target.
  BodyPose step(const BodyPose& target, const PoseLimits& envelope, float dt);

  // Snap to `pose` and drop the stored velocity. Callers must do this whenever
  // the pose stops being applied, or the body ramps from a stale offset when it
  // re-activates.
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
  // x/y/roll/pitch are DERIVED from the two polar states every tick; only z and
  // yaw are integrated in place here.
  BodyPose pose_ = IDENTITY;
  PolarState xy_{};
  PolarState tilt_{};
  float vel_z_ = 0.0f;    // m/s
  float vel_yaw_ = 0.0f;  // rad/s
};

}  // namespace hexa::posture
