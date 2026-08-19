// Posture controller, driven by the 200 Hz loop. The user pose and
// animation-mode selection arrive via setters (they change only on teleop
// events); the per-tick signals go straight to update(), which reads the gait
// engine's output in-process. The signal-derivation and low-pass helpers are
// free functions so the host test can exercise them without a controller.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "gait/types.hpp"
#include "posture/animations.hpp"
#include "posture/pose.hpp"

namespace hexa::posture {

// Minimum stance legs for a meaningful support polygon. The count dips during
// swing transitions; the caller then holds the previous filtered value.
inline constexpr int kMinStanceForCentroid = 3;

// Mean of foot_target.{x,y} over stance legs; nullopt below
// kMinStanceForCentroid.
std::optional<std::pair<float, float>> stance_centroid_xy(
    const std::map<std::string, gait::LegOutput>& legs);

// Where the body should be standing a short while from now: the stance feet
// weighted by how long each has left before its lift-off,
//
//   w_i = clamp((1 - phase_i) / lead, 0, 1),   phase 0 == lift-off
//
// so a foot about to leave stops counting and the target has already moved into
// the triangle that will remain. Every weight is non-negative and at least one
// is positive, which makes the result a convex combination of grounded feet and
// therefore always strictly inside the support polygon — the property the whole
// quadruped creep rests on. `lead` is in cycles; 0 degrades to the plain
// centroid. nullopt below kMinStanceForCentroid, like stance_centroid_xy.
std::optional<std::pair<float, float>> anticipated_support_xy(
    const std::map<std::string, gait::LegOutput>& legs, float lead);

// max(swing z) - mean(stance z), clamped >= 0. nullopt on a degenerate stance
// polygon; 0.0 with no leg in swing (observed and quiet, not missing).
std::optional<float> max_swing_lift_z(
    const std::map<std::string, gait::LegOutput>& legs);

// One first-order low-pass step, alpha = dt / (tau + dt). prev=nullopt seeds
// from raw (no startup transient); raw=nullopt holds prev.
std::optional<std::pair<float, float>> lpf_step_xy(
    std::optional<std::pair<float, float>> prev,
    std::optional<std::pair<float, float>> raw, float tau, float dt);

// Same, for a scalar signal.
std::optional<float> lpf_step_scalar(std::optional<float> prev,
                                     std::optional<float> raw, float tau,
                                     float dt);

// The same first-order step, taken in POLAR: the previous value and the raw
// target are each resolved to a radius and an angle, both lagged, and the result
// converted back to x-y. Same tau, same alpha, same hold/seed rules — only the
// path between two points differs, and that is the whole point.
//
// Lagging x and y independently cuts the chord on a direction change: the two
// axes cross their midpoints together, so the magnitude collapses toward 0.707
// of the reach at the crossing and the path comes out as angled segments — the
// corners. In polar the radius barely moves while the angle sweeps, so the
// signal arcs through the turn at full reach and there is no corner to round
// off. PoseSmoother eases the commanded body pose the same way and for the same
// reason.
//
// No extra state: the angle is recovered from `prev` each tick, so this is a
// drop-in for lpf_step_xy. At radius zero the direction is undefined and comes
// back 0 — which costs nothing, the origin having no heading to preserve.
std::optional<std::pair<float, float>> lpf_step_polar_xy(
    std::optional<std::pair<float, float>> prev,
    std::optional<std::pair<float, float>> raw, float tau, float dt);

// True where the legs are at (or transitioning around) the nominal stance
// footprint. Elsewhere the controller emits IDENTITY.
bool posture_active(gait::EngineState state);

// Move current toward target by at most rate_per_s * dt, no overshoot. Drives
// the gait-animation activation crossfade.
float slew_toward(float current, float target, float rate_per_s, float dt);

// The stateful half of the posture stack: the animation stacks, the signal
// filters, the persistent user pose and the animation-mode selection.
class PostureController {
 public:
  // The default stack plus one per config::kAnimationModeAnimations entry, all
  // on the baked config::kPosture tuning.
  PostureController();

  // Explicit posture tuning (hexa_locomotion's runtime-YAML path). The
  // enabled-animation SET stays baked — structural, not a tuning knob.
  explicit PostureController(const ::hexa::config::PostureConfig& posture);

  // Persistent until the next teleop update. The smoother's TARGET, not the pose
  // that reaches IK.
  void set_user_pose(const BodyPose& pose) { user_pose_ = pose; }

  // Empty selects the default stack. An unknown name returns false and leaves
  // the selection unchanged, so the caller can log it.
  bool set_animation_mode(std::string_view mode);

  std::string_view animation_mode() const { return animation_mode_; }

  // Advance the filters and evaluate the active stack: slew the activation
  // toward `walking`, evaluate the stack twice (walking true/false), lerp, and
  // sum with the user pose under the pose-limit clamp. IDENTITY where
  // posture_active(state) is false. master_phase is wrapped to [0, 1)
  // defensively; phase-locked animations gate on gait_name.
  BodyPose update(const std::map<std::string, gait::LegOutput>& legs,
                  float master_phase, bool walking, gait::EngineState state,
                  std::string_view gait_name, gait::LegSet leg_set, float dt,
                  float t);

 private:
  Stack default_stack_;
  // Not built from config: see the constructor.
  Stack quadruped_stack_;
  std::map<std::string, Stack> animation_stacks_;
  std::string animation_mode_;

  BodyPose user_pose_ = IDENTITY;
  PoseLimits limits_;
  PoseSmoother pose_smoother_;

  // Slews 0<->1 toward the walking flag so the gait animations ramp instead of
  // stepping. Reset to 0 while the posture stack is inactive.
  float activation_ = 0.0f;
  // Drops the DEFAULT stack's walking output, so gait_sway/gait_bounce stay off
  // while gait-active. Idle animations, the user pose and an explicitly selected
  // animation mode are untouched.
  bool gait_body_animations_enabled_;
  float activation_slew_rate_;

  std::optional<std::pair<float, float>> support_centroid_xy_;
  std::optional<std::pair<float, float>> latest_raw_centroid_;
  std::optional<std::pair<float, float>> anticipated_support_xy_;
  std::optional<std::pair<float, float>> latest_raw_anticipated_;
  std::optional<float> swing_lift_z_;
  std::optional<float> latest_raw_swing_lift_;

  float centroid_tau_;
  float swing_lift_tau_;
  float support_shift_lead_;
  float support_shift_tau_;
};

}  // namespace hexa::posture
