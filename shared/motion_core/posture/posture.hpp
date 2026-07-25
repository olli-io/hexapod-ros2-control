// Posture controller — float fork of hexa_posture/posture_node.py (plan
// part 08).
//
// The ROS node's timer callback becomes a per-tick update() call driven by the
// firmware's 200 Hz loop. Its /body/pose, /cmd_vel, /gait/state, /gait/params,
// /legs/targets, and /animation/mode subscriptions collapse into direct inputs:
// the user pose and animation-mode selection arrive via setters (they change
// only on teleop events); the per-tick signals (engine leg outputs, master
// phase, walking flag, engine state, gait name) are passed straight to
// update(). There is no wire — the posture chain reads the gait engine's output
// in-process, exactly the signals the ROS node sniffed off /legs/targets.
//
// The signal-derivation and low-pass helpers are free functions (as in the
// Python module) so the host test can exercise them without a controller.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "config_generated.hpp"  // config::PostureConfig (parameterized ctor)
#include "gait/engine.hpp"  // EngineState
#include "gait/types.hpp"   // LegOutput, LEG_NAMES
#include "posture/animations.hpp"
#include "posture/pose.hpp"

namespace hexa::posture {

// Minimum stance legs needed to define a meaningful support polygon. During
// swing transitions the count can momentarily dip; the caller holds the
// previous filtered value rather than emit a noisy one.
inline constexpr int kMinStanceForCentroid = 3;

// Mean of foot_target.{x,y} over legs flagged stance. Returns nullopt when
// fewer than kMinStanceForCentroid legs are in stance (degenerate polygon).
// Mirrors _stance_centroid_xy.
std::optional<std::pair<float, float>> stance_centroid_xy(
    const std::map<std::string, gait::LegOutput>& legs);

// Max foot lift (m) above the stance polygon's mean z:
// max(swing z) - mean(stance z), clamped >= 0. Returns nullopt when the stance
// polygon is degenerate; 0.0 when no leg is in swing (observed and quiet, not
// missing). Mirrors _max_swing_lift_z.
std::optional<float> max_swing_lift_z(
    const std::map<std::string, gait::LegOutput>& legs);

// One first-order low-pass step on an XY signal, alpha = dt / (tau + dt).
// prev=nullopt seeds from raw (no startup transient); raw=nullopt holds prev
// (mid-swing-transition behaviour). Mirrors _lpf_step_xy.
std::optional<std::pair<float, float>> lpf_step_xy(
    std::optional<std::pair<float, float>> prev,
    std::optional<std::pair<float, float>> raw, float tau, float dt);

// One first-order low-pass step on a scalar signal. Same seeding / hold-on-None
// semantics as lpf_step_xy. Mirrors _lpf_step_scalar.
std::optional<float> lpf_step_scalar(std::optional<float> prev,
                                     std::optional<float> raw, float tau,
                                     float dt);

// True iff body posture is meaningful in this engine state — the legs are at
// (or transitioning around) the nominal stance footprint. Outside this set
// (FOLDED / INITIALIZE / FOLDING) the controller emits IDENTITY. Mirrors
// POSTURE_ACTIVE_STATES.
bool posture_active(gait::EngineState state);

// Move current toward target by at most rate_per_s * dt (linear, no overshoot).
// Returns current unchanged when rate_per_s <= 0 or dt <= 0. Drives the
// gait-animation activation crossfade. Mirrors _slew_toward.
float slew_toward(float current, float target, float rate_per_s, float dt);

// Owns the animation stacks, the signal filters, the persistent user pose, and
// the active animation-mode selection — the stateful half of the posture node.
// The pure animations stay stateless; the clock (t) and walking flag are fed in
// per tick.
class PostureController {
 public:
  // Builds the default stack (config::kEnabledAnimations) and one per-animation
  // stack per config::kAnimationModeAnimations entry, all with the baked
  // config::kPosture amplitudes. Filter time constants come from the same
  // config.
  PostureController();

  // Build with explicit posture tuning (hexa_locomotion's runtime-YAML path):
  // the animation amplitudes, filter taus, activation slew rate, and per-axis
  // animation reserve all come from `posture`. The enabled-animation SET stays
  // baked (kEnabledAnimations / kAnimationModeAnimations — structural, not a
  // tuning knob). The no-arg ctor delegates here with config::kPosture.
  explicit PostureController(const ::hexa::config::PostureConfig& posture);

  // Persistent user pose from map_joy (the /body/pose subscription). Survives
  // across ticks until the next teleop update.
  void set_user_pose(const BodyPose& pose) { user_pose_ = pose; }

  // Select the ANIMATION-mode stack. Empty selects the default stack. Returns
  // false (and leaves the selection unchanged) for an unknown name so the
  // caller can log it — mirrors _on_animation_mode's warn-and-ignore.
  bool set_animation_mode(std::string_view mode);

  std::string_view animation_mode() const { return animation_mode_; }

  // Advance the filters and evaluate the active stack for this tick. Slews the
  // activation toward the walking flag, evaluates the stack twice (walking
  // true/false), lerps between them, and composes the result with the user pose
  // under the layered clamp — returning the body_pose_target, or IDENTITY when
  // the engine state is outside POSTURE_ACTIVE_STATES. Mirrors _tick
  // (+ _on_leg_targets / _step_filters, which run in-line here since firmware
  // has no wire).
  //   legs         — the engine's per-leg output this tick.
  //   master_phase — engine master phase; wrapped to [0, 1) defensively.
  //   walking      — latest /cmd_vel non-zero (from map_joy output).
  //   state        — engine FSM state; gates the whole stack.
  //   gait_name    — active strategy name; phase-locked animations gate on it.
  //   dt, t        — tick period and monotonic time, seconds.
  BodyPose update(const std::map<std::string, gait::LegOutput>& legs,
                  float master_phase, bool walking, gait::EngineState state,
                  std::string_view gait_name, float dt, float t);

 private:
  Stack default_stack_;
  std::map<std::string, Stack> animation_stacks_;
  std::string animation_mode_;

  BodyPose user_pose_ = IDENTITY;
  PoseLimits limits_;

  // Gait-animation crossfade: activation slews 0<->1 toward the walking flag so
  // the gait animations ramp in/out instead of stepping. The stack is evaluated
  // twice each tick (walking true/false) and lerp'd by this. Reset to 0 while
  // the posture stack is inactive.
  float activation_ = 0.0f;
  // Master switch for the gait-active regime (tuning.yaml
  // gait_body_animations_enabled). When false the stack's walking output is
  // dropped, so no body animation runs while gait-active — the idle animations
  // and the user pose are untouched.
  bool gait_body_animations_enabled_;
  float activation_slew_rate_;
  // Per-axis animation budget for the layered clamp (compose_layered).
  PoseLimits anim_reserve_;

  // Filter state (mirrors the node's _support_centroid_xy / _swing_lift_z and
  // their latest-raw holders).
  std::optional<std::pair<float, float>> support_centroid_xy_;
  std::optional<std::pair<float, float>> latest_raw_centroid_;
  std::optional<float> swing_lift_z_;
  std::optional<float> latest_raw_swing_lift_;

  float centroid_tau_;
  float swing_lift_tau_;
};

}  // namespace hexa::posture
