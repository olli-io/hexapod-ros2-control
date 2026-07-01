// Animation stack for the posture controller. Port of
// hexa_posture/animations/*.py (base + the seven concrete animations) and the
// factory/stack builder from hexa_posture/posture_node.py.
//
// An animation is a pure function from AnimationContext to a BodyPose offset:
// no I/O, no clocks, no ROS. The posture node owns the clock and feeds `t` in
// via the context. A Stack sums the offsets from its child animations via
// pose.add (component-wise), which is only valid for small offsets.
#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "hexa_posture_cpp/pose.hpp"

namespace hexa_posture {

// Read-only inputs an animation may consult. Extend this — don't bypass it —
// when an animation needs new state.
struct AnimationContext {
  // Monotonic time in seconds. The only clock source animations should read.
  double t = 0.0;
  // True iff the latest /cmd_vel was non-zero.
  bool walking = false;
  // Reserved: gait phase once it lands on the wire. Currently unused (kept for
  // signature parity with the Python AnimationContext).
  std::optional<double> gait_phase;
  // Master gait phase in [0, 1), sniffed from /legs/targets. nullopt until the
  // first targets message. Phase-locked animations gate on this.
  std::optional<double> master_phase;
  // Active gait strategy name (e.g. "tripod"), from /gait/params. nullopt until
  // a params message. Gait-specific animations gate on this.
  std::optional<std::string> gait_name;
  // Low-pass-filtered XY centroid of the support polygon (m), body frame.
  // nullopt until the first usable /legs/targets frame.
  std::optional<std::pair<double, double>> support_centroid_xy;
  // Max foot lift above the stance polygon mean (m), clamped >= 0. nullopt
  // until /legs/targets has been observed with a usable stance polygon.
  std::optional<double> swing_lift_z;
};

// Animation strategy interface. Mirrors the Python Animation Protocol.
class Animation {
 public:
  virtual ~Animation() = default;
  virtual BodyPose operator()(const AnimationContext& ctx) const = 0;
};

using AnimationPtr = std::shared_ptr<const Animation>;

// Composition primitive: sums the offsets of its child layers, starting from
// IDENTITY. Itself an Animation so stacks compose uniformly.
class Stack : public Animation {
 public:
  Stack() = default;
  explicit Stack(std::vector<AnimationPtr> layers)
      : layers_(std::move(layers)) {}
  BodyPose operator()(const AnimationContext& ctx) const override;
  const std::vector<AnimationPtr>& layers() const { return layers_; }

 private:
  std::vector<AnimationPtr> layers_;
};

// Identity animation — always a zero pose offset.
class Still : public Animation {
 public:
  BodyPose operator()(const AnimationContext& ctx) const override;
};

// Idle breathing — a slow vertical bob when standing still; silent while
// walking.
class Breathing : public Animation {
 public:
  explicit Breathing(double amplitude = 0.005, double period = 4.0)
      : amplitude_(amplitude), period_(period) {}
  BodyPose operator()(const AnimationContext& ctx) const override;

 private:
  double amplitude_;  // m, peak-to-zero height offset
  double period_;     // s, one full breath cycle
};

// Gait-sensitive sway — translates the body in XY toward the live support-
// polygon centroid. Off when not walking or the centroid is unseen.
class GaitSway : public Animation {
 public:
  explicit GaitSway(double gain = 1.0, double strength = 0.5)
      : gain_(gain), strength_(strength) {}
  BodyPose operator()(const AnimationContext& ctx) const override;

 private:
  double gain_;
  double strength_;
};

// Gait-synced vertical bounce (tripod-only) — lifts the body in +z to match
// the swing arc. Off when not walking, signal unseen, or gait != tripod.
class GaitBounce : public Animation {
 public:
  explicit GaitBounce(double arc_height = 0.02, double step_height_ref = 0.06)
      : arc_height_(arc_height), step_height_ref_(step_height_ref) {}
  BodyPose operator()(const AnimationContext& ctx) const override;

 private:
  double arc_height_;
  double step_height_ref_;
};

// Vertical body roll (tripod-only) — heave in z + pitch, phase-locked to the
// gait clock.
class VerticalBodyRoll : public Animation {
 public:
  explicit VerticalBodyRoll(double z_amplitude = 0.02,
                            double pitch_amplitude = 0.1745,
                            double pitch_phase_offset = 0.0)
      : z_amplitude_(z_amplitude),
        pitch_amplitude_(pitch_amplitude),
        pitch_phase_offset_(pitch_phase_offset) {}
  BodyPose operator()(const AnimationContext& ctx) const override;

 private:
  double z_amplitude_;
  double pitch_amplitude_;
  double pitch_phase_offset_;
};

// Horizontal body roll (tripod-only) — sway in y + yaw, phase-locked to the
// gait clock.
class HorizontalBodyRoll : public Animation {
 public:
  explicit HorizontalBodyRoll(double y_amplitude = 0.02,
                              double yaw_amplitude = 0.1745,
                              double yaw_phase_offset = 0.0)
      : y_amplitude_(y_amplitude),
        yaw_amplitude_(yaw_amplitude),
        yaw_phase_offset_(yaw_phase_offset) {}
  BodyPose operator()(const AnimationContext& ctx) const override;

 private:
  double y_amplitude_;
  double yaw_amplitude_;
  double yaw_phase_offset_;
};

// 3D body roll (tripod-only) — VerticalBodyRoll + HorizontalBodyRoll with a
// quarter-cycle offset so (y, z) and (pitch, yaw) each trace a circle.
class BodyRoll3D : public Animation {
 public:
  explicit BodyRoll3D(double z_amplitude = 0.02, double pitch_amplitude = 0.1745,
                      double y_amplitude = 0.02, double yaw_amplitude = 0.1745,
                      double horizontal_phase_offset = 0.25,
                      double pitch_phase_offset = 0.0,
                      double yaw_phase_offset = 0.0)
      : z_amplitude_(z_amplitude),
        pitch_amplitude_(pitch_amplitude),
        y_amplitude_(y_amplitude),
        yaw_amplitude_(yaw_amplitude),
        horizontal_phase_offset_(horizontal_phase_offset),
        pitch_phase_offset_(pitch_phase_offset),
        yaw_phase_offset_(yaw_phase_offset) {}
  BodyPose operator()(const AnimationContext& ctx) const override;

 private:
  double z_amplitude_;
  double pitch_amplitude_;
  double y_amplitude_;
  double yaw_amplitude_;
  double horizontal_phase_offset_;
  double pitch_phase_offset_;
  double yaw_phase_offset_;
};

// Default animation stack when no animation mode is active.
inline const std::vector<std::string> DEFAULT_ANIMATIONS = {"still",
                                                            "breathing"};
// Animation-mode entries the teleop layer cycles through on /animation/mode.
inline const std::vector<std::string> DEFAULT_ANIMATION_MODE_ANIMATIONS = {
    "vertical_body_roll", "horizontal_body_roll", "body_roll_3d"};

// Build a Stack from a list of animation names. Uses the override instance when
// a name is present in `overrides`, else default-constructs the named animation.
// Throws std::invalid_argument on an unknown name. Mirrors
// posture_node._build_animation_stack.
Stack build_animation_stack(
    const std::vector<std::string>& names,
    const std::map<std::string, AnimationPtr>& overrides = {});

}  // namespace hexa_posture
