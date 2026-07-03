// Animation stack — float fork of hexa_posture/animations (plan part 08).
//
// An animation is a pure function from AnimationContext to a BodyPose offset.
// State that persists across calls lives in the animation instance (its config
// fields); the function must not perform I/O, read clocks, or touch the engine
// — the posture controller owns the clock and feeds `t` in via the context.
//
// A Stack sums its child animations via pose.add (component-wise), valid only
// for small offsets. The Python module uses a Protocol + frozen dataclasses;
// here each animation is a small polymorphic type deriving Animation, and a
// Stack owns shared_ptrs to its layers (built once by the posture controller
// from the baked config, then reused every tick).
#pragma once

#include <memory>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "posture/pose.hpp"

namespace hexa::posture {

// Read-only inputs an animation may consult. Mirrors AnimationContext; the
// optional fields carry the Python `None` semantics (signal not yet observed).
struct AnimationContext {
  // Monotonic time in seconds. Animations treat this as the only clock source.
  float t = 0.0f;
  // True iff the latest /cmd_vel was non-zero. Gates pose-mode-only animations
  // (idle breathing) and gait-only ones (sway, rolls).
  bool walking = false;
  // Master gait phase in [0, 1), 0 at lift-off for the reference leg. Empty
  // until the first engine output has been seen; phase-locked animations gate
  // on it.
  std::optional<float> master_phase = std::nullopt;
  // Active gait strategy name (e.g. "tripod"). Empty view means "unknown";
  // animations only safe under a specific gait gate on it.
  std::string_view gait_name = "";
  // Low-pass-filtered XY centroid of the support polygon in the body frame (m).
  // Empty until observed.
  std::optional<std::pair<float, float>> support_centroid_xy = std::nullopt;
  // Max foot lift above the stance mean (m) across swing legs, clamped >= 0.
  // Empty until observed with a usable stance polygon.
  std::optional<float> swing_lift_z = std::nullopt;
};

// Pure animation interface: (context) -> body-pose offset.
class Animation {
 public:
  virtual ~Animation() = default;
  virtual BodyPose eval(const AnimationContext& ctx) const = 0;
};

// Identity animation — always a zero offset. Mirrors Still.
class Still : public Animation {
 public:
  BodyPose eval(const AnimationContext& ctx) const override;
};

// Idle breathing — slow vertical bob when standing still, off during walking.
// Mirrors Breathing.
class Breathing : public Animation {
 public:
  explicit Breathing(float amplitude = 0.005f, float period = 4.0f)
      : amplitude_(amplitude), period_(period) {}
  BodyPose eval(const AnimationContext& ctx) const override;

 private:
  float amplitude_;  // m, peak-to-zero height offset
  float period_;     // s, one full breath cycle
};

// Gait-sensitive sway — translates the body in XY to track the support-polygon
// centroid. Off when not walking or the centroid hasn't been observed. Mirrors
// GaitSway.
class GaitSway : public Animation {
 public:
  explicit GaitSway(float gain = 1.0f, float strength = 0.5f)
      : gain_(gain), strength_(strength) {}
  BodyPose eval(const AnimationContext& ctx) const override;

 private:
  float gain_;      // feedforward gain on the centroid
  float strength_;  // user-facing attenuator in [0, 1]
};

// Gait-synced vertical bounce — lifts the body in +z to match the swing arc.
// Tripod-only. Mirrors GaitBounce.
class GaitBounce : public Animation {
 public:
  explicit GaitBounce(float arc_height = 0.02f, float step_height_ref = 0.06f)
      : arc_height_(arc_height), step_height_ref_(step_height_ref) {}
  BodyPose eval(const AnimationContext& ctx) const override;

 private:
  float arc_height_;       // max body lift (m) at swing apex
  float step_height_ref_;  // reference foot swing apex (m) for normalisation
};

// Vertical body roll — heave (z) + pitch, phase-locked, tripod-only. Mirrors
// VerticalBodyRoll.
class VerticalBodyRoll : public Animation {
 public:
  explicit VerticalBodyRoll(float z_amplitude = 0.02f,
                            float pitch_amplitude = 0.1745f,
                            float pitch_phase_offset = 0.0f)
      : z_amplitude_(z_amplitude),
        pitch_amplitude_(pitch_amplitude),
        pitch_phase_offset_(pitch_phase_offset) {}
  BodyPose eval(const AnimationContext& ctx) const override;

 private:
  float z_amplitude_;
  float pitch_amplitude_;
  float pitch_phase_offset_;
};

// Horizontal body roll — sway (y) + yaw, phase-locked, tripod-only. Mirror of
// VerticalBodyRoll in the transverse plane. Mirrors HorizontalBodyRoll.
class HorizontalBodyRoll : public Animation {
 public:
  explicit HorizontalBodyRoll(float y_amplitude = 0.02f,
                              float yaw_amplitude = 0.1745f,
                              float yaw_phase_offset = 0.0f)
      : y_amplitude_(y_amplitude),
        yaw_amplitude_(yaw_amplitude),
        yaw_phase_offset_(yaw_phase_offset) {}
  BodyPose eval(const AnimationContext& ctx) const override;

 private:
  float y_amplitude_;
  float yaw_amplitude_;
  float yaw_phase_offset_;
};

// 3D body roll — vertical + horizontal rolls with a quarter-cycle offset so the
// (y, z) translation and (pitch, yaw) rotation trace circles. Tripod-only.
// Mirrors BodyRoll3D.
class BodyRoll3D : public Animation {
 public:
  explicit BodyRoll3D(float z_amplitude = 0.02f, float pitch_amplitude = 0.1745f,
                      float y_amplitude = 0.02f, float yaw_amplitude = 0.1745f,
                      float horizontal_phase_offset = 0.25f,
                      float pitch_phase_offset = 0.0f,
                      float yaw_phase_offset = 0.0f)
      : z_amplitude_(z_amplitude),
        pitch_amplitude_(pitch_amplitude),
        y_amplitude_(y_amplitude),
        yaw_amplitude_(yaw_amplitude),
        horizontal_phase_offset_(horizontal_phase_offset),
        pitch_phase_offset_(pitch_phase_offset),
        yaw_phase_offset_(yaw_phase_offset) {}
  BodyPose eval(const AnimationContext& ctx) const override;

 private:
  float z_amplitude_;
  float pitch_amplitude_;
  float y_amplitude_;
  float yaw_amplitude_;
  float horizontal_phase_offset_;
  float pitch_phase_offset_;
  float yaw_phase_offset_;
};

// Composition primitive — sums its layers' offsets via pose.add. Mirrors Stack.
class Stack : public Animation {
 public:
  Stack() = default;
  explicit Stack(std::vector<std::shared_ptr<const Animation>> layers)
      : layers_(std::move(layers)) {}
  BodyPose eval(const AnimationContext& ctx) const override;

 private:
  std::vector<std::shared_ptr<const Animation>> layers_;
};

}  // namespace hexa::posture
