// Animation stack. An animation is a pure function from AnimationContext to a
// BodyPose offset: no I/O, no clocks, no engine — the posture controller owns
// the clock and feeds `t` in via the context. A Stack sums its layers with
// pose.add, so it is valid only for small offsets.
#pragma once

#include <memory>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "posture/pose.hpp"

namespace hexa::posture {

// Read-only inputs an animation may consult. An empty optional means the signal
// has not been observed yet.
struct AnimationContext {
  // The only clock source, in seconds.
  float t = 0.0f;
  // Latest /cmd_vel non-zero. Gates pose-mode-only animations (idle breathing)
  // and gait-only ones (sway, rolls).
  bool walking = false;
  // Master gait phase in [0, 1), 0 at lift-off for the reference leg.
  std::optional<float> master_phase = std::nullopt;
  // Active gait strategy; an empty view means unknown.
  std::string_view gait_name = "";
  // Low-pass-filtered XY centroid of the support polygon, body frame (m).
  std::optional<std::pair<float, float>> support_centroid_xy = std::nullopt;
  // Max foot lift above the stance mean (m) across swing legs, clamped >= 0.
  std::optional<float> swing_lift_z = std::nullopt;
};

class Animation {
 public:
  virtual ~Animation() = default;
  virtual BodyPose eval(const AnimationContext& ctx) const = 0;
};

class Still : public Animation {
 public:
  BodyPose eval(const AnimationContext& ctx) const override;
};

// Slow vertical bob when standing still, off during walking.
class Breathing : public Animation {
 public:
  explicit Breathing(float amplitude = 0.005f, float period = 4.0f)
      : amplitude_(amplitude), period_(period) {}
  BodyPose eval(const AnimationContext& ctx) const override;

 private:
  float amplitude_;  // m, peak-to-zero height offset
  float period_;     // s, one full breath cycle
};

// Translates the body in XY to track the support-polygon centroid. Off when not
// walking or the centroid has not been observed.
class GaitSway : public Animation {
 public:
  explicit GaitSway(float gain = 1.0f, float strength = 0.5f)
      : gain_(gain), strength_(strength) {}
  BodyPose eval(const AnimationContext& ctx) const override;

 private:
  float gain_;      // feedforward gain on the centroid
  float strength_;  // user-facing attenuator in [0, 1]
};

// Lifts the body in +z to match the swing arc. Tripod-only.
class GaitBounce : public Animation {
 public:
  explicit GaitBounce(float arc_height = 0.02f, float step_height_ref = 0.06f)
      : arc_height_(arc_height), step_height_ref_(step_height_ref) {}
  BodyPose eval(const AnimationContext& ctx) const override;

 private:
  float arc_height_;       // max body lift (m) at swing apex
  float step_height_ref_;  // reference foot swing apex (m) for normalisation
};

// Heave (z) + pitch, phase-locked, tripod-only.
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

// VerticalBodyRoll in the transverse plane: sway (y) + yaw.
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

// Vertical + horizontal rolls a quarter cycle apart, so the (y, z) translation
// and (pitch, yaw) rotation trace circles. Tripod-only.
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

// Sums its layers' offsets via pose.add.
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
