#include "posture/leveling.hpp"

#include <cmath>

namespace hexa::posture {

namespace {

// Below this the decay snaps to exactly zero, well inside the pipeline's
// neutral-pose check, so a waiting preset change can go.
constexpr float kDecaySnap = 1e-4f;

float lpf(float prev, float raw, float tau, float dt) {
  return tau > 0.0f ? prev + (raw - prev) * (dt / (tau + dt)) : raw;
}

}  // namespace

LevelingGate leveling_gate(bool enabled, bool preset_listed, bool sample_fresh,
                           bool preset_change_pending,
                           gait::EngineState state) {
  if (!enabled || !preset_listed || !sample_fresh || preset_change_pending) {
    return LevelingGate::kDecay;
  }
  using E = gait::EngineState;
  if (state == E::STAND || state == E::ENGAGING || state == E::GAIT ||
      state == E::SETTLING) {
    return LevelingGate::kIntegrate;
  }
  return LevelingGate::kHold;
}

Leveler::Leveler(const ::hexa::config::StabilizationConfig& cfg) : cfg_(cfg) {}

void Leveler::step(LevelingGate gate, bool walking, float target_roll,
                   float target_pitch, std::optional<TiltSample> sample,
                   float dt) {
  if (!sample.has_value()) {
    filtered_.reset();
  } else if (!filtered_.has_value()) {
    filtered_ = sample;
  } else {
    filtered_->roll = lpf(filtered_->roll, sample->roll, cfg_.measurement_tau, dt);
    filtered_->pitch =
        lpf(filtered_->pitch, sample->pitch, cfg_.measurement_tau, dt);
  }

  switch (gate) {
    case LevelingGate::kHold:
      return;
    case LevelingGate::kDecay: {
      roll_ = lpf(roll_, 0.0f, cfg_.decay_tau, dt);
      pitch_ = lpf(pitch_, 0.0f, cfg_.decay_tau, dt);
      if (std::hypot(roll_, pitch_) < kDecaySnap) {
        roll_ = 0.0f;
        pitch_ = 0.0f;
      }
      return;
    }
    case LevelingGate::kIntegrate:
      break;
  }
  if (!filtered_.has_value()) {
    return;
  }
  const float ki = walking ? cfg_.ki_gait : cfg_.ki_stand;
  roll_ += ki * dt * (target_roll - filtered_->roll);
  pitch_ += ki * dt * (target_pitch - filtered_->pitch);
  // One disc, like the pose envelope's tilt pair.
  const float r = std::hypot(roll_, pitch_);
  if (r > cfg_.max_correction && r > 0.0f) {
    const float k = cfg_.max_correction / r;
    roll_ *= k;
    pitch_ *= k;
  }
}

void Leveler::limit_to(float roll, float pitch) {
  if (std::hypot(roll, pitch) < std::hypot(roll_, pitch_)) {
    roll_ = roll;
    pitch_ = pitch;
  }
}

void Leveler::reset() {
  roll_ = 0.0f;
  pitch_ = 0.0f;
  filtered_.reset();
}

}  // namespace hexa::posture
