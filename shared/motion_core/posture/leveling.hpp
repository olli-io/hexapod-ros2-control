// Body stabilization: a slow integral loop that levels the body to gravity from
// an IMU attitude. The posture stack commands tilt relative to the feet plane;
// the IMU measures it relative to gravity, which on a slope is the commanded
// tilt plus the slope plus the correction already applied:
//
//   measured ~= slope + target + c
//   c += ki * dt * (target - lpf(measured))   ->   c = -slope at rest
//
// so the operator's tilt and the gait animations read relative to gravity, and
// only the slow residual is corrected. Pure: the caller supplies dt and the
// sample, and decides the gate.
#pragma once

#include <optional>

#include "config_generated.hpp"
#include "gait/engine.hpp"

namespace hexa::posture {

// IMU roll/pitch of the body relative to gravity, rad, REP-103.
struct TiltSample {
  float roll = 0.0f;
  float pitch = 0.0f;
};

// What the correction does this tick.
enum class LevelingGate {
  kIntegrate,  // close the loop on the sample
  kHold,       // keep the correction; the stance is in motion
  kDecay,      // ease the correction out to zero
};

// kDecay unless enabled, on a listed preset, with a fresh sample and no preset
// change waiting on a neutral pose; then kIntegrate where the feet are at
// nominal stance, kHold elsewhere.
LevelingGate leveling_gate(bool enabled, bool preset_listed, bool sample_fresh,
                           bool preset_change_pending,
                           gait::EngineState state);

class Leveler {
 public:
  explicit Leveler(const ::hexa::config::StabilizationConfig& cfg);

  // Advance the correction. `target_*` is the tilt the posture stack commands
  // this tick, before the correction. `walking` selects ki_gait over ki_stand.
  // A missing sample drops the measurement filter, so the next one re-seeds it.
  void step(LevelingGate gate, bool walking, float target_roll,
            float target_pitch, std::optional<TiltSample> sample, float dt);

  // Back-calculation anti-windup: the share of the correction that survived
  // the pose clamp. Never grows the correction.
  void limit_to(float roll, float pitch);

  void reset();

  float roll() const { return roll_; }
  float pitch() const { return pitch_; }

 private:
  ::hexa::config::StabilizationConfig cfg_;
  float roll_ = 0.0f;
  float pitch_ = 0.0f;
  std::optional<TiltSample> filtered_;
};

}  // namespace hexa::posture
