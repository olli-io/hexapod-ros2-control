// Pause: soft GAIT release that preserves the in-flight gait state. Float fork of
// pause.hpp (plan part 06).
//
// LOWERING lowers each currently-airborne leg straight down to nominal.z (XY
// frozen); stance legs hold. PAUSED holds every foot once all descents land.
// The gait clock is preserved by the engine so the operator can re-engage
// without resetting the cycle.
#pragma once

#include <map>
#include <string>

#include "gait/types.hpp"

namespace hexa::gait {

enum class PauseState { LOWERING, PAUSED };

class PauseController {
 public:
  PauseController(std::map<std::string, Vec3> nominal_stance,
                  float swing_clearance, float swing_width, float controller_dt,
                  float descent_speed, float min_reset_time,
                  float max_reset_time);

  PauseState state() const { return state_; }

  // Seed the controller with the legs' current pose at pause time.
  // swing_flags[n] == true means leg n was airborne and will be lowered to
  // nominal.z; stance legs hold. No airborne leg -> straight to PAUSED.
  void begin(const std::map<std::string, Vec3>& last_targets,
             const std::map<std::string, bool>& swing_flags);

  std::map<std::string, LegOutput> update(float dt);

  // Last per-leg foot positions emitted (for engine handoff).
  std::map<std::string, Vec3> positions() const { return positions_; }

 private:
  struct LegDescent {
    Vec3 origin = Vec3::Zero();
    Vec3 target = Vec3::Zero();
    float duration = 0.0f;
    float elapsed = 0.0f;
  };

  std::map<std::string, LegOutput> tick(float dt);
  std::map<std::string, LegOutput> emit_held() const;
  float adaptive_descent_time(float distance_z) const;
  Vec3 descent_point(const LegDescent& descent) const;

  std::map<std::string, Vec3> nominal_;
  float swing_clearance_;
  float swing_width_;
  float controller_dt_;
  float descent_speed_;
  float min_reset_time_;
  float max_reset_time_;

  PauseState state_ = PauseState::PAUSED;
  std::map<std::string, Vec3> positions_;
  std::map<std::string, LegDescent> descents_;
};

}  // namespace hexa::gait
