// Pause: soft GAIT release that preserves the in-flight gait state. Port of
// pause.py.
//
// LOWERING lowers each currently-airborne leg straight down to nominal.z (XY
// frozen); stance legs hold. PAUSED holds every foot once all descents land.
// The gait clock is preserved by the engine so the operator can re-engage
// without resetting the cycle.
#pragma once

#include <map>
#include <string>

#include "hexa_gait_cpp/leg_output.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {

enum class PauseState { LOWERING, PAUSED };

class PauseController {
 public:
  PauseController(std::map<std::string, Vec3> nominal_stance,
                  double swing_clearance, double swing_width,
                  double controller_dt, double descent_speed,
                  double min_reset_time, double max_reset_time);

  PauseState state() const { return state_; }

  // Seed the controller with the legs' current pose at pause time.
  // swing_flags[n] == true means leg n was airborne and will be lowered to
  // nominal.z; stance legs hold. No airborne leg -> straight to PAUSED.
  void begin(const std::map<std::string, Vec3>& last_targets,
             const std::map<std::string, bool>& swing_flags);

  std::map<std::string, LegOutput> update(double dt);

  // Last per-leg foot positions emitted (for engine handoff).
  std::map<std::string, Vec3> positions() const { return positions_; }

 private:
  struct LegDescent {
    Vec3 origin = Vec3::Zero();
    Vec3 target = Vec3::Zero();
    double duration = 0.0;
    double elapsed = 0.0;
  };

  std::map<std::string, LegOutput> tick(double dt);
  std::map<std::string, LegOutput> emit_held() const;
  double adaptive_descent_time(double distance_z) const;
  Vec3 descent_point(const LegDescent& descent) const;

  std::map<std::string, Vec3> nominal_;
  double swing_clearance_;
  double swing_width_;
  double controller_dt_;
  double descent_speed_;
  double min_reset_time_;
  double max_reset_time_;

  PauseState state_ = PauseState::PAUSED;
  std::map<std::string, Vec3> positions_;
  std::map<std::string, LegDescent> descents_;
};

}  // namespace hexa_gait
