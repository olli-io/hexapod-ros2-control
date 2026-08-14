// Stand -> Gait engagement. Float fork of engagement.hpp (plan part 06).
//
// One per-leg state machine (INITIAL_STANCE / INITIAL_SWING / GAIT_LIKE) runs
// one full master cycle from a standing start, under a smoothstep
// body-velocity envelope, and hands the clock to the engine at master = 1.
//
// There is no resume counterpart: a stop runs through EngineState::SETTLING,
// which never stops the gait clock, so a command that comes back is picked up
// by the engine's own tick with nothing to re-enter.
#pragma once

#include <map>
#include <string>
#include <utility>

#include "gait/gaits/base.hpp"
#include "gait/types.hpp"

namespace hexa::gait {

enum class EngagementState { IDLE, ENGAGING, DONE };

class EngagementController {
 public:
  EngagementController(std::map<std::string, Vec3> nominal_stance,
                       float stride_length, float stride_length_radial,
                       float min_cycle_time, float max_cycle_time,
                       float duty_factor, float swing_phase_margin,
                       const SwingProfile& swing, float controller_dt);

  EngagementState state() const { return state_; }

  // Master phase to seed the engine clock with on GAIT handoff (master mod 1).
  float exit_master() const { return pymod(master_, 1.0f); }

  // Current internal body velocity (v_x, v_y, omega_z). Diagnostics/tests only.
  Vec3 v_body() const { return Vec3(v_body_x_, v_body_y_, omega_); }

  // Master horizon over which the body-velocity smoothstep ramps. Tests only.
  float smoothstep_window() const { return smoothstep_window_; }

  // Arm the engagement from a STAND start.
  void begin(const Strategy& strategy,
             const std::map<std::string, LegContext>& leg_contexts);

  std::map<std::string, LegOutput> update(float dt,
                                          std::pair<float, float> v_cmd_xy,
                                          float omega_cmd);

 private:
  std::map<std::string, LegOutput> emit_nominal_stance() const;

  std::map<std::string, Vec3> nominal_;
  float stride_length_;
  float stride_length_radial_;
  float min_cycle_time_;
  float max_cycle_time_;
  float duty_factor_;
  // Phase at which swing ends; the nominal window already shrunk by the margin.
  float swing_end_;
  SwingProfile swing_;
  float controller_dt_;

  EngagementState state_ = EngagementState::IDLE;
  const Strategy* strategy_ = nullptr;
  std::map<std::string, LegContext> leg_contexts_;
  std::map<std::string, bool> is_initial_swing_;
  std::map<std::string, float> first_lift_off_master_;
  std::map<std::string, float> first_touchdown_master_;
  float smoothstep_window_ = 1.0f;

  float master_ = 0.0f;
  float v_body_x_ = 0.0f;
  float v_body_y_ = 0.0f;
  float omega_ = 0.0f;
  std::map<std::string, Vec3> foot_position_;
  std::map<std::string, Vec3> lift_off_position_;
  std::map<std::string, bool> has_lifted_off_;
};

}  // namespace hexa::gait
