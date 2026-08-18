// Stand -> Gait engagement: one full master cycle from a standing start under a
// smoothstep body-velocity envelope, handing the clock to the engine at
// master = 1. There is no resume counterpart — a stop runs through
// EngineState::SETTLING, which never stops the gait clock.
#pragma once

#include <map>
#include <string>
#include <utility>

#include "gait/gaits/base.hpp"
#include "gait/types.hpp"

namespace hexa::gait {

// SHIFTING is the lead-in: every foot planted while the master clock winds up to
// the first lift-off, so the body has left the foot before it goes.
enum class EngagementState { IDLE, SHIFTING, ENGAGING, DONE };

class EngagementController {
 public:
  // `shift_time` is the SHIFTING lead-in; zero skips it and the first leg lifts
  // on the first tick. Only the quadruped leg set owes one — a tripod's initial
  // swing leaves a tripod standing, which the body is already inside, and any
  // delay there is a walk that does not start when the stick moves.
  EngagementController(std::map<std::string, Vec3> nominal_stance,
                       float stride_length, float stride_length_radial,
                       float min_cycle_time, float max_cycle_time,
                       float duty_factor, float swing_phase_margin,
                       const SwingProfile& swing, float controller_dt,
                       float shift_time = 0.0f);

  EngagementState state() const { return state_; }

  // Master phase to seed the engine clock with on GAIT handoff (master mod 1).
  float exit_master() const { return pymod(master_, 1.0f); }

  // Diagnostics/tests only.
  Vec3 v_body() const { return Vec3(v_body_x_, v_body_y_, omega_); }

  // Master horizon the body-velocity smoothstep ramps over. Tests only.
  float smoothstep_window() const { return smoothstep_window_; }

  // `leg_contexts` is the set of legs that WALK, not necessarily all six: a
  // parked pair is simply absent, and the output map omits it for the engine to
  // overlay. Passing the middles in while they are parked would price the walk
  // against the longest lever arms on the chassis.
  void begin(const Strategy& strategy,
             const std::map<std::string, LegContext>& leg_contexts);

  std::map<std::string, LegOutput> update(float dt,
                                          std::pair<float, float> v_cmd_xy,
                                          float omega_cmd);

 private:
  std::map<std::string, LegOutput> emit_nominal_stance() const;
  std::map<std::string, LegOutput> tick_shift(float dt);
  bool walks(const std::string& name) const;

  std::map<std::string, Vec3> nominal_;
  float stride_length_;
  float stride_length_radial_;
  float min_cycle_time_;
  float max_cycle_time_;
  float duty_factor_;
  // Swing end, with the margin already taken off the nominal window.
  float swing_end_;
  SwingProfile swing_;
  float controller_dt_;
  float shift_time_;

  EngagementState state_ = EngagementState::IDLE;
  const Strategy* strategy_ = nullptr;
  std::map<std::string, LegContext> leg_contexts_;
  std::map<std::string, bool> is_initial_swing_;
  std::map<std::string, float> first_lift_off_master_;
  std::map<std::string, float> first_touchdown_master_;
  float smoothstep_window_ = 1.0f;

  float t_in_shift_ = 0.0f;
  float master_ = 0.0f;
  float v_body_x_ = 0.0f;
  float v_body_y_ = 0.0f;
  float omega_ = 0.0f;
  std::map<std::string, Vec3> foot_position_;
  std::map<std::string, Vec3> lift_off_position_;
  std::map<std::string, bool> has_lifted_off_;
};

}  // namespace hexa::gait
