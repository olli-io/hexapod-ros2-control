// Stand -> Gait engagement and Paused -> Gait resume. Port of engagement.py.
//
// One per-leg state machine (INITIAL_STANCE / INITIAL_SWING / GAIT_LIKE) drives
// two entry points: engage mode (begin: STAND -> GAIT, one full master cycle
// with a smoothstep body-velocity envelope) and resume mode (begin_resume:
// PAUSED -> GAIT, seeded from the paused master phase, no envelope). The handoff
// to GAIT carries no position step.
#pragma once

#include <map>
#include <string>
#include <utility>

#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/leg_output.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {

enum class EngagementState { IDLE, ENGAGING, DONE };

class EngagementController {
 public:
  EngagementController(std::map<std::string, Vec3> nominal_stance,
                       double stride_length, double min_cycle_time,
                       double max_cycle_time, double duty_factor,
                       double swing_clearance, double swing_width,
                       double controller_dt);

  EngagementState state() const { return state_; }

  // Master phase to seed the engine clock with on GAIT handoff (master mod 1).
  double exit_master() const { return pymod(master_, 1.0); }

  // Current internal body velocity (v_x, v_y, omega_z). Diagnostics/tests only.
  Vec3 v_body() const { return Vec3(v_body_x_, v_body_y_, omega_); }

  // Master horizon over which the body-velocity smoothstep ramps. Tests only.
  double smoothstep_window() const { return smoothstep_window_; }

  // Arm the engagement from a STAND start.
  void begin(const Strategy& strategy,
             const std::map<std::string, LegContext>& leg_contexts);

  // Arm the engagement from a PAUSED start, resuming from master_phase.
  void begin_resume(const Strategy& strategy,
                    const std::map<std::string, LegContext>& leg_contexts,
                    const std::map<std::string, Vec3>& last_targets,
                    const std::map<std::string, bool>& prev_swing_flags,
                    double master_phase);

  std::map<std::string, LegOutput> update(double dt,
                                          std::pair<double, double> v_cmd_xy,
                                          double omega_cmd);

 private:
  std::map<std::string, LegOutput> emit_nominal_stance() const;

  std::map<std::string, Vec3> nominal_;
  double stride_length_;
  double min_cycle_time_;
  double max_cycle_time_;
  double duty_factor_;
  double swing_end_;
  double swing_clearance_;
  double swing_width_;
  double controller_dt_;

  EngagementState state_ = EngagementState::IDLE;
  // "engage" — STAND -> GAIT; "resume" — PAUSED -> GAIT.
  std::string mode_ = "engage";

  const Strategy* strategy_ = nullptr;
  std::map<std::string, LegContext> leg_contexts_;
  std::map<std::string, bool> is_initial_swing_;
  std::map<std::string, double> first_lift_off_master_;
  std::map<std::string, double> first_touchdown_master_;
  double smoothstep_window_ = 1.0;

  double master_ = 0.0;
  double v_body_x_ = 0.0;
  double v_body_y_ = 0.0;
  double omega_ = 0.0;
  std::map<std::string, Vec3> foot_position_;
  std::map<std::string, Vec3> lift_off_position_;
  std::map<std::string, bool> has_lifted_off_;
  std::map<std::string, bool> has_completed_first_swing_;
};

}  // namespace hexa_gait
