// Gait engine — orchestrates clock, strategy, and the engagement / pause
// controllers. Port of engine.py. The engine is the only stateful component in
// the gait chain; strategies stay pure. update() routes between modes based on
// the commanded body velocity through the
// FOLDED/INITIALIZE/STAND/ENGAGING/GAIT/PAUSING/PAUSED/RESUMING/FOLDING/
// RESEATING state machine.
#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "hexa_gait_cpp/clock.hpp"
#include "hexa_gait_cpp/engagement.hpp"
#include "hexa_gait_cpp/fold.hpp"
#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/initialize.hpp"
#include "hexa_gait_cpp/kinematics_stub.hpp"
#include "hexa_gait_cpp/leg_output.hpp"
#include "hexa_gait_cpp/pause.hpp"
#include "hexa_gait_cpp/reseat.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {

enum class EngineState {
  FOLDED,
  INITIALIZE,
  STAND,
  ENGAGING,
  GAIT,
  PAUSING,
  PAUSED,
  RESUMING,
  FOLDING,
  RESEATING,
};

// Engine-internal knobs, sourced entirely from gait.yaml. None are on the wire.
struct EngineConfig {
  double stride_length = 0.0;
  double min_swing_time = 0.0;
  double max_swing_time = 0.0;
  double step_height = 0.0;
  double swing_width = 0.0;
  double controller_dt = 0.0;
  double cmd_zero_tol = 0.0;
  double pause_debounce_delay = 0.0;
  double pause_to_reseat_delay = 0.0;
  double gait_change_pause_to_reseat_delay = 0.0;
  double max_reset_time = 0.0;
  double init_pair_swing_time = 0.0;
  double init_lift_body_time = 0.0;
  double init_swing_clearance = 0.0;
  double init_place_feet_clearance = 0.0;
  double reseat_pose_settle_delay = 0.0;
  double reseat_height_change_threshold = 0.0;
  double reseat_pair_swing_time = 0.0;
  double reseat_pair_dwell_time = 0.0;
  double reseat_swing_clearance = 0.0;
};

// Per-leg body-frame stance target as an integral from touchdown. Removes
// foot-scrub under varying velocity; reproduces the closed-form stance Bezier
// under constant velocity.
class StanceIntegrator {
 public:
  StanceIntegrator();
  void seed(const std::map<std::string, Vec3>& last_targets,
            const std::map<std::string, bool>& last_stance);
  // Returns the integrated body-frame target if in stance, else nullopt.
  std::optional<Vec3> step(const std::string& name, bool in_stance,
                           const Vec3& swing_target,
                           std::pair<double, double> v_leg, double dt);
  void reset();
  bool is_stance(const std::string& name) const { return is_stance_.at(name); }

 private:
  std::map<std::string, Vec3> anchor_;
  std::map<std::string, bool> is_stance_;
};

// Per-leg latched swing plan, captured at lift-off and held until touchdown.
class SwingPlanner {
 public:
  SwingPlanner();
  void liftoff(const std::string& name, const Vec3& origin, const Vec3& target,
               std::pair<double, double> v_leg, double swing_time,
               int identity_y_sign_val);
  void touchdown(const std::string& name);
  Vec3 evaluate(const std::string& name, double phase_in_swing,
                double swing_clearance, double swing_width,
                double controller_dt) const;
  void reset();
  bool is_swing(const std::string& name) const { return is_swing_.at(name); }
  const Vec3& target(const std::string& name) const { return target_.at(name); }

 private:
  std::map<std::string, Vec3> origin_;
  std::map<std::string, Vec3> target_;
  std::map<std::string, std::pair<double, double>> v_leg_;
  std::map<std::string, double> swing_time_;
  std::map<std::string, int> identity_y_sign_;
  std::map<std::string, bool> is_swing_;
};

class Engine {
 public:
  // leg_specs and reseat_geometry must be supplied together (both empty
  // disables reseat, both set enables it). strategy_name must match the
  // registry key for the supplied strategy.
  Engine(EngineConfig config, std::unique_ptr<Strategy> strategy,
         std::string strategy_name, std::map<std::string, Vec3> nominal_stance,
         std::map<std::string, Vec3> initial_stance, double coxa_to_bottom,
         std::map<std::string, LegContext> leg_contexts,
         std::optional<std::map<std::string, kin::LegSpec>> leg_specs =
             std::nullopt,
         std::optional<ReseatGeometry> reseat_geometry = std::nullopt);

  EngineState state() const { return state_; }
  double master_phase() const;
  const std::string& strategy_name() const { return strategy_name_; }
  std::optional<std::string> pending_strategy_name() const {
    return pending_strategy_name_;
  }

  bool set_strategy(const std::string& name);
  bool start_initialize();
  bool start_fold();
  bool request_fold();
  void set_target_height(double target_height);

  std::map<std::string, LegOutput> update(double dt,
                                          std::pair<double, double> v_body_xy,
                                          double omega_z);

 private:
  void apply_strategy(const std::string& name);
  std::unique_ptr<InitializeController> build_initialize();
  std::unique_ptr<FoldController> build_fold();
  std::unique_ptr<PauseController> build_pause();
  std::unique_ptr<EngagementController> build_engagement();
  std::unique_ptr<ReseatController> build_reseat(
      const std::map<std::string, Vec3>& target_stance);
  void commit_new_nominal(const std::map<std::string, Vec3>& new_nominal,
                          double applied_height);

  bool cmd_is_zero(std::pair<double, double> v_body_xy, double omega_z) const;
  std::map<std::string, LegOutput> emit_stand() const;
  std::map<std::string, LegOutput> emit_held() const;
  std::map<std::string, LegOutput> tick_gait(double dt,
                                             std::pair<double, double> v_body_xy,
                                             double omega_z, bool cmd_zero);
  void enter_pausing();
  void enter_resuming();
  std::map<std::string, LegOutput> tick_pause(double dt);
  std::map<std::string, LegOutput> tick_reseat(double dt);
  std::map<std::string, LegOutput> tick_fold(double dt);
  std::map<std::string, LegOutput> tick_engagement(
      double dt, std::pair<double, double> v_body_xy, double omega_z);
  void capture_state(const std::map<std::string, LegOutput>& out);

  EngineConfig config_;
  std::unique_ptr<Strategy> strategy_;
  std::string strategy_name_;
  std::map<std::string, Vec3> nominal_;
  std::map<std::string, Vec3> initial_;
  double coxa_to_bottom_;
  std::map<std::string, LegContext> legs_;
  std::optional<std::map<std::string, kin::LegSpec>> leg_specs_;
  std::optional<ReseatGeometry> reseat_geometry_;

  std::optional<GaitClock> clock_;
  StanceIntegrator stance_;
  SwingPlanner swing_;
  std::unique_ptr<PauseController> pause_;
  std::unique_ptr<EngagementController> engagement_;
  std::unique_ptr<InitializeController> initialize_;
  std::unique_ptr<FoldController> fold_;
  std::unique_ptr<ReseatController> reseat_;

  EngineState state_ = EngineState::FOLDED;
  std::map<std::string, Vec3> last_targets_;
  std::map<std::string, bool> last_stance_;
  double cmd_zero_elapsed_ = 0.0;
  double paused_elapsed_ = 0.0;
  std::map<std::string, bool> last_swing_flags_;

  double applied_height_ = 0.0;
  double target_height_ = 0.0;
  double height_stable_elapsed_ = 0.0;
  bool pending_fold_ = false;
  std::optional<std::string> pending_strategy_name_;

  // Snapshot of the reseat target, set when RESEATING is entered.
  std::map<std::string, Vec3> reseat_target_stance_;
  double reseat_target_height_ = 0.0;
};

// ── YAML builders (use the kinematics stub until hexa_kinematics is ported) ──

std::map<std::string, Vec3> nominal_stance_from_yaml(
    const std::string& geometry_yaml, const std::string& standing_pose_yaml);

ReseatGeometry reseat_geometry_from_yaml(const std::string& geometry_yaml,
                                         const std::string& standing_pose_yaml);

std::map<std::string, Vec3> initial_stance_from_yaml(
    const std::string& geometry_yaml);

std::map<std::string, LegContext> build_leg_contexts(
    const std::string& geometry_yaml, const std::string& standing_pose_yaml);

// Wire string for /gait/state (folded, initialize, stand, ...). Mirrors the
// Python EngineState enum's .value.
std::string state_value(EngineState s);

// Uppercase name for log messages (FOLDED, INITIALIZE, STAND, ...). Mirrors the
// Python enum's .name.
std::string state_name(EngineState s);

}  // namespace hexa_gait
