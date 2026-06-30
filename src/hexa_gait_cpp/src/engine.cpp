#include "hexa_gait_cpp/engine.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "hexa_gait_cpp/gaits/registry.hpp"
#include "hexa_gait_cpp/validation.hpp"

namespace hexa_gait {

namespace {
// Float-noise epsilon for "is the user still moving the D-pad?".
constexpr double kHeightNoiseEpsilon = 1e-6;
// Inclusive tolerance for the swing->stance boundary (absorbs float noise on
// the touchdown seam at gaits where 1 - duty_factor is not representable).
constexpr double kStanceSeamEpsilon = 1e-9;

// Per-gait cycle-time bounds derived from swing-phase bounds. Both ends scale
// by 1 / (1 - beta) so the swing-phase foot-velocity envelope is gait-agnostic.
std::pair<double, double> cycle_time_bounds(const EngineConfig& cfg,
                                            double beta) {
  if (beta >= 1.0) {
    return {cfg.max_swing_time, cfg.max_swing_time};
  }
  const double scale = 1.0 / (1.0 - beta);
  return {cfg.min_swing_time * scale, cfg.max_swing_time * scale};
}
}  // namespace

// ───────────────────────────── StanceIntegrator ─────────────────────────────

StanceIntegrator::StanceIntegrator() {
  for (const auto& n : LEG_NAMES) {
    anchor_[n] = Vec3::Zero();
    is_stance_[n] = false;
  }
}

void StanceIntegrator::seed(const std::map<std::string, Vec3>& last_targets,
                            const std::map<std::string, bool>& last_stance) {
  for (const auto& n : LEG_NAMES) {
    anchor_[n] = last_targets.at(n);
    is_stance_[n] = last_stance.at(n);
  }
}

std::optional<Vec3> StanceIntegrator::step(const std::string& name,
                                           bool in_stance,
                                           const Vec3& swing_target,
                                           std::pair<double, double> v_leg,
                                           double dt) {
  if (!in_stance) {
    is_stance_[name] = false;
    return std::nullopt;
  }
  if (!is_stance_[name]) {
    anchor_[name] = swing_target;
    is_stance_[name] = true;
    return anchor_[name];
  }
  Vec3& a = anchor_[name];
  a = Vec3(a[0] - v_leg.first * dt, a[1] - v_leg.second * dt, a[2]);
  return a;
}

void StanceIntegrator::reset() {
  for (const auto& n : LEG_NAMES) {
    is_stance_[n] = false;
  }
}

// ─────────────────────────────── SwingPlanner ───────────────────────────────

SwingPlanner::SwingPlanner() {
  for (const auto& n : LEG_NAMES) {
    origin_[n] = Vec3::Zero();
    target_[n] = Vec3::Zero();
    v_leg_[n] = {0.0, 0.0};
    swing_time_[n] = 0.0;
    identity_y_sign_[n] = 1;
    is_swing_[n] = false;
  }
}

void SwingPlanner::liftoff(const std::string& name, const Vec3& origin,
                           const Vec3& target, std::pair<double, double> v_leg,
                           double swing_time, int identity_y_sign_val) {
  origin_[name] = origin;
  target_[name] = target;
  v_leg_[name] = v_leg;
  swing_time_[name] = swing_time;
  identity_y_sign_[name] = identity_y_sign_val;
  is_swing_[name] = true;
}

void SwingPlanner::touchdown(const std::string& name) {
  is_swing_[name] = false;
}

Vec3 SwingPlanner::evaluate(const std::string& name, double phase_in_swing,
                            double swing_clearance, double swing_width,
                            double controller_dt) const {
  const auto& v = v_leg_.at(name);
  // Stance-frame foot velocity is -v_leg; pass it as both endpoints so the
  // Bezier's C1 nodes match the stance-frame velocity at lift-off and touchdown.
  const Vec3 v_match(-v.first, -v.second, 0.0);
  return swing_arc(phase_in_swing, origin_.at(name), target_.at(name),
                   swing_clearance, swing_width, identity_y_sign_.at(name),
                   swing_time_.at(name), controller_dt, v_match, v_match);
}

void SwingPlanner::reset() {
  for (const auto& n : LEG_NAMES) {
    is_swing_[n] = false;
  }
}

// ─────────────────────────────────── Engine ─────────────────────────────────

Engine::Engine(EngineConfig config, std::unique_ptr<Strategy> strategy,
               std::string strategy_name,
               std::map<std::string, Vec3> nominal_stance,
               std::map<std::string, Vec3> initial_stance,
               double coxa_to_bottom,
               std::map<std::string, LegContext> leg_contexts,
               std::optional<std::map<std::string, kin::LegSpec>> leg_specs,
               std::optional<ReseatGeometry> reseat_geometry)
    : config_(config),
      strategy_(std::move(strategy)),
      strategy_name_(std::move(strategy_name)),
      coxa_to_bottom_(coxa_to_bottom),
      legs_(std::move(leg_contexts)),
      leg_specs_(std::move(leg_specs)),
      reseat_geometry_(std::move(reseat_geometry)) {
  require_all_legs(nominal_stance, "nominal_stance");
  require_all_legs(initial_stance, "initial_stance");
  require_all_legs(legs_, "leg_contexts");
  if (leg_specs_.has_value() != reseat_geometry_.has_value()) {
    throw std::invalid_argument(
        "leg_specs and reseat_geometry must be supplied together");
  }

  for (const auto& n : LEG_NAMES) {
    nominal_[n] = nominal_stance.at(n);
    initial_[n] = initial_stance.at(n);
  }

  clock_.emplace(strategy_->phase_offsets());
  pause_ = build_pause();
  engagement_ = build_engagement();
  initialize_ = build_initialize();

  state_ = EngineState::FOLDED;
  last_targets_ = initial_;
  for (const auto& n : LEG_NAMES) {
    last_stance_[n] = true;
    last_swing_flags_[n] = false;
  }
}

double Engine::master_phase() const {
  if (state_ == EngineState::ENGAGING || state_ == EngineState::RESUMING) {
    return engagement_->exit_master();
  }
  return clock_->master();
}

void Engine::apply_strategy(const std::string& name) {
  strategy_ = strategies().at(name)();
  strategy_name_ = name;
  clock_.emplace(strategy_->phase_offsets());
  engagement_ = build_engagement();
}

bool Engine::set_strategy(const std::string& name) {
  if (strategies().find(name) == strategies().end()) {
    return false;
  }
  if (state_ == EngineState::STAND) {
    if (name != strategy_name_) {
      apply_strategy(name);
    }
    return true;
  }
  if (state_ == EngineState::GAIT || state_ == EngineState::PAUSING ||
      state_ == EngineState::PAUSED || state_ == EngineState::RESEATING) {
    if (!pending_strategy_name_.has_value() && name == strategy_name_) {
      return true;
    }
    pending_strategy_name_ = name;
    return true;
  }
  return false;
}

bool Engine::start_initialize() {
  if (state_ != EngineState::FOLDED) {
    return false;
  }
  initialize_ = build_initialize();
  state_ = EngineState::INITIALIZE;
  return true;
}

bool Engine::start_fold() {
  if (state_ != EngineState::STAND) {
    return false;
  }
  fold_ = build_fold();
  state_ = EngineState::FOLDING;
  return true;
}

bool Engine::request_fold() {
  if (state_ == EngineState::FOLDED || state_ == EngineState::FOLDING) {
    return false;
  }
  pending_fold_ = true;
  return true;
}

void Engine::set_target_height(double target_height) {
  if (std::abs(target_height - target_height_) > kHeightNoiseEpsilon) {
    height_stable_elapsed_ = 0.0;
  }
  target_height_ = target_height;
}

std::unique_ptr<InitializeController> Engine::build_initialize() {
  return std::make_unique<InitializeController>(
      initial_, nominal_, coxa_to_bottom_, config_.init_pair_swing_time,
      config_.init_lift_body_time, config_.init_swing_clearance,
      config_.init_place_feet_clearance, config_.swing_width,
      config_.controller_dt);
}

std::unique_ptr<FoldController> Engine::build_fold() {
  return std::make_unique<FoldController>(
      initial_, nominal_, coxa_to_bottom_, config_.init_pair_swing_time,
      config_.init_lift_body_time, config_.init_swing_clearance,
      config_.init_place_feet_clearance, config_.swing_width,
      config_.controller_dt);
}

std::unique_ptr<PauseController> Engine::build_pause() {
  return std::make_unique<PauseController>(
      nominal_, config_.step_height, config_.swing_width, config_.controller_dt,
      /*descent_speed=*/config_.stride_length / config_.min_swing_time,
      /*min_reset_time=*/config_.min_swing_time, config_.max_reset_time);
}

std::unique_ptr<EngagementController> Engine::build_engagement() {
  const double beta = strategy_->duty_factor();
  const auto [min_cycle_time, max_cycle_time] = cycle_time_bounds(config_, beta);
  return std::make_unique<EngagementController>(
      nominal_, config_.stride_length, min_cycle_time, max_cycle_time, beta,
      config_.step_height, config_.swing_width, config_.controller_dt);
}

std::unique_ptr<ReseatController> Engine::build_reseat(
    const std::map<std::string, Vec3>& target_stance) {
  // Always reseat from where the feet actually are (last_targets_ is rewritten
  // every tick).
  return std::make_unique<ReseatController>(
      last_targets_, target_stance, config_.reseat_pair_swing_time,
      config_.reseat_pair_dwell_time, config_.reseat_swing_clearance,
      config_.controller_dt);
}

void Engine::commit_new_nominal(const std::map<std::string, Vec3>& new_nominal,
                                double applied_height) {
  for (const auto& n : LEG_NAMES) {
    nominal_[n] = new_nominal.at(n);
    legs_[n].nominal_stance = nominal_[n];
  }
  pause_ = build_pause();
  engagement_ = build_engagement();
  applied_height_ = applied_height;
}

bool Engine::cmd_is_zero(std::pair<double, double> v_body_xy,
                         double omega_z) const {
  const double tol = config_.cmd_zero_tol;
  return std::abs(v_body_xy.first) < tol && std::abs(v_body_xy.second) < tol &&
         std::abs(omega_z) < tol;
}

std::map<std::string, LegOutput> Engine::emit_stand() const {
  std::map<std::string, LegOutput> out;
  for (const auto& n : LEG_NAMES) {
    out[n] = LegOutput{nominal_.at(n), 0.0, true};
  }
  return out;
}

std::map<std::string, LegOutput> Engine::emit_held() const {
  std::map<std::string, LegOutput> out;
  for (const auto& n : LEG_NAMES) {
    out[n] = LegOutput{last_targets_.at(n), 0.0, true};
  }
  return out;
}

std::map<std::string, LegOutput> Engine::update(
    double dt, std::pair<double, double> v_body_xy, double omega_z) {
  const bool cmd_zero = cmd_is_zero(v_body_xy, omega_z);
  if (cmd_zero) {
    cmd_zero_elapsed_ += dt;
  } else {
    cmd_zero_elapsed_ = 0.0;
  }
  const bool should_pause =
      cmd_zero && (cmd_zero_elapsed_ >= config_.pause_debounce_delay);
  height_stable_elapsed_ += dt;

  if (state_ == EngineState::FOLDED) {
    std::map<std::string, LegOutput> out;
    for (const auto& n : LEG_NAMES) {
      out[n] = LegOutput{initial_.at(n), 0.0, true};
    }
    return out;
  }

  if (state_ == EngineState::INITIALIZE) {
    auto out = initialize_->update(dt);
    capture_state(out);
    if (initialize_->done()) {
      state_ = EngineState::STAND;
      last_targets_ = nominal_;
      for (const auto& n : LEG_NAMES) last_stance_[n] = true;
    }
    return out;
  }

  if (state_ == EngineState::FOLDING) {
    auto out = fold_->update(dt);
    capture_state(out);
    if (fold_->done()) {
      state_ = EngineState::FOLDED;
      last_targets_ = initial_;
      for (const auto& n : LEG_NAMES) last_stance_[n] = true;
    }
    return out;
  }

  if (state_ == EngineState::STAND) {
    if (!cmd_zero) {
      // Walking takes priority over a pending reseat / fold.
      engagement_->begin(*strategy_, legs_);
      state_ = EngineState::ENGAGING;
      return tick_engagement(dt, v_body_xy, omega_z);
    }
    if (pending_fold_ &&
        std::abs(applied_height_) <= config_.reseat_height_change_threshold &&
        std::abs(target_height_) <= config_.reseat_height_change_threshold) {
      pending_fold_ = false;
      fold_ = build_fold();
      state_ = EngineState::FOLDING;
      return tick_fold(dt);
    }
    if (reseat_geometry_.has_value() && leg_specs_.has_value() &&
        std::abs(target_height_ - applied_height_) >
            config_.reseat_height_change_threshold &&
        height_stable_elapsed_ >= config_.reseat_pose_settle_delay) {
      std::map<std::string, Vec3> target_stance;
      try {
        target_stance = reseat_nominal_stance(target_height_, *reseat_geometry_,
                                              *leg_specs_);
      } catch (const std::invalid_argument&) {
        // Geometrically infeasible target — drop the reseat silently.
        return emit_stand();
      }
      reseat_ = build_reseat(target_stance);
      state_ = EngineState::RESEATING;
      reseat_target_stance_ = target_stance;
      reseat_target_height_ = target_height_;
      return tick_reseat(dt);
    }
    return emit_stand();
  }

  if (state_ == EngineState::RESEATING) {
    return tick_reseat(dt);
  }

  if (state_ == EngineState::ENGAGING) {
    if (cmd_zero) {
      enter_pausing();
      return tick_pause(dt);
    }
    auto out = tick_engagement(dt, v_body_xy, omega_z);
    if (engagement_->state() == EngagementState::DONE) {
      clock_->reset(engagement_->exit_master());
      stance_.seed(last_targets_, last_stance_);
      swing_.reset();
      state_ = EngineState::GAIT;
    }
    return out;
  }

  if (state_ == EngineState::GAIT) {
    if (pending_strategy_name_.has_value()) {
      enter_pausing();
      return tick_pause(dt);
    }
    if (should_pause) {
      enter_pausing();
      return tick_pause(dt);
    }
    return tick_gait(dt, v_body_xy, omega_z, cmd_zero);
  }

  if (state_ == EngineState::PAUSING) {
    if (!cmd_zero && !pending_strategy_name_.has_value()) {
      enter_resuming();
      return tick_engagement(dt, v_body_xy, omega_z);
    }
    auto out = tick_pause(dt);
    if (pause_->state() == PauseState::PAUSED) {
      state_ = EngineState::PAUSED;
      paused_elapsed_ = 0.0;
    }
    return out;
  }

  if (state_ == EngineState::PAUSED) {
    if (!cmd_zero && !pending_strategy_name_.has_value()) {
      enter_resuming();
      return tick_engagement(dt, v_body_xy, omega_z);
    }
    paused_elapsed_ += dt;
    const double dwell = pending_strategy_name_.has_value()
                             ? config_.gait_change_pause_to_reseat_delay
                             : config_.pause_to_reseat_delay;
    if (paused_elapsed_ >= dwell) {
      reseat_ = build_reseat(nominal_);
      reseat_target_stance_ = nominal_;
      reseat_target_height_ = applied_height_;
      state_ = EngineState::RESEATING;
      return tick_reseat(dt);
    }
    return emit_held();
  }

  // RESUMING.
  if (cmd_zero) {
    enter_pausing();
    return tick_pause(dt);
  }
  auto out = tick_engagement(dt, v_body_xy, omega_z);
  if (engagement_->state() == EngagementState::DONE) {
    clock_->reset(engagement_->exit_master());
    stance_.seed(last_targets_, last_stance_);
    state_ = EngineState::GAIT;
  }
  return out;
}

std::map<std::string, LegOutput> Engine::tick_gait(
    double dt, std::pair<double, double> v_body_xy, double omega_z,
    bool cmd_zero) {
  // Hold the previous tick's targets verbatim during the cmd-zero debounce.
  if (cmd_zero) {
    const auto phases = clock_->phases();
    std::map<std::string, LegOutput> out;
    for (const auto& n : LEG_NAMES) {
      out[n] = LegOutput{last_targets_[n], phases.at(n), last_stance_[n]};
    }
    return out;
  }

  const double duty_factor = strategy_->duty_factor();
  const double stride_length = config_.stride_length;
  const double swing_end = 1.0 - duty_factor;

  const auto leg_velocities =
      per_leg_planar_velocity(legs_, v_body_xy, omega_z);
  double max_leg_v = 0.0;
  for (const auto& [name, v] : leg_velocities) {
    (void)name;
    max_leg_v = std::max(max_leg_v, std::hypot(v.first, v.second));
  }

  const auto [min_cycle_time, max_cycle_time] =
      cycle_time_bounds(config_, duty_factor);
  const double cycle_time =
      derive_cycle_time(max_leg_v, config_.stride_length, duty_factor,
                        min_cycle_time, max_cycle_time);
  const double stance_time = cycle_time * duty_factor;
  const double swing_time = cycle_time * swing_end;

  clock_->advance(dt, cycle_time);
  const auto phases = clock_->phases();

  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    const LegContext& leg = legs_.at(name);
    const auto& v = leg_velocities.at(name);
    const double v_x = v.first;
    const double v_y = v.second;
    const Vec3 stride_vec =
        stride_vector(v_x, v_y, stance_time, stride_length);
    StrideParams stride;
    stride.stride_vector = stride_vec;
    stride.cycle_time = cycle_time;
    stride.duty_factor = duty_factor;
    stride.swing_clearance = config_.step_height;
    stride.swing_width = config_.swing_width;
    stride.controller_dt = config_.controller_dt;
    // Strategy is evaluated unconditionally; the result is consumed only as a
    // fallback for stance legs that have never lifted off under the planner.
    const Vec3 strategy_target =
        strategy_->foot_target(phases.at(name), stride, leg);
    const bool stance = phases.at(name) >= swing_end - kStanceSeamEpsilon;

    Vec3 target;
    if (stance) {
      Vec3 touchdown_anchor;
      if (swing_.is_swing(name)) {
        // Touchdown edge: adopt the latched swing target as the new anchor.
        touchdown_anchor = swing_.target(name);
        swing_.touchdown(name);
      } else {
        touchdown_anchor = strategy_target;
      }
      auto integrated =
          stance_.step(name, true, touchdown_anchor, {v_x, v_y}, dt);
      target = *integrated;  // in_stance=true always returns a position
    } else {
      if (!swing_.is_swing(name)) {
        // Lift-off edge: capture origin/target/velocity, held for the swing.
        const Vec3 nominal = nominal_[name];
        const Vec3 aep = live_aep(nominal, stride_vec);
        swing_.liftoff(name, last_targets_[name], aep, {v_x, v_y},
                       std::max(swing_time, 1.0e-9), identity_y_sign(nominal));
      }
      const double phase_in_swing =
          swing_end > 0.0 ? phases.at(name) / swing_end : 0.0;
      target = swing_.evaluate(name, phase_in_swing, config_.step_height,
                               config_.swing_width, config_.controller_dt);
      // Keep the stance integrator's per-leg flag in sync.
      stance_.step(name, false, target, {v_x, v_y}, dt);
    }

    out[name] = LegOutput{target, phases.at(name), stance};
  }

  capture_state(out);
  return out;
}

void Engine::enter_pausing() {
  for (const auto& n : LEG_NAMES) {
    last_swing_flags_[n] = !last_stance_[n];
  }
  pause_->begin(last_targets_, last_swing_flags_);
  stance_.reset();
  swing_.reset();
  state_ = EngineState::PAUSING;
}

void Engine::enter_resuming() {
  engagement_->begin_resume(*strategy_, legs_, last_targets_, last_swing_flags_,
                            clock_->master());
  state_ = EngineState::RESUMING;
}

std::map<std::string, LegOutput> Engine::tick_pause(double dt) {
  auto out = pause_->update(dt);
  capture_state(out);
  return out;
}

std::map<std::string, LegOutput> Engine::tick_reseat(double dt) {
  auto out = reseat_->update(dt);
  capture_state(out);
  if (reseat_->done()) {
    // Commit a pending gait change at the RESEATING -> STAND handoff.
    std::optional<std::string> pending = pending_strategy_name_;
    pending_strategy_name_.reset();
    if (pending.has_value() && *pending != strategy_name_) {
      apply_strategy(*pending);
    }
    commit_new_nominal(reseat_target_stance_, reseat_target_height_);
    state_ = EngineState::STAND;
    last_targets_ = nominal_;
    for (const auto& n : LEG_NAMES) last_stance_[n] = true;
  }
  return out;
}

std::map<std::string, LegOutput> Engine::tick_fold(double dt) {
  auto out = fold_->update(dt);
  capture_state(out);
  if (fold_->done()) {
    state_ = EngineState::FOLDED;
    last_targets_ = initial_;
    for (const auto& n : LEG_NAMES) last_stance_[n] = true;
  }
  return out;
}

std::map<std::string, LegOutput> Engine::tick_engagement(
    double dt, std::pair<double, double> v_body_xy, double omega_z) {
  auto out = engagement_->update(dt, v_body_xy, omega_z);
  capture_state(out);
  return out;
}

void Engine::capture_state(const std::map<std::string, LegOutput>& out) {
  for (const auto& n : LEG_NAMES) {
    last_targets_[n] = out.at(n).foot_target;
    last_stance_[n] = out.at(n).stance;
  }
}

// ───────────────────────────── YAML builders ───────────────────────────────

std::map<std::string, Vec3> nominal_stance_from_yaml(
    const std::string& geometry_yaml, const std::string& standing_pose_yaml) {
  const auto legs = kin::load_leg_specs(geometry_yaml);
  const JointAngles angles =
      kin::load_standing_pose(standing_pose_yaml, geometry_yaml);
  std::map<std::string, Vec3> out;
  for (const auto& n : LEG_NAMES) {
    out[n] = kin::leg_to_body(kin::forward_kinematics(angles, legs.at(n)),
                              legs.at(n));
  }
  return out;
}

ReseatGeometry reseat_geometry_from_yaml(
    const std::string& geometry_yaml, const std::string& standing_pose_yaml) {
  const auto legs = kin::load_leg_specs(geometry_yaml);
  const JointAngles angles =
      kin::load_standing_pose(standing_pose_yaml, geometry_yaml);
  return default_geometry_from_pose(angles, legs.at(LEG_NAMES[0]));
}

std::map<std::string, Vec3> initial_stance_from_yaml(
    const std::string& geometry_yaml) {
  const auto legs = kin::load_leg_specs(geometry_yaml);
  const auto angles_per_leg = kin::load_initial_pose(geometry_yaml);
  std::map<std::string, Vec3> out;
  for (const auto& n : LEG_NAMES) {
    out[n] = kin::leg_to_body(
        kin::forward_kinematics(angles_per_leg.at(n), legs.at(n)), legs.at(n));
  }
  return out;
}

std::map<std::string, LegContext> build_leg_contexts(
    const std::string& geometry_yaml, const std::string& standing_pose_yaml) {
  const auto legs = kin::load_leg_specs(geometry_yaml);
  const auto nominal =
      nominal_stance_from_yaml(geometry_yaml, standing_pose_yaml);
  std::map<std::string, LegContext> out;
  for (const auto& n : LEG_NAMES) {
    LegContext ctx;
    ctx.name = n;
    ctx.mount_xyz = legs.at(n).mount_xyz;
    ctx.mount_yaw = legs.at(n).mount_yaw;
    ctx.nominal_stance = nominal.at(n);
    out[n] = ctx;
  }
  return out;
}

std::string state_value(EngineState s) {
  switch (s) {
    case EngineState::FOLDED: return "folded";
    case EngineState::INITIALIZE: return "initialize";
    case EngineState::STAND: return "stand";
    case EngineState::ENGAGING: return "engaging";
    case EngineState::GAIT: return "gait";
    case EngineState::PAUSING: return "pausing";
    case EngineState::PAUSED: return "paused";
    case EngineState::RESUMING: return "resuming";
    case EngineState::FOLDING: return "folding";
    case EngineState::RESEATING: return "reseating";
  }
  return "unknown";
}

std::string state_name(EngineState s) {
  switch (s) {
    case EngineState::FOLDED: return "FOLDED";
    case EngineState::INITIALIZE: return "INITIALIZE";
    case EngineState::STAND: return "STAND";
    case EngineState::ENGAGING: return "ENGAGING";
    case EngineState::GAIT: return "GAIT";
    case EngineState::PAUSING: return "PAUSING";
    case EngineState::PAUSED: return "PAUSED";
    case EngineState::RESUMING: return "RESUMING";
    case EngineState::FOLDING: return "FOLDING";
    case EngineState::RESEATING: return "RESEATING";
  }
  return "UNKNOWN";
}

}  // namespace hexa_gait
