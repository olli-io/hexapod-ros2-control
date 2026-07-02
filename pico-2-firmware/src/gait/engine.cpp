#include "gait/engine.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "config_generated.hpp"  // hexa::config::k*
#include "gait/gaits/registry.hpp"

namespace hexa::gait {

namespace {
// Float-noise epsilon for "is the user still moving the D-pad?".
constexpr float kHeightNoiseEpsilon = 1e-6f;
// Inclusive tolerance for the swing->stance boundary (absorbs float noise on the
// touchdown seam at gaits where 1 - duty_factor is not representable).
constexpr float kStanceSeamEpsilon = 1e-9f;

// Per-gait cycle-time bounds derived from swing-phase bounds. Both ends scale by
// 1 / (1 - beta) so the swing-phase foot-velocity envelope is gait-agnostic.
std::pair<float, float> cycle_time_bounds(const EngineConfig& cfg, float beta) {
  if (beta >= 1.0f) {
    return {cfg.max_swing_time, cfg.max_swing_time};
  }
  const float scale = 1.0f / (1.0f - beta);
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
                                           std::pair<float, float> v_leg,
                                           float dt) {
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
    v_leg_[n] = {0.0f, 0.0f};
    swing_time_[n] = 0.0f;
    identity_y_sign_[n] = 1;
    is_swing_[n] = false;
  }
}

void SwingPlanner::liftoff(const std::string& name, const Vec3& origin,
                           const Vec3& target, std::pair<float, float> v_leg,
                           float swing_time, int identity_y_sign_val) {
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

Vec3 SwingPlanner::evaluate(const std::string& name, float phase_in_swing,
                            float swing_clearance, float swing_width,
                            float controller_dt) const {
  const auto& v = v_leg_.at(name);
  // Stance-frame foot velocity is -v_leg; pass it as both endpoints so the
  // Bezier's C1 nodes match the stance-frame velocity at lift-off and touchdown.
  const Vec3 v_match(-v.first, -v.second, 0.0f);
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
               std::map<std::string, Vec3> initial_stance, float coxa_to_bottom,
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

float Engine::master_phase() const {
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

void Engine::set_target_height(float target_height) {
  if (std::fabs(target_height - target_height_) > kHeightNoiseEpsilon) {
    height_stable_elapsed_ = 0.0f;
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
  const float beta = strategy_->duty_factor();
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
                                float applied_height) {
  for (const auto& n : LEG_NAMES) {
    nominal_[n] = new_nominal.at(n);
    legs_[n].nominal_stance = nominal_[n];
  }
  pause_ = build_pause();
  engagement_ = build_engagement();
  applied_height_ = applied_height;
}

bool Engine::cmd_is_zero(std::pair<float, float> v_body_xy,
                         float omega_z) const {
  const float tol = config_.cmd_zero_tol;
  return std::fabs(v_body_xy.first) < tol && std::fabs(v_body_xy.second) < tol &&
         std::fabs(omega_z) < tol;
}

std::map<std::string, LegOutput> Engine::emit_stand() const {
  std::map<std::string, LegOutput> out;
  for (const auto& n : LEG_NAMES) {
    out[n] = LegOutput{nominal_.at(n), 0.0f, true};
  }
  return out;
}

std::map<std::string, LegOutput> Engine::emit_held() const {
  std::map<std::string, LegOutput> out;
  for (const auto& n : LEG_NAMES) {
    out[n] = LegOutput{last_targets_.at(n), 0.0f, true};
  }
  return out;
}

std::map<std::string, LegOutput> Engine::update(
    float dt, std::pair<float, float> v_body_xy, float omega_z) {
  const bool cmd_zero = cmd_is_zero(v_body_xy, omega_z);
  if (cmd_zero) {
    cmd_zero_elapsed_ += dt;
  } else {
    cmd_zero_elapsed_ = 0.0f;
  }
  const bool should_pause =
      cmd_zero && (cmd_zero_elapsed_ >= config_.pause_debounce_delay);
  height_stable_elapsed_ += dt;

  if (state_ == EngineState::FOLDED) {
    std::map<std::string, LegOutput> out;
    for (const auto& n : LEG_NAMES) {
      out[n] = LegOutput{initial_.at(n), 0.0f, true};
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
        std::fabs(applied_height_) <= config_.reseat_height_change_threshold &&
        std::fabs(target_height_) <= config_.reseat_height_change_threshold) {
      pending_fold_ = false;
      fold_ = build_fold();
      state_ = EngineState::FOLDING;
      return tick_fold(dt);
    }
    if (reseat_geometry_.has_value() && leg_specs_.has_value() &&
        std::fabs(target_height_ - applied_height_) >
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
      paused_elapsed_ = 0.0f;
    }
    return out;
  }

  if (state_ == EngineState::PAUSED) {
    if (!cmd_zero && !pending_strategy_name_.has_value()) {
      enter_resuming();
      return tick_engagement(dt, v_body_xy, omega_z);
    }
    paused_elapsed_ += dt;
    const float dwell = pending_strategy_name_.has_value()
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
    float dt, std::pair<float, float> v_body_xy, float omega_z,
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

  const float duty_factor = strategy_->duty_factor();
  const float stride_length = config_.stride_length;
  const float swing_end = 1.0f - duty_factor;

  const auto leg_velocities = per_leg_planar_velocity(legs_, v_body_xy, omega_z);
  float max_leg_v = 0.0f;
  for (const auto& [name, v] : leg_velocities) {
    (void)name;
    max_leg_v = std::max(max_leg_v, std::hypot(v.first, v.second));
  }

  const auto [min_cycle_time, max_cycle_time] =
      cycle_time_bounds(config_, duty_factor);
  const float cycle_time =
      derive_cycle_time(max_leg_v, config_.stride_length, duty_factor,
                        min_cycle_time, max_cycle_time);
  const float stance_time = cycle_time * duty_factor;
  const float swing_time = cycle_time * swing_end;

  clock_->advance(dt, cycle_time);
  const auto phases = clock_->phases();

  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    const LegContext& leg = legs_.at(name);
    const auto& v = leg_velocities.at(name);
    const float v_x = v.first;
    const float v_y = v.second;
    const Vec3 stride_vec = stride_vector(v_x, v_y, stance_time, stride_length);
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
                       std::max(swing_time, 1.0e-9f), identity_y_sign(nominal));
      }
      const float phase_in_swing =
          swing_end > 0.0f ? phases.at(name) / swing_end : 0.0f;
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

std::map<std::string, LegOutput> Engine::tick_pause(float dt) {
  auto out = pause_->update(dt);
  capture_state(out);
  return out;
}

std::map<std::string, LegOutput> Engine::tick_reseat(float dt) {
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

std::map<std::string, LegOutput> Engine::tick_fold(float dt) {
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
    float dt, std::pair<float, float> v_body_xy, float omega_z) {
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

// ───────────────────────────── Config builders ──────────────────────────────

EngineConfig engine_config_from_config() {
  const auto& c = ::hexa::config::kEngine;
  EngineConfig cfg;
  cfg.stride_length = c.stride_length;
  cfg.min_swing_time = c.min_swing_time;
  cfg.max_swing_time = c.max_swing_time;
  cfg.step_height = c.step_height;
  cfg.swing_width = c.swing_width;
  cfg.controller_dt = c.controller_dt;
  cfg.cmd_zero_tol = c.cmd_zero_tol;
  cfg.pause_debounce_delay = c.pause_debounce_delay;
  cfg.pause_to_reseat_delay = c.pause_to_reseat_delay;
  cfg.gait_change_pause_to_reseat_delay = c.gait_change_pause_to_reseat_delay;
  cfg.max_reset_time = c.max_reset_time;
  cfg.init_pair_swing_time = c.init_pair_swing_time;
  cfg.init_lift_body_time = c.init_lift_body_time;
  cfg.init_swing_clearance = c.init_swing_clearance;
  cfg.init_place_feet_clearance = c.init_place_feet_clearance;
  cfg.reseat_pose_settle_delay = c.reseat_pose_settle_delay;
  cfg.reseat_height_change_threshold = c.reseat_height_change_threshold;
  cfg.reseat_pair_swing_time = c.reseat_pair_swing_time;
  cfg.reseat_pair_dwell_time = c.reseat_pair_dwell_time;
  cfg.reseat_swing_clearance = c.reseat_swing_clearance;
  return cfg;
}

std::map<std::string, Vec3> nominal_stance_from_config() {
  std::map<std::string, Vec3> out;
  for (int i = 0; i < 6; ++i) {
    const auto& spec = ::hexa::config::kLegSpecs[static_cast<std::size_t>(i)];
    out[LEG_NAMES[i]] = kin::leg_to_body(
        kin::forward_kinematics(::hexa::config::kStandingPose, spec), spec);
  }
  return out;
}

std::map<std::string, Vec3> initial_stance_from_config() {
  std::map<std::string, Vec3> out;
  for (int i = 0; i < 6; ++i) {
    const std::size_t idx = static_cast<std::size_t>(i);
    const auto& spec = ::hexa::config::kLegSpecs[idx];
    out[LEG_NAMES[i]] = kin::leg_to_body(
        kin::forward_kinematics(::hexa::config::kInitialPose[idx], spec), spec);
  }
  return out;
}

std::map<std::string, kin::LegSpec> leg_specs_from_config() {
  std::map<std::string, kin::LegSpec> out;
  for (int i = 0; i < 6; ++i) {
    out[LEG_NAMES[i]] = ::hexa::config::kLegSpecs[static_cast<std::size_t>(i)];
  }
  return out;
}

ReseatGeometry reseat_geometry_from_config() {
  return default_geometry_from_pose(::hexa::config::kStandingPose,
                                    ::hexa::config::kLegSpecs[0]);
}

std::map<std::string, LegContext> build_leg_contexts_from_config() {
  const auto nominal = nominal_stance_from_config();
  std::map<std::string, LegContext> out;
  for (int i = 0; i < 6; ++i) {
    const auto& spec = ::hexa::config::kLegSpecs[static_cast<std::size_t>(i)];
    LegContext ctx;
    ctx.name = LEG_NAMES[i];
    ctx.mount_xyz = spec.mount_xyz;
    ctx.mount_yaw = spec.mount_yaw;
    ctx.nominal_stance = nominal.at(LEG_NAMES[i]);
    out[LEG_NAMES[i]] = ctx;
  }
  return out;
}

std::unique_ptr<Engine> make_default_engine(const std::string& strategy_name) {
  auto factory = strategies().find(strategy_name);
  if (factory == strategies().end()) {
    throw std::invalid_argument("unknown strategy: " + strategy_name);
  }
  return std::make_unique<Engine>(
      engine_config_from_config(), factory->second(), strategy_name,
      nominal_stance_from_config(), initial_stance_from_config(),
      ::hexa::config::kCoxaToBottom, build_leg_contexts_from_config(),
      leg_specs_from_config(), reseat_geometry_from_config());
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

}  // namespace hexa::gait
