#include "gait/engine.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "config_generated.hpp"
#include "gait/gaits/registry.hpp"

namespace hexa::gait {

namespace {
constexpr float kHeightNoiseEpsilon = 1e-6f;
// Absorbs float noise on the touchdown seam where swing_end is not representable.
constexpr float kStanceSeamEpsilon = 1e-9f;

// How close to nominal a planted foot counts as re-planted. A foot that landed
// at a zero stride is on nominal exactly, so this only absorbs float noise.
constexpr float kSettledEpsilon = 1e-5f;

// Does this gait ever have every walking foot down at once? A leg lifts at
// master pymod(-offset, 1) and lands swing_end later, so the window exists
// exactly where two successive lift-offs are further apart than one swing.
// Parked legs are filtered out: they would invent gaps no foot walks.
bool has_all_down_window(const PhaseOffsets& offsets, float swing_end,
                         LegSet leg_set) {
  // Fixed-size and sorted in place: no heap allocation on the control tick.
  std::array<float, kNumLegs> starts{};
  std::size_t n = 0;
  for (const auto& [name, offset] : offsets.offsets()) {
    if (leg_is_parked(leg_set, name)) {
      continue;
    }
    starts[n++] = pymod(-offset, 1.0f);
  }
  if (n == 0) {
    return false;
  }
  std::sort(starts.begin(), starts.begin() + n);
  for (std::size_t i = 0; i < n; ++i) {
    const bool last = (i + 1 == n);
    const float gap = last ? (starts[0] + 1.0f - starts[n - 1])
                           : starts[i + 1] - starts[i];
    if (gap > swing_end) {
      return true;
    }
  }
  return false;
}

// Per-gait cycle-time bounds from the swing-time bounds: both ends scale by
// 1 / swing_end, so a leg gets its full airborne window whatever the gait.
std::pair<float, float> cycle_time_bounds(const EngineConfig& cfg,
                                          float swing_end) {
  if (swing_end <= 0.0f) {
    return {cfg.max_swing_time, cfg.max_swing_time};
  }
  const float scale = 1.0f / swing_end;
  return {cfg.min_swing_time * scale, cfg.max_swing_time * scale};
}

}  // namespace

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
                                           float dt, const StanceBand& bound) {
  if (!in_stance) {
    is_stance_[name] = false;
    return std::nullopt;
  }
  if (!is_stance_[name]) {
    // The live AEP is already inside the band (stride_vector's magnitude clamp),
    // so saturating it would only put a step back in at the seam.
    anchor_[name] = swing_target;
    is_stance_[name] = true;
    return anchor_[name];
  }
  Vec3& a = anchor_[name];
  const auto [d_x, d_y] =
      ease_outward(a[0] - bound.nominal[0], a[1] - bound.nominal[1],
                   -v_leg.first * dt, -v_leg.second * dt, bound.band,
                   bound.ceiling);
  a = Vec3(a[0] + d_x, a[1] + d_y, a[2]);
  return a;
}

void StanceIntegrator::reset() {
  for (const auto& n : LEG_NAMES) {
    is_stance_[n] = false;
  }
}

SwingPlanner::SwingPlanner() {
  for (const auto& n : LEG_NAMES) {
    origin_[n] = Vec3::Zero();
    target_[n] = Vec3::Zero();
    v_origin_[n] = {0.0f, 0.0f};
    v_target_[n] = {0.0f, 0.0f};
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
  v_origin_[name] = v_leg;
  v_target_[name] = v_leg;
  swing_time_[name] = swing_time;
  identity_y_sign_[name] = identity_y_sign_val;
  is_swing_[name] = true;
}

void SwingPlanner::retarget(const std::string& name, const Vec3& target,
                            std::pair<float, float> v_leg, float swing_time,
                            float phase_in_swing, float dt,
                            const SwingProfile& profile) {
  if (!is_swing_.at(name)) {
    return;
  }
  v_target_[name] = v_leg;
  swing_time_[name] = swing_time;

  Vec3& aimed = target_[name];
  if (phase_in_swing < profile.probe_start()) {
    aimed = target;
    return;
  }
  const Vec3 move = target - aimed;
  const float distance = std::hypot(move[0], move[1]);
  const float budget = profile.touchdown_velocity * dt;
  aimed = distance <= budget ? target : aimed + (budget / distance) * move;
}

void SwingPlanner::touchdown(const std::string& name) {
  is_swing_[name] = false;
}

Vec3 SwingPlanner::evaluate(const std::string& name, float phase_in_swing,
                            const SwingProfile& profile) const {
  // A planted foot's body-frame velocity is -v_leg, so handing that to both ends
  // makes the foot lift straight up in the world frame and set straight down
  // again, with no horizontal slip either way.
  const auto& v_in = v_origin_.at(name);
  const auto& v_out = v_target_.at(name);
  const Vec3 velocity_in(-v_in.first, -v_in.second, 0.0f);
  const Vec3 velocity_out(-v_out.first, -v_out.second, 0.0f);
  return swing_arc(phase_in_swing, origin_.at(name), target_.at(name),
                   identity_y_sign_.at(name), swing_time_.at(name), profile,
                   velocity_in, velocity_out);
}

void SwingPlanner::reset() {
  for (const auto& n : LEG_NAMES) {
    is_swing_[n] = false;
  }
}

Engine::Engine(EngineConfig config, std::unique_ptr<Strategy> strategy,
               std::string strategy_name,
               std::map<std::string, Vec3> nominal_stance,
               std::map<std::string, Vec3> folded_stance,
               std::map<std::string, Vec3> initialized_stance,
               float coxa_to_bottom, float foot_radius,
               std::map<std::string, LegContext> leg_contexts,
               std::optional<std::map<std::string, kin::LegSpec>> leg_specs,
               std::optional<ReseatGeometryByLeg> reseat_geometry,
               std::vector<PresetSetup> presets, std::size_t default_preset)
    : config_(config),
      strategy_(std::move(strategy)),
      strategy_name_(std::move(strategy_name)),
      coxa_to_bottom_(coxa_to_bottom),
      foot_radius_(foot_radius),
      legs_(std::move(leg_contexts)),
      leg_specs_(std::move(leg_specs)),
      reseat_geometry_(std::move(reseat_geometry)),
      presets_(std::move(presets)) {
  require_all_legs(nominal_stance, "nominal_stance");
  require_all_legs(folded_stance, "folded_stance");
  require_all_legs(initialized_stance, "initialized_stance");
  require_all_legs(legs_, "leg_contexts");
  if (leg_specs_.has_value() != reseat_geometry_.has_value()) {
    throw std::invalid_argument(
        "leg_specs and reseat_geometry must be supplied together");
  }
  if (presets_.empty()) {
    // No table: one unnamed hexapod preset from the caller's stance and config.
    PresetSetup only;
    only.leg_set = LegSet::HEXAPOD;
    only.nominal_stance = nominal_stance;
    if (reseat_geometry_.has_value()) {
      only.reseat_geometry = *reseat_geometry_;
    }
    only.stride_length = config_.stride_length;
    only.stride_length_radial = config_.stride_length_radial;
    only.min_swing_time = config_.min_swing_time;
    only.max_swing_time = config_.max_swing_time;
    only.step_height = config_.step_height;
    presets_.push_back(std::move(only));
    default_preset = 0;
  }
  if (default_preset >= presets_.size()) {
    throw std::invalid_argument("default_preset is out of range");
  }
  if (presets_[default_preset].leg_set != LegSet::HEXAPOD) {
    throw std::invalid_argument(
        "the default preset must stand on all six legs — the cold-start "
        "baseline is folded on six, and it is what FAULT recovers to");
  }
  {
    std::map<std::string, std::size_t> seen;
    for (std::size_t i = 0; i < presets_.size(); ++i) {
      if (!seen.emplace(presets_[i].id, i).second) {
        throw std::invalid_argument("duplicate preset id: " + presets_[i].id);
      }
      require_all_legs(presets_[i].nominal_stance,
                       "preset " + presets_[i].id + " nominal_stance");
      if (presets_[i].leg_set != LegSet::HEXAPOD &&
          !reseat_geometry_.has_value()) {
        throw std::invalid_argument(
            "the quadruped leg set needs its own reseat geometry: a height "
            "change on four feet re-solves that footprint, not the hexapod "
            "one");
      }
    }
  }
  if (strategy_->leg_set() != LegSet::HEXAPOD) {
    throw std::invalid_argument(
        "the engine must be constructed on a hexapod gait — the cold-start "
        "baseline is folded on all six, and it is what FAULT recovers to");
  }

  for (const auto& n : LEG_NAMES) {
    nominal_[n] = nominal_stance.at(n);
    folded_[n] = folded_stance.at(n);
    initialized_[n] = initialized_stance.at(n);
  }
  preset_ = default_preset;
  fallback_preset_ = default_preset;
  leg_set_ = presets_[preset_].leg_set;
  apply_preset_knobs();
  fallback_strategy_name_ = strategy_name_;
  refresh_active_legs();

  clock_.emplace(strategy_->phase_offsets());
  engagement_ = build_engagement();
  initialize_ = build_initialize();

  state_ = EngineState::FOLDED;
  last_targets_ = folded_;
  for (const auto& n : LEG_NAMES) {
    last_stance_[n] = true;
    held_down_[n] = false;
    on_schedule_[n] = true;
  }
}

void Engine::refresh_active_legs() {
  active_legs_.clear();
  for (const auto& [name, ctx] : legs_) {
    if (!is_parked(name)) {
      active_legs_[name] = ctx;
    }
  }
}

std::optional<std::size_t> Engine::preset_for_leg_set(LegSet set) const {
  for (std::size_t i = 0; i < presets_.size(); ++i) {
    if (presets_[i].leg_set == set) {
      return i;
    }
  }
  return std::nullopt;
}

void Engine::apply_preset_knobs() {
  const PresetSetup& p = presets_[preset_];
  config_.stride_length = p.stride_length;
  config_.stride_length_radial = p.stride_length_radial;
  config_.min_swing_time = p.min_swing_time;
  config_.max_swing_time = p.max_swing_time;
  config_.step_height = p.step_height;
}

std::map<std::string, Vec3> Engine::stance_for(std::size_t preset) const {
  const std::map<std::string, Vec3>& base = presets_[preset].nominal_stance;
  // Without the reseat geometry there is nothing to re-solve against, so the
  // zero-offset base stands.
  if (!reseat_geometry_.has_value() || !leg_specs_.has_value() ||
      applied_height_ == 0.0f) {
    return base;
  }
  try {
    // The geometry is the target preset's, not the applied one's: this is
    // asking where that preset stands at the current height.
    return reseat_nominal_stance(applied_height_, geometry_for(preset),
                                 *leg_specs_, base);
  } catch (const std::invalid_argument&) {
    return base;
  }
}

void Engine::overlay_parked(std::map<std::string, LegOutput>& out) const {
  if (leg_set_ != LegSet::QUADRUPED) {
    return;
  }
  // Parked is the folded pose — the belly-rest angles the pair powers up in.
  for (const auto& n : PARKED_LEGS) {
    out[n] = LegOutput{folded_.at(n), 0.0f, false, true};
  }
}

float Engine::master_phase() const {
  if (state_ == EngineState::ENGAGING) {
    return engagement_->exit_master();
  }
  return clock_->master();
}

std::tuple<float, float, float> Engine::shape_reversal(
    float dt, std::pair<float, float> v_body_xy, float omega_z) {
  const float swing_end =
      swing_end_phase(strategy_->duty_factor(), swing_margin());
  const float stance_fraction = 1.0f - swing_end;

  ReversalGate::Input in;
  in.applied_xy = applied_xy_;
  in.applied_omega = applied_omega_;
  in.request_xy = v_body_xy;
  in.request_omega = omega_z;
  // The engagement is a walk too: it re-plans off the live command every tick, so
  // the ladder can hold it at the knee. What it cannot do there is reflect.
  in.walking =
      state_ == EngineState::GAIT || state_ == EngineState::ENGAGING;
  in.engaging = state_ == EngineState::ENGAGING;
  // Left honest: quadruped SHIFTING stands on all four with nothing moved yet,
  // and it is `engaging` that must stop the gate firing there.
  in.all_planted = all_planted();
  // GAIT only: inside the engagement the answer is the engagement's to give, and
  // it gives it once, at the handoff.
  in.feet_on_schedule = state_ == EngineState::GAIT && feet_on_schedule();
  in.can_mirror = clock_.has_value() &&
                  has_all_down_window(clock_->offsets(), swing_end, leg_set_);
  // The knee, read off the stride the *held* travel lays down: on a direction the
  // radial budget has cut, the isotropic knee sits above that direction's own
  // velocity cap and every reversal would read as already below it.
  const float knee_stride = effective_stride_length(
      active_legs_, applied_xy_, applied_omega_, config_.stride_length,
      config_.stride_length_radial);
  in.knee_speed = stance_fraction > 0.0f && config_.max_swing_time > 0.0f
                      ? knee_stride * swing_end /
                            (config_.max_swing_time * stance_fraction)
                      : 0.0f;
  // Two cycles at the slowest the gait runs: a window cannot be missed, and a
  // gait that never offers one does not sit on the stick.
  in.timeout = swing_end > 0.0f ? 2.0f * config_.max_swing_time / swing_end
                                : 0.0f;
  in.zero_tol = config_.cmd_zero_tol;
  in.dt = dt;

  const ReversalGate::Output out = reversal_.step(active_legs_, in);
  // GAIT rather than in.walking, which now spans the engagement: this reflects
  // clock_, which the engagement does not run.
  if (out.mirror && state_ == EngineState::GAIT && in.all_planted) {
    clock_->mirror(swing_end);
  }
  return {out.v_xy.first, out.v_xy.second, out.omega};
}

void Engine::apply_strategy(const std::string& name) {
  strategy_ = strategies().at(name)();
  strategy_name_ = name;
  // What FAULT recovery reverts to, so the folded baseline is never paired with
  // a strategy that walks four legs.
  if (strategy_->leg_set() == LegSet::HEXAPOD) {
    fallback_strategy_name_ = name;
  }
  clock_.emplace(strategy_->phase_offsets());
  engagement_ = build_engagement();
}

bool Engine::set_strategy(const std::string& name) {
  auto factory = strategies().find(name);
  if (factory == strategies().end()) {
    return false;
  }
  const LegSet want = factory->second()->leg_set();
  // The preset owns the leg set, so a gait that walks a different one is refused
  // rather than read as a request to change it. Measured against the PENDING
  // preset when a change is armed: both arrive in the same tick, preset first.
  const LegSet target =
      pending_preset_.has_value() ? presets_[*pending_preset_].leg_set : leg_set_;
  if (want != target) {
    return false;
  }
  // From the belly nothing is standing yet, so the switch is immediate.
  if (state_ == EngineState::FOLDED || state_ == EngineState::FAULT) {
    if (name != strategy_name_) {
      apply_strategy(name);
    }
    return true;
  }
  // Hand the strategy to the armed change's commit, so the set and the gait that
  // walks it move together.
  if (pending_preset_.has_value()) {
    pending_strategy_name_ = name;
    return true;
  }
  if (state_ == EngineState::STAND) {
    if (name != strategy_name_) {
      apply_strategy(name);
    }
    return true;
  }
  if (state_ == EngineState::GAIT || state_ == EngineState::SETTLING ||
      state_ == EngineState::RESEATING) {
    if (!pending_strategy_name_.has_value() && name == strategy_name_) {
      return true;
    }
    pending_strategy_name_ = name;
    return true;
  }
  return false;
}

std::string Engine::default_strategy_for(LegSet set) const {
  // Only reached when a /cmd_preset arrives with no /cmd_gait behind it. Registry
  // order is alphabetical and stable, and a stable gait is preferred so the
  // accidental pairing is never the risky one.
  std::string fallback;
  for (const auto& [name, factory] : strategies()) {
    auto strategy = factory();
    if (strategy->leg_set() != set) {
      continue;
    }
    if (!strategy->unstable()) {
      return name;
    }
    if (fallback.empty()) {
      fallback = name;
    }
  }
  if (fallback.empty()) {
    throw std::invalid_argument("no strategy walks the requested leg set");
  }
  return fallback;
}

bool Engine::request_leg_set(LegSet set) {
  if (set == LegSet::HEXAPOD) {
    return request_preset(presets_[fallback_preset_].id);
  }
  const auto want = preset_for_leg_set(set);
  if (!want.has_value()) {
    return false;
  }
  return request_preset(presets_[*want].id);
}

bool Engine::request_preset(const std::string& id) {
  std::optional<std::size_t> want;
  for (std::size_t i = 0; i < presets_.size(); ++i) {
    if (presets_[i].id == id) {
      want = i;
      break;
    }
  }
  if (!want.has_value()) {
    return false;
  }
  // Already there, and nothing armed: idempotent, so a latched topic re-read on
  // every tick costs nothing.
  if (*want == preset_ && !pending_preset_.has_value()) {
    return true;
  }
  // From the belly the choice is still open — this is what start_initialize()
  // reads to decide which stance it climbs to, and which ladder it climbs.
  if (state_ == EngineState::FOLDED || state_ == EngineState::FAULT) {
    pending_preset_.reset();
    if (*want != preset_) {
      apply_preset(*want);
      // A legal pairing is all this owes; what the operator wants arrives on
      // the next latched /cmd_gait.
      if (strategy_->leg_set() != leg_set_) {
        apply_strategy(default_strategy_for(leg_set_));
      }
    }
    return true;
  }
  // Off the belly a preset change re-plants every foot, so it is only taken from
  // a stand, which is what makes "refused while walking" total. The ladder itself
  // is armed in update()'s STAND branch; this only latches the request.
  if (state_ != EngineState::STAND) {
    return false;
  }
  if (!leg_specs_.has_value() || !reseat_geometry_.has_value()) {
    // No geometry to reseat through, so there is no ladder to run.
    return false;
  }
  if (*want == preset_) {
    // Cancelling a change that has not started yet.
    pending_preset_.reset();
    return true;
  }
  pending_preset_ = *want;
  if (strategy_->leg_set() != presets_[*want].leg_set) {
    pending_strategy_name_ = default_strategy_for(presets_[*want].leg_set);
  }
  return true;
}

bool Engine::start_initialize() {
  // FAULT recovery reuses the cold start exactly.
  if (state_ != EngineState::FOLDED && state_ != EngineState::FAULT) {
    return false;
  }
  // The preset is already chosen — request_preset applies it outright from the
  // belly — so this only commits the stance that goes with it.
  apply_preset(preset_);
  initialize_ = build_initialize();
  state_ = EngineState::INITIALIZE;
  return true;
}

void Engine::set_leg_set(std::size_t preset) {
  if (preset == preset_) {
    return;
  }
  preset_ = preset;
  leg_set_ = presets_[preset].leg_set;
  if (leg_set_ == LegSet::HEXAPOD) {
    fallback_preset_ = preset;
  }
  // All three read the preset in force.
  apply_preset_knobs();
  refresh_active_legs();
  engagement_ = build_engagement();
}

void Engine::apply_preset(std::size_t preset) {
  if (preset == preset_) {
    return;
  }
  preset_ = preset;
  leg_set_ = presets_[preset].leg_set;
  if (leg_set_ == LegSet::HEXAPOD) {
    fallback_preset_ = preset;
  }
  apply_preset_knobs();
  commit_new_nominal(stance_for(preset), applied_height_);
}

void Engine::enter_fault() {
  if (state_ == EngineState::FAULT) {
    return;  // the pipeline may assert the fault every tick
  }
  // The constructor's FOLDED baseline, so start_initialize() runs the ladder from
  // a pristine folded state.
  state_ = EngineState::FAULT;
  last_targets_ = folded_;
  for (const auto& n : LEG_NAMES) {
    last_stance_[n] = true;
    held_down_[n] = false;
    on_schedule_[n] = true;
  }
  cmd_zero_elapsed_ = 0.0f;
  cmd_gain_ = 1.0f;
  reversal_.reset();
  pending_fold_ = false;
  pending_strategy_name_.reset();
  // Recovery is the cold start, on six legs; an armed change means nothing here.
  pending_preset_.reset();
  // The folded baseline is a six-leg pose, so the preset and the strategy come
  // back with it — start_initialize() stands on the applied preset, and a
  // four-leg one would stand the robot up on a pair still lying under the
  // chassis. It reverts to the LAST six-leg preset applied, not the boot one.
  if (leg_set_ != LegSet::HEXAPOD) {
    applied_height_ = 0.0f;
    apply_preset(fallback_preset_);
  }
  if (strategy_->leg_set() != LegSet::HEXAPOD) {
    apply_strategy(fallback_strategy_name_);
  }
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
      leg_set_, folded_, initialized_, nominal_, coxa_to_bottom_, foot_radius_,
      config_.init_pair_swing_time, config_.init_lift_body_time,
      config_.init_unfold_time, config_.init_place_clearance,
      config_.init_swing_clearance, config_.swing_width,
      config_.touchdown_velocity, config_.touchdown_probe_fraction,
      config_.controller_dt);
}

std::unique_ptr<FoldController> Engine::build_fold() {
  // The tuck is the unfold run backwards, so it takes the same time.
  return std::make_unique<FoldController>(
      leg_set_, folded_, initialized_, nominal_, coxa_to_bottom_, foot_radius_,
      config_.init_pair_swing_time, config_.init_lift_body_time,
      config_.init_unfold_time, config_.init_swing_clearance,
      config_.swing_width, config_.touchdown_velocity,
      config_.touchdown_probe_fraction, config_.controller_dt);
}

std::unique_ptr<EngagementController> Engine::build_engagement() {
  const float swing_end =
      swing_end_phase(strategy_->duty_factor(), swing_margin());
  const auto [min_cycle_time, max_cycle_time] =
      cycle_time_bounds(config_, swing_end);
  return std::make_unique<EngagementController>(
      nominal_, config_.stride_length, config_.stride_length_radial,
      min_cycle_time, max_cycle_time, strategy_->duty_factor(),
      swing_margin(), config_.swing_profile(),
      config_.controller_dt, ladder_shift_time());
}

std::unique_ptr<ReseatController> Engine::build_reseat(
    const std::map<std::string, Vec3>& target_stance) {
  // last_targets_ is where the feet are. The rung order carries the leg set: a
  // parked middle is not the ladder's to move.
  return std::make_unique<ReseatController>(
      last_targets_, target_stance, config_.reseat_pair_swing_time,
      config_.reseat_pair_dwell_time, config_.reseat_profile(),
      config_.controller_dt, reseat_rungs(leg_set_), ladder_shift_time(),
      config_.support_shift_lead);
}

void Engine::begin_reseat(const std::map<std::string, Vec3>& target_stance,
                          float target_height) {
  reseat_ = build_reseat(target_stance);
  reseat_target_stance_ = target_stance;
  reseat_target_height_ = target_height;
  plane_target_.reset();
  plane_ramping_ = false;
  state_ = EngineState::RESEATING;
}

std::map<std::string, Vec3> Engine::lateral_stance_for(
    std::size_t preset) const {
  const std::map<std::string, Vec3> want = stance_for(preset);
  std::map<std::string, Vec3> out;
  for (const auto& n : LEG_NAMES) {
    out[n] = Vec3(want.at(n).x, want.at(n).y, nominal_.at(n).z);
  }
  return out;
}

void Engine::begin_preset_reseat(std::size_t preset) {
  const std::map<std::string, Vec3> want = stance_for(preset);
  const std::map<std::string, Vec3> lateral = lateral_stance_for(preset);
  begin_reseat(lateral, applied_height_);
  // Same body height, no plane move. The threshold is the height reseat's, so
  // "the same height" means the same thing to both.
  for (const auto& n : LEG_NAMES) {
    if (std::fabs(want.at(n).z - lateral.at(n).z) >
        config_.reseat_height_change_threshold) {
      plane_target_ = want;
      return;
    }
  }
}

std::unique_ptr<PairFoldController> Engine::build_pair_fold(
    PairFoldDirection direction) {
  if (leg_set_ != LegSet::HEXAPOD) {
    // Reaching here on the quadruped set would mean the reseat ahead of it ran
    // the four-corner ladder, which the ordering exists to prevent.
    throw std::invalid_argument("pair fold requires the hexapod leg set");
  }
  return std::make_unique<PairFoldController>(
      direction, last_targets_, folded_, nominal_,
      config_.pair_fold_swing_time, config_.pair_fold_dwell_time,
      config_.pair_fold_probe_band(), config_.pair_fold_profile(),
      config_.controller_dt);
}

void Engine::commit_preset_change() {
  if (!pending_preset_.has_value()) {
    return;
  }
  // The preset first: apply_strategy rebuilds the engagement, which prices itself
  // against whichever preset is in force when it runs.
  set_leg_set(*pending_preset_);
  pending_preset_.reset();
  if (pending_strategy_name_.has_value()) {
    apply_strategy(*pending_strategy_name_);
    pending_strategy_name_.reset();
  }
}

void Engine::commit_new_nominal(const std::map<std::string, Vec3>& new_nominal,
                                float applied_height) {
  for (const auto& n : LEG_NAMES) {
    nominal_[n] = new_nominal.at(n);
    legs_[n].nominal_stance = nominal_[n];
  }
  refresh_active_legs();
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
  overlay_parked(out);
  return out;
}

std::map<std::string, LegOutput> Engine::emit_held() const {
  std::map<std::string, LegOutput> out;
  for (const auto& n : LEG_NAMES) {
    out[n] = LegOutput{last_targets_.at(n), 0.0f, true};
  }
  overlay_parked(out);
  return out;
}

std::map<std::string, LegOutput> Engine::update(
    float dt, std::pair<float, float> v_body_xy, float omega_z) {
  applied_xy_ = v_body_xy;
  applied_omega_ = omega_z;
  // cmd_zero_tol reads operator intent, where the ladder's hold means the
  // opposite: the engine asking to keep walking. Reading that hold as a release
  // would decay cmd_gain_ and speed the clock, which the reflection forbids.
  //
  // reversing(), not armed(): the limiter slews the planar command through the
  // origin, so every sign flip spends a tenth of a second inside the tolerance —
  // and ENGAGING re-plants on the first such tick.
  const bool cmd_zero =
      cmd_is_zero(v_body_xy, omega_z) && !reversal_.reversing();
  if (cmd_zero) {
    cmd_zero_elapsed_ += dt;
  } else {
    cmd_zero_elapsed_ = 0.0f;
  }
  // One debounce from full to nothing, so a released stick reaches zero exactly
  // as the debounce expires and the settle arms onto a still command.
  const bool wants_still =
      cmd_zero || pending_strategy_name_.has_value() ||
      state_ == EngineState::SETTLING;
  const float gain_rate = config_.settle_debounce_delay > 0.0f
                              ? dt / config_.settle_debounce_delay
                              : 1.0f;
  cmd_gain_ = wants_still ? std::max(0.0f, cmd_gain_ - gain_rate)
                          : std::min(1.0f, cmd_gain_ + gain_rate);
  const bool should_settle = wants_still && cmd_gain_ <= 0.0f;
  height_stable_elapsed_ += dt;

  if (state_ == EngineState::FOLDED || state_ == EngineState::FAULT) {
    // FAULT holds the folded baseline like FOLDED; servos are limp on the real
    // board. Recovery is start_initialize(), routed from pipeline::tick.
    std::map<std::string, LegOutput> out;
    for (const auto& n : LEG_NAMES) {
      out[n] = LegOutput{folded_.at(n), 0.0f, true};
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
    return tick_fold(dt);
  }

  if (state_ == EngineState::STAND) {
    if (!cmd_zero) {
      // Walking takes priority over a pending reseat / fold, and drops an armed
      // preset change outright: banking it would turn "refused while walking"
      // into "deferred until you stop".
      pending_preset_.reset();
      pending_strategy_name_.reset();
      engagement_->begin(*strategy_, active_legs_);
      state_ = EngineState::ENGAGING;
      return tick_engagement(dt, v_body_xy, omega_z);
    }
    if (pending_fold_ &&
        std::fabs(applied_height_) <= config_.reseat_height_change_threshold &&
        std::fabs(target_height_) <= config_.reseat_height_change_threshold) {
      pending_fold_ = false;
      // The fold is where the operator wants to be; it reaches a leg-set-
      // neutral FOLDED on its own, so an unstarted change is just dropped.
      pending_preset_.reset();
      pending_strategy_name_.reset();
      fold_ = build_fold();
      state_ = EngineState::FOLDING;
      return tick_fold(dt);
    }
    // A preset change, ahead of the height reseat: both want the ladder. Where
    // the leg sets differ the middle pair moves too, ordered by which way it is
    // going so the reseat always runs on six planted feet.
    if (pending_preset_.has_value() && *pending_preset_ != preset_) {
      const LegSet want = presets_[*pending_preset_].leg_set;
      if (want == leg_set_) {
        // Same legs, new footprint: the ladder and, where the two presets stand
        // at different heights, the plane move behind it. Committed on arrival.
        begin_preset_reseat(*pending_preset_);
        return tick_reseat(dt);
      }
      if (want == LegSet::QUADRUPED) {
        // Corners onto the four-corner footprint first, middle pair still down.
        // Both stances agree on the middles, so the ladder skips their rung.
        begin_preset_reseat(*pending_preset_);
        return tick_reseat(dt);
      }
      // The other way the pair comes down first, so the reseat that follows it
      // also runs on six. Commit the preset now: nominal_ still holds the quad
      // stance, whose middle entries are exactly where the pair is headed.
      commit_preset_change();
      pair_fold_ = build_pair_fold(PairFoldDirection::UNFOLD);
      state_ = EngineState::UNFOLDING_PAIR;
      return tick_pair_fold(dt);
    }
    if (reseat_geometry_.has_value() && leg_specs_.has_value() &&
        std::fabs(target_height_ - applied_height_) >
            config_.reseat_height_change_threshold &&
        height_stable_elapsed_ >= config_.reseat_pose_settle_delay) {
      std::map<std::string, Vec3> target_stance;
      try {
        target_stance = reseat_nominal_stance(
            target_height_, geometry_for(preset_), *leg_specs_, nominal_);
      } catch (const std::invalid_argument&) {
        // Infeasible target: one leg out of range aborts all six, or the body
        // would rest on a stance it was never solved for.
        return emit_stand();
      }
      begin_reseat(target_stance, target_height_);
      return tick_reseat(dt);
    }
    return emit_stand();
  }

  if (state_ == EngineState::RESEATING) {
    return tick_reseat(dt);
  }

  // Neither reads the command: a stick pushed while the pair is in the air is
  // ignored, and the walk engages from the stand on the far side.
  if (state_ == EngineState::FOLDING_PAIR ||
      state_ == EngineState::UNFOLDING_PAIR) {
    return tick_pair_fold(dt);
  }

  if (state_ == EngineState::ENGAGING) {
    // A command *withdrawn* mid-engagement goes straight to the reseat ladder:
    // the engagement cannot be run out at a zero command, where its swing branch
    // would teleport every leg home at its own lift-off. A command merely turned
    // around does not arrive here — the reversal gate latches it. No all_planted()
    // window to wait for: the ladder lands the airborne feet itself.
    if (cmd_zero) {
      hand_off_to_reseat();
      return tick_reseat(dt);
    }
    auto out = tick_engagement(dt, v_body_xy, omega_z);
    if (engagement_->state() == EngagementState::DONE) {
      // The engagement schedules off the strategy's own offset table, so a clock
      // left mirrored by an earlier reversal would hand every leg a phase out by
      // twice its offset. This walk starts from a stand.
      clock_->set_offsets(strategy_->phase_offsets());
      clock_->reset(engagement_->exit_master());
      stance_.seed(last_targets_, last_stance_);
      reset_swing_state();
      // What the engagement's velocity envelope left owing, the walk has to make
      // good before the reversal ladder may register anything against it.
      for (const auto& n : LEG_NAMES) {
        on_schedule_[n] = engagement_->foot_on_schedule(n);
      }
      state_ = EngineState::GAIT;
    }
    return out;
  }

  if (state_ == EngineState::GAIT) {
    // A pending gait change is folded into should_settle via cmd_gain_, so the
    // command eases out before the settle rather than dropping at walking speed.
    if (should_settle) {
      // Nothing to arm: the clock, the stance integrator and the swing planner
      // all carry straight on, which makes a settle abortable for free.
      state_ = EngineState::SETTLING;
      auto out = tick_gait(dt, {0.0f, 0.0f}, 0.0f, /*settling=*/true);
      // The debounce already ran the gait at a zero stride, so the feet may be
      // home the moment the settle arms.
      finish_or_hand_off_settle();
      return out;
    }
    return tick_gait(dt, v_body_xy, omega_z, /*settling=*/false);
  }

  if (state_ == EngineState::SETTLING) {
    // The clock never stopped and every foot rides a live AEP, so a command that
    // comes back just carries on walking — no engagement pass to re-enter.
    if (!cmd_zero && !pending_strategy_name_.has_value()) {
      state_ = EngineState::GAIT;
      return tick_gait(dt, v_body_xy, omega_z, /*settling=*/false);
    }
    auto out = tick_gait(dt, {0.0f, 0.0f}, 0.0f, /*settling=*/true);
    finish_or_hand_off_settle();
    return out;
  }

  // Only reachable if a state is added without a branch above.
  return emit_held();
}

std::map<std::string, LegOutput> Engine::tick_gait(
    float dt, std::pair<float, float> v_body_xy, float omega_z, bool settling) {
  // A zero command is not a special case: stride_vector collapses and
  // AEP == PEP == nominal, so the gait steps in place until the debounce arms the
  // settle. Holding the targets instead would freeze an airborne leg mid-arc. By
  // the time settling is reached cmd_gain_ has already eased the command out, so
  // the hard zero here only lands the AEP on nominal to the last bit.
  if (settling) {
    v_body_xy = {0.0f, 0.0f};
    omega_z = 0.0f;
  } else {
    v_body_xy = {v_body_xy.first * cmd_gain_, v_body_xy.second * cmd_gain_};
    omega_z *= cmd_gain_;
  }
  const float swing_end =
      swing_end_phase(strategy_->duty_factor(), swing_margin());
  const float stance_fraction = 1.0f - swing_end;

  // Parked legs are excluded from every velocity and stride computation: their
  // lever arms are the longest on the chassis, so counting them would overstate
  // the fastest foot and slow the whole walk.
  const auto leg_velocities =
      per_leg_planar_velocity(active_legs_, v_body_xy, omega_z);
  float max_leg_v = 0.0f;
  for (const auto& [name, v] : leg_velocities) {
    (void)name;
    max_leg_v = std::max(max_leg_v, std::hypot(v.first, v.second));
  }

  // The stride this tick can actually lay down; the worst-placed leg sets it for
  // all of them, since they share one stance_time. Everything downstream reads
  // this rather than config_.stride_length.
  const float stride_length =
      effective_stride_length(active_legs_, leg_velocities,
                              config_.stride_length,
                              config_.stride_length_radial);
  const SwingProfile swing_profile = config_.swing_profile(stride_length);

  const auto [min_cycle_time, max_cycle_time] =
      cycle_time_bounds(config_, swing_end);
  // Scaled by 1 / swing_end like the walk's, so every gait gets the same airborne
  // time out of one settle_swing_time. Faded in on cmd_gain_ so the clock never
  // changes rate under an airborne foot.
  const float settle_cycle_time =
      config_.settle_swing_time / std::max(swing_end, 1.0e-6f);
  const float walk_cycle_time =
      derive_cycle_time(max_leg_v, stride_length, stance_fraction,
                        min_cycle_time, max_cycle_time);
  const float cycle_time =
      settling ? settle_cycle_time
               : cmd_gain_ * walk_cycle_time +
                     (1.0f - cmd_gain_) * settle_cycle_time;
  const float stance_time = cycle_time * stance_fraction;
  const float swing_time = cycle_time * swing_end;

  // On the ladder's route the gait only lands what is already in the air, or a
  // gait whose swings overlap end to end would never give the ladder its
  // all-planted moment. A held leg does not move: the command is zero.
  const bool hold_liftoffs = settling && !settle_beats_reseat();

  // A steady walk rides exactly on `stance_band` at AEP and PEP, so the bound is
  // free until the command turns under a planted leg.
  const float stance_band = 0.5f * stride_length;
  const float stance_ceiling = stance_band * (1.0f + kStanceExcursionGrace);

  clock_->advance(dt, cycle_time);
  const auto phases = clock_->phases();

  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    if (is_parked(name)) {
      continue;  // overlaid below, out of the walk entirely
    }
    const LegContext& leg = legs_.at(name);
    const auto& v = leg_velocities.at(name);
    const float v_x = v.first;
    const float v_y = v.second;
    const Vec3 stride_vec = stride_vector(v_x, v_y, stance_time, stride_length);
    StrideParams stride;
    stride.stride_vector = stride_vec;
    stride.cycle_time = cycle_time;
    stride.swing_end = swing_end;
    stride.controller_dt = config_.controller_dt;
    stride.swing = swing_profile;
    // Strategy is evaluated unconditionally; the result is consumed only as a
    // fallback for stance legs that have never lifted off under the planner.
    const Vec3 strategy_target =
        strategy_->foot_target(phases.at(name), stride, leg);
    bool stance = phases.at(name) >= swing_end - kStanceSeamEpsilon;
    if (stance) {
      held_down_[name] = false;
    } else if (!swing_.is_swing(name) && (hold_liftoffs || held_down_[name])) {
      // Lift-off came due while lift-offs were held, so the leg sits the window
      // out. The flag outlives the settle deliberately: dropping it when the
      // command came back would start the arc part-way along and jump the foot.
      held_down_[name] = true;
      stance = true;
    }
    const Vec3 nominal = nominal_[name];
    // Read every tick, so a reseat's new stance is picked up on the next one.
    const StanceBand bound{nominal, stance_band, stance_ceiling};

    Vec3 target;
    if (stance) {
      Vec3 touchdown_anchor;
      if (swing_.is_swing(name)) {
        // Touchdown edge: the latched swing target becomes the anchor. At a zero
        // stride it is nominal, which is how a settle re-plants with no extra pass.
        touchdown_anchor = swing_.target(name);
        swing_.touchdown(name);
        // Landed on the live AEP with the integrator anchored there: whatever
        // the engagement left this leg owing, it is square with its phase now.
        on_schedule_[name] = true;
      } else {
        touchdown_anchor = strategy_target;
      }
      auto integrated =
          stance_.step(name, true, touchdown_anchor, {v_x, v_y}, dt, bound);
      target = *integrated;  // in_stance=true always returns a position
    } else {
      const Vec3 aep = live_aep(nominal, stride_vec);
      if (!swing_.is_swing(name)) {
        // Lift-off edge: capture the origin end, held for the whole swing.
        swing_.liftoff(name, last_targets_[name], aep, {v_x, v_y},
                       std::max(swing_time, 1.0e-9f), identity_y_sign(nominal));
      }
      const float phase_in_swing =
          swing_end > 0.0f ? phases.at(name) / swing_end : 0.0f;
      // Re-aimed every tick and bounded from the probe on; see
      // SwingPlanner::retarget.
      swing_.retarget(name, aep, {v_x, v_y}, std::max(swing_time, 1.0e-9f),
                      phase_in_swing, dt, swing_profile);
      target = swing_.evaluate(name, phase_in_swing, swing_profile);
      stance_.step(name, false, target, {v_x, v_y}, dt, bound);  // keeps its flag in sync
    }

    out[name] = LegOutput{target, phases.at(name), stance};
  }

  overlay_parked(out);
  capture_state(out);
  return out;
}

// Standing is a position, not a sequence: a leg that re-planted while the
// command ramped out does not have to do it again.
bool Engine::all_settled() const {
  for (const auto& n : LEG_NAMES) {
    if (is_parked(n)) {
      continue;
    }
    if (!last_stance_.at(n)) {
      return false;
    }
    if ((last_targets_.at(n) - nominal_.at(n)).norm() > kSettledEpsilon) {
      return false;
    }
  }
  return true;
}

bool Engine::all_planted() const {
  for (const auto& n : LEG_NAMES) {
    // A parked foot is never coming down, so testing its stance flag would
    // leave this false forever and no reversal could ever mirror.
    if (is_parked(n)) {
      continue;
    }
    if (!last_stance_.at(n)) {
      return false;
    }
  }
  return true;
}

bool Engine::feet_on_schedule() const {
  for (const auto& n : LEG_NAMES) {
    if (is_parked(n)) {
      continue;  // lays nothing down, so it can owe the schedule nothing
    }
    if (!on_schedule_.at(n)) {
      return false;
    }
  }
  return true;
}

bool Engine::settle_beats_reseat() const {
  const float swing_end =
      swing_end_phase(strategy_->duty_factor(), swing_margin());
  if (swing_end <= 0.0f) {
    return false;
  }
  // A gait that never has every walking foot down cannot finish a settle on its
  // own.
  if (!has_all_down_window(strategy_->phase_offsets(), swing_end, leg_set_)) {
    return false;
  }
  // Worst case for the gait: the last leg to get its turn is a whole cycle away.
  const float natural = config_.settle_swing_time / swing_end;
  // Worst case for the ladder: one swing waiting for the airborne legs to land,
  // then its rungs with a dwell between them, each preceded by whatever shift
  // hold the leg set owes.
  const float rungs = static_cast<float>(reseat_rungs(leg_set_).size());
  const float ladder = config_.settle_swing_time +
                       rungs * (config_.reseat_pair_swing_time +
                                ladder_shift_time()) +
                       (rungs - 1.0f) * config_.reseat_pair_dwell_time;
  return natural <= ladder;
}

void Engine::reset_swing_state() {
  swing_.reset();
  for (const auto& n : LEG_NAMES) {
    held_down_[n] = false;
  }
}

void Engine::finish_or_hand_off_settle() {
  if (all_settled()) {
    finish_settling();
    return;
  }
  if (settle_beats_reseat() || !all_planted()) {
    return;  // keep walking the feet home, or wait for the airborne one to land
  }
  // The gait needs a whole cycle to get round every leg. Hand what is still out
  // to the ladder, which lifts three mirrored pairs and skips the rest.
  hand_off_to_reseat();
}

void Engine::hand_off_to_reseat() {
  // The ladder starts from last_targets_, and its first stage lands anything
  // airborne before lifting a foot that is down — hence no all_planted() check.
  // The reversal latch suppresses cmd_zero, so it is torn down with the walk.
  reversal_.reset();
  stance_.reset();
  reset_swing_state();
  begin_reseat(nominal_, applied_height_);
}

void Engine::finish_settling() {
  // The new strategy's phase offsets are only meaningful from a standing start.
  std::optional<std::string> pending = pending_strategy_name_;
  pending_strategy_name_.reset();
  if (pending.has_value() && *pending != strategy_name_) {
    apply_strategy(*pending);
  }
  stance_.reset();
  reset_swing_state();
  state_ = EngineState::STAND;
  // A no-op by construction, stated so STAND never holds a stale target.
  last_targets_ = nominal_;
  for (const auto& n : LEG_NAMES) last_stance_[n] = true;
}

std::map<std::string, LegOutput> Engine::tick_reseat(float dt) {
  if (plane_ramping_) {
    return tick_plane_ramp(dt);
  }
  auto out = reseat_->update(dt);
  // The ladder reports only the legs its rung order owns.
  overlay_parked(out);
  capture_state(out);
  if (!reseat_->done()) {
    return out;
  }
  // The corners have arrived; the footprint they arrived on is now nominal.
  commit_new_nominal(reseat_target_stance_, reseat_target_height_);
  if (plane_target_.has_value()) {
    // Every foot on its final x-y and all six down: the calmest moment to move
    // the body, and ahead of the pair fold so it crosses on six feet, not four.
    plane_origin_ = nominal_;
    plane_elapsed_ = 0.0f;
    plane_ramping_ = true;
    return out;
  }
  return finish_reseat(std::move(out));
}

std::map<std::string, LegOutput> Engine::tick_plane_ramp(float dt) {
  plane_elapsed_ += dt;
  const float tau = config_.reseat_plane_ramp_time > 0.0f
                        ? plane_elapsed_ / config_.reseat_plane_ramp_time
                        : 1.0f;
  const float s = eased_ramp(tau);
  const std::map<std::string, Vec3>& want = *plane_target_;
  std::map<std::string, LegOutput> out;
  for (const auto& n : LEG_NAMES) {
    const Vec3& a = plane_origin_.at(n);
    out[n] = LegOutput{a + (want.at(n) - a) * s, 0.0f, true};
  }
  overlay_parked(out);
  capture_state(out);
  if (tau < 1.0f) {
    return out;
  }
  const std::map<std::string, Vec3> arrived = want;
  plane_target_.reset();
  plane_ramping_ = false;
  commit_new_nominal(arrived, applied_height_);
  return finish_reseat(std::move(out));
}

std::map<std::string, LegOutput> Engine::finish_reseat(
    std::map<std::string, LegOutput> out) {
  // A preset change still owing its pair fold: the pair goes up next rather than
  // the robot standing. The pending strategy is left alone — it names a gait of
  // the set the robot is not on yet, and commit_preset_change applies both.
  if (pending_preset_.has_value() &&
      presets_[*pending_preset_].leg_set != leg_set_) {
    pair_fold_ = build_pair_fold(PairFoldDirection::FOLD);
    state_ = EngineState::FOLDING_PAIR;
    return out;
  }
  // Either a same-leg-set change whose whole move has just arrived, or nothing
  // owing — a no-op, since the reseat after an unfold had its preset committed
  // before it started.
  commit_preset_change();
  // Commit a pending gait change at the RESEATING -> STAND handoff.
  std::optional<std::string> pending = pending_strategy_name_;
  pending_strategy_name_.reset();
  if (pending.has_value() && *pending != strategy_name_) {
    apply_strategy(*pending);
  }
  state_ = EngineState::STAND;
  last_targets_ = nominal_;
  for (const auto& n : LEG_NAMES) last_stance_[n] = true;
  return out;
}

std::map<std::string, LegOutput> Engine::tick_pair_fold(float dt) {
  auto out = pair_fold_->update(dt);
  // No overlay_parked: leg_set_ is HEXAPOD for the whole move, and the pair is
  // what this controller emits itself. capture_state is what makes last_targets_
  // honest for all six before the reseat that follows reads it.
  capture_state(out);
  if (!pair_fold_->done()) {
    return out;
  }
  if (pair_fold_->direction() == PairFoldDirection::FOLD) {
    // The one tick where leg_set_ and the pair's actual position change
    // together.
    commit_preset_change();
    state_ = EngineState::STAND;
    last_targets_ = nominal_;
    for (const auto& n : LEG_NAMES) last_stance_[n] = true;
  } else {
    // Down and planted on the middle nominal, which both stances agree on. The
    // corners are still out on the quadruped footprint; the reseat walks them in
    // and the plane ramp rides behind it, on all six.
    begin_preset_reseat(preset_);
  }
  return out;
}

std::map<std::string, LegOutput> Engine::tick_fold(float dt) {
  auto out = fold_->update(dt);
  capture_state(out);
  if (fold_->done()) {
    state_ = EngineState::FOLDED;
    last_targets_ = folded_;
    for (const auto& n : LEG_NAMES) last_stance_[n] = true;
    // Folded is one pose whichever set walked into it, so FOLDED is handed back
    // on the last six-leg preset applied. The next stand's preset is chosen from
    // the belly, where request_preset applies it outright.
    apply_preset(fallback_preset_);
  }
  return out;
}

std::map<std::string, LegOutput> Engine::tick_engagement(
    float dt, std::pair<float, float> v_body_xy, float omega_z) {
  auto out = engagement_->update(dt, v_body_xy, omega_z);
  overlay_parked(out);
  capture_state(out);
  return out;
}

void Engine::capture_state(const std::map<std::string, LegOutput>& out) {
  for (const auto& n : LEG_NAMES) {
    last_targets_[n] = out.at(n).foot_target;
    last_stance_[n] = out.at(n).stance;
  }
}

// Config builders.

EngineConfig engine_config_from_config() {
  const auto& c = ::hexa::config::kEngine;
  EngineConfig cfg;
  // The five preset-owned knobs, seeded from the default preset. The engine
  // rewrites them on every preset change.
  const auto& p = ::hexa::config::kPresets[::hexa::config::kDefaultPreset];
  cfg.stride_length = p.stride_length;
  cfg.stride_length_radial = p.stride_length_radial;
  cfg.min_swing_time = p.min_swing_time;
  cfg.max_swing_time = p.max_swing_time;
  cfg.step_height = p.step_height;
  cfg.swing_width = c.swing_width;
  cfg.touchdown_velocity = c.touchdown_velocity;
  cfg.touchdown_probe_fraction = c.touchdown_probe_fraction;
  cfg.swing_phase_margin = c.swing_phase_margin;
  cfg.quadruped_swing_phase_margin = c.quadruped_swing_phase_margin;
  cfg.quadruped_shift_time = c.quadruped_shift_time;
  cfg.support_shift_lead = ::hexa::config::kPosture.support_shift_lead;
  cfg.controller_dt = c.controller_dt;
  cfg.cmd_zero_tol = c.cmd_zero_tol;
  cfg.settle_debounce_delay = c.settle_debounce_delay;
  cfg.settle_swing_time = c.settle_swing_time;
  cfg.init_unfold_time = c.init_unfold_time;
  cfg.init_pair_swing_time = c.init_pair_swing_time;
  cfg.init_lift_body_time = c.init_lift_body_time;
  cfg.init_place_clearance = c.init_place_clearance;
  cfg.init_swing_clearance = c.init_swing_clearance;
  cfg.reseat_pose_settle_delay = c.reseat_pose_settle_delay;
  cfg.reseat_height_change_threshold = c.reseat_height_change_threshold;
  cfg.reseat_pair_swing_time = c.reseat_pair_swing_time;
  cfg.reseat_pair_dwell_time = c.reseat_pair_dwell_time;
  cfg.reseat_swing_clearance = c.reseat_swing_clearance;
  cfg.reseat_plane_ramp_time = c.reseat_plane_ramp_time;
  cfg.pair_fold_swing_time = c.pair_fold_swing_time;
  cfg.pair_fold_dwell_time = c.pair_fold_dwell_time;
  return cfg;
}

// So the no-arg builders can delegate to the parameterized versions.
namespace {
std::array<kin::LegSpec, kNumLegs> baked_leg_specs() {
  return ::hexa::config::kLegSpecs;
}
}  // namespace

namespace {

std::map<std::string, Vec3> stance_from(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::array<JointAngles, kNumLegs>& poses) {
  std::map<std::string, Vec3> out;
  for (int i = 0; i < 6; ++i) {
    const std::size_t idx = static_cast<std::size_t>(i);
    const auto& spec = specs[idx];
    out[LEG_NAMES[i]] =
        kin::leg_to_body(kin::forward_kinematics(poses[idx], spec), spec);
  }
  return out;
}

}  // namespace

namespace {

// The IK z a whole standing pose shares: the tip touches ground body_height
// below the body bottom, and IK aims at the tip sphere's centre, one radius
// above that contact point.
float standing_target_z(float coxa_to_bottom, float body_height,
                        float foot_radius) {
  return kin::ik_z_for_contact(-(coxa_to_bottom + body_height), foot_radius);
}

// One leg's standing triple, from its group's reach and splay. `pose_key` names
// the tuning.yaml block for the joint-limit message.
JointAngles standing_leg_from(const kin::LegSpec& spec, Leg leg,
                              const ::hexa::config::LegGroupStance& grp,
                              float target_z, const std::string& pose_key) {
  const LegGroup group = leg_group(leg);
  // The configured splay is the left leg's, positive outward, so it takes two
  // negations: rear legs face the other way, right legs mirror left ones. Keep
  // in step with gen_config.py's group_splay.
  const float sign = (group == LegGroup::REAR ? -1.0f : 1.0f) *
                     (leg_is_right(leg) ? -1.0f : 1.0f);
  const float th_c = sign * grp.coxa;

  // The radial reach is the group's tip_reach whatever the splay, so femur and
  // tibia come out uniform within a group. Throws UnreachableTarget if it cannot.
  const JointAngles out = kin::inverse_kinematics(
      Vec3(grp.tip_reach * std::cos(th_c), grp.tip_reach * std::sin(th_c),
           target_z),
      spec);

  for (std::size_t j = 0; j < 3; ++j) {
    const auto& lim = ::hexa::config::kJointLimits[j];
    if (out[j] < lim.lower || out[j] > lim.upper) {
      static constexpr std::array<const char*, 3> kJointNames = {
          "coxa", "femur", "tibia"};
      throw std::invalid_argument(
          "standing pose " + std::string(kJointNames[j]) + " for " +
          LEG_NAMES[static_cast<std::size_t>(leg)] + " is " +
          std::to_string(out[j]) + " rad, outside the joint limit window [" +
          std::to_string(lim.lower) + ", " + std::to_string(lim.upper) +
          "] rad — check tuning.yaml " + pose_key + ".standing_pose." +
          std::string(leg_group_name(group)));
    }
  }
  return out;
}

}  // namespace

std::array<JointAngles, kNumLegs> standing_pose_from(
    const std::array<kin::LegSpec, kNumLegs>& specs, float coxa_to_bottom,
    float foot_radius, const ::hexa::config::StandingPose& standing,
    const std::string& pose_key) {
  const float target_z =
      standing_target_z(coxa_to_bottom, standing.body_height, foot_radius);

  std::array<JointAngles, kNumLegs> out{};
  for (std::size_t i = 0; i < kNumLegs; ++i) {
    const Leg leg = static_cast<Leg>(i);
    const auto& grp = standing.groups[static_cast<std::size_t>(
        group_index(leg_group(leg)))];
    out[i] = standing_leg_from(specs[i], leg, grp, target_z, pose_key);
  }
  return out;
}

std::array<JointAngles, kNumLegs> parked_pair_standing_pose_from(
    const std::array<kin::LegSpec, kNumLegs>& specs, float coxa_to_bottom,
    float foot_radius, const ::hexa::config::StandingPose& standing,
    const std::array<JointAngles, kNumLegs>& middle_fallback,
    const std::string& pose_key) {
  const float target_z =
      standing_target_z(coxa_to_bottom, standing.body_height, foot_radius);

  std::array<JointAngles, kNumLegs> out{};
  for (std::size_t i = 0; i < kNumLegs; ++i) {
    const Leg leg = static_cast<Leg>(i);
    if (leg_group(leg) == LegGroup::MIDDLE) {
      out[i] = middle_fallback[i];
      continue;
    }
    const auto& grp = standing.groups[static_cast<std::size_t>(
        group_index(leg_group(leg)))];
    out[i] = standing_leg_from(specs[i], leg, grp, target_z, pose_key);
  }
  return out;
}

PresetSetup solve_preset(
    const PresetSpec& spec,
    const std::array<kin::LegSpec, kNumLegs>& specs, float coxa_to_bottom,
    float foot_radius,
    const std::array<JointAngles, kNumLegs>& middle_fallback) {
  const std::string key = "presets." + spec.id;
  const auto pose =
      spec.leg_set == LegSet::QUADRUPED
          ? parked_pair_standing_pose_from(specs, coxa_to_bottom, foot_radius,
                                           spec.standing, middle_fallback, key)
          : standing_pose_from(specs, coxa_to_bottom, foot_radius,
                               spec.standing, key);
  PresetSetup out;
  out.id = spec.id;
  out.leg_set = spec.leg_set;
  out.nominal_stance = nominal_stance_from(specs, pose);
  // Its own snapshot; see PresetSetup::reseat_geometry.
  out.reseat_geometry = reseat_geometry_from(specs, pose);
  out.stride_length = spec.stride_length;
  out.stride_length_radial = spec.stride_length_radial;
  out.min_swing_time = spec.min_swing_time;
  out.max_swing_time = spec.max_swing_time;
  out.step_height = spec.step_height;
  return out;
}

std::vector<PresetSetup> solve_presets(
    const std::vector<PresetSpec>& specs_in,
    const std::array<kin::LegSpec, kNumLegs>& specs, float coxa_to_bottom,
    float foot_radius, std::size_t default_preset) {
  if (specs_in.empty()) {
    throw std::invalid_argument("the preset table is empty");
  }
  if (default_preset >= specs_in.size()) {
    throw std::invalid_argument("default_preset is out of range");
  }
  // The middle rows a parked-pair preset borrows come from the default one,
  // which is the single preset guaranteed to stand on all six.
  const auto fallback = standing_pose_from(
      specs, coxa_to_bottom, foot_radius, specs_in[default_preset].standing,
      "presets." + specs_in[default_preset].id);
  std::vector<PresetSetup> out;
  out.reserve(specs_in.size());
  for (const auto& spec : specs_in) {
    out.push_back(
        solve_preset(spec, specs, coxa_to_bottom, foot_radius, fallback));
  }
  return out;
}

std::map<std::string, Vec3> nominal_stance_from(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::array<JointAngles, kNumLegs>& standing_pose) {
  return stance_from(specs, standing_pose);
}

std::map<std::string, Vec3> rest_stance_from(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::array<JointAngles, kNumLegs>& rest_pose) {
  return stance_from(specs, rest_pose);
}

std::map<std::string, kin::LegSpec> leg_specs_from(
    const std::array<kin::LegSpec, kNumLegs>& specs) {
  std::map<std::string, kin::LegSpec> out;
  for (int i = 0; i < 6; ++i) {
    out[LEG_NAMES[i]] = specs[static_cast<std::size_t>(i)];
  }
  return out;
}

ReseatGeometryByLeg reseat_geometry_from(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::array<JointAngles, kNumLegs>& standing_pose) {
  // One snapshot per leg: groups reach out different distances and stand with
  // different femur/tibia pairs.
  ReseatGeometryByLeg out{};
  for (std::size_t i = 0; i < static_cast<std::size_t>(kNumLegs); ++i) {
    out[i] = default_geometry_from_pose(standing_pose[i], specs[i]);
  }
  return out;
}

std::map<std::string, LegContext> build_leg_contexts_from(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::array<JointAngles, kNumLegs>& standing_pose) {
  const auto nominal = nominal_stance_from(specs, standing_pose);
  std::map<std::string, LegContext> out;
  for (int i = 0; i < 6; ++i) {
    const auto& spec = specs[static_cast<std::size_t>(i)];
    LegContext ctx;
    ctx.name = LEG_NAMES[i];
    ctx.mount_xyz = spec.mount_xyz;
    ctx.mount_yaw = spec.mount_yaw;
    ctx.nominal_stance = nominal.at(LEG_NAMES[i]);
    out[LEG_NAMES[i]] = ctx;
  }
  return out;
}

std::map<std::string, LegContext> leg_contexts_from_stance(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::map<std::string, Vec3>& nominal_stance) {
  std::map<std::string, LegContext> out;
  for (int i = 0; i < 6; ++i) {
    const auto& spec = specs[static_cast<std::size_t>(i)];
    LegContext ctx;
    ctx.name = LEG_NAMES[i];
    ctx.mount_xyz = spec.mount_xyz;
    ctx.mount_yaw = spec.mount_yaw;
    ctx.nominal_stance = nominal_stance.at(LEG_NAMES[i]);
    out[LEG_NAMES[i]] = ctx;
  }
  return out;
}

std::array<JointAngles, kNumLegs> standing_pose_from_config() {
  const auto& p = ::hexa::config::kPresets[::hexa::config::kDefaultPreset];
  return standing_pose_from(baked_leg_specs(), ::hexa::config::kCoxaToBottom,
                            ::hexa::config::kFootRadius, p.standing,
                            "presets." + std::string(p.id));
}

std::vector<PresetSpec> preset_specs_from_config() {
  std::vector<PresetSpec> out;
  out.reserve(::hexa::config::kPresets.size());
  for (const auto& p : ::hexa::config::kPresets) {
    PresetSpec spec;
    spec.id = std::string(p.id);
    spec.leg_set = p.leg_set;
    spec.standing = p.standing;
    spec.stride_length = p.stride_length;
    spec.stride_length_radial = p.stride_length_radial;
    spec.min_swing_time = p.min_swing_time;
    spec.max_swing_time = p.max_swing_time;
    spec.step_height = p.step_height;
    out.push_back(std::move(spec));
  }
  return out;
}

std::array<JointAngles, kNumLegs> preset_standing_pose_from_config(
    const std::string& id) {
  const auto specs = preset_specs_from_config();
  for (const auto& spec : specs) {
    if (spec.id != id) {
      continue;
    }
    const auto fallback = standing_pose_from_config();
    return spec.leg_set == LegSet::QUADRUPED
               ? parked_pair_standing_pose_from(
                     baked_leg_specs(), ::hexa::config::kCoxaToBottom,
                     ::hexa::config::kFootRadius, spec.standing, fallback,
                     "presets." + id)
               : standing_pose_from(baked_leg_specs(),
                                    ::hexa::config::kCoxaToBottom,
                                    ::hexa::config::kFootRadius, spec.standing,
                                    "presets." + id);
  }
  throw std::invalid_argument("unknown preset: " + id);
}

std::vector<PresetSetup> preset_setups_from_config() {
  return solve_presets(preset_specs_from_config(), baked_leg_specs(),
                       ::hexa::config::kCoxaToBottom,
                       ::hexa::config::kFootRadius,
                       ::hexa::config::kDefaultPreset);
}

std::map<std::string, Vec3> nominal_stance_from_config() {
  return nominal_stance_from(baked_leg_specs(), standing_pose_from_config());
}

std::map<std::string, Vec3> folded_stance_from_config() {
  return rest_stance_from(baked_leg_specs(), ::hexa::config::kFoldedPose);
}

std::map<std::string, Vec3> initialized_stance_from_config() {
  return rest_stance_from(baked_leg_specs(),
                          ::hexa::config::kInitializedPose);
}

std::map<std::string, kin::LegSpec> leg_specs_from_config() {
  return leg_specs_from(baked_leg_specs());
}

ReseatGeometryByLeg reseat_geometry_from_config() {
  return reseat_geometry_from(baked_leg_specs(), standing_pose_from_config());
}

std::map<std::string, LegContext> build_leg_contexts_from_config() {
  return build_leg_contexts_from(baked_leg_specs(), standing_pose_from_config());
}

std::unique_ptr<Engine> make_default_engine(
    const std::string& strategy_name,
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const EngineConfig& engine_cfg,
    const std::array<JointAngles, kNumLegs>& standing_pose,
    const std::array<JointAngles, kNumLegs>& folded_pose,
    const std::array<JointAngles, kNumLegs>& initialized_pose,
    float coxa_to_bottom, float foot_radius,
    std::vector<PresetSetup> presets, std::size_t default_preset) {
  auto factory = strategies().find(strategy_name);
  if (factory == strategies().end()) {
    throw std::invalid_argument("unknown strategy: " + strategy_name);
  }
  return std::make_unique<Engine>(
      engine_cfg, factory->second(), strategy_name,
      nominal_stance_from(specs, standing_pose),
      rest_stance_from(specs, folded_pose),
      rest_stance_from(specs, initialized_pose), coxa_to_bottom, foot_radius,
      build_leg_contexts_from(specs, standing_pose), leg_specs_from(specs),
      reseat_geometry_from(specs, standing_pose), std::move(presets),
      default_preset);
}

std::unique_ptr<Engine> make_default_engine(const std::string& strategy_name) {
  return make_default_engine(
      strategy_name, baked_leg_specs(), engine_config_from_config(),
      standing_pose_from_config(), ::hexa::config::kFoldedPose,
      ::hexa::config::kInitializedPose, ::hexa::config::kCoxaToBottom,
      ::hexa::config::kFootRadius, preset_setups_from_config(),
      ::hexa::config::kDefaultPreset);
}

std::string leg_set_value(LegSet set) {
  return set == LegSet::QUADRUPED ? "quadruped" : "hexapod";
}

std::string state_value(EngineState s) {
  switch (s) {
    case EngineState::FOLDED: return "folded";
    case EngineState::INITIALIZE: return "initialize";
    case EngineState::STAND: return "stand";
    case EngineState::ENGAGING: return "engaging";
    case EngineState::GAIT: return "gait";
    case EngineState::SETTLING: return "settling";
    case EngineState::FOLDING: return "folding";
    case EngineState::RESEATING: return "reseating";
    case EngineState::FOLDING_PAIR: return "folding_pair";
    case EngineState::UNFOLDING_PAIR: return "unfolding_pair";
    case EngineState::FAULT: return "fault";
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
    case EngineState::SETTLING: return "SETTLING";
    case EngineState::FOLDING: return "FOLDING";
    case EngineState::RESEATING: return "RESEATING";
    case EngineState::FOLDING_PAIR: return "FOLDING_PAIR";
    case EngineState::UNFOLDING_PAIR: return "UNFOLDING_PAIR";
    case EngineState::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

}  // namespace hexa::gait
