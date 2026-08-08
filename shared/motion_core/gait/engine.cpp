#include "gait/engine.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "config_generated.hpp"  // hexa::config::k*
#include "gait/gaits/registry.hpp"

namespace hexa::gait {

namespace {
// Float-noise epsilon for "is the user still moving the D-pad?".
constexpr float kHeightNoiseEpsilon = 1e-6f;
// Inclusive tolerance for the swing->stance boundary (absorbs float noise on the
// touchdown seam at gaits where the swing end is not representable).
constexpr float kStanceSeamEpsilon = 1e-9f;

// How close to its nominal stance a planted foot counts as re-planted. A foot
// that landed at a zero stride is on live_aep(nominal, 0) == nominal exactly,
// and the stance integrator then holds it there at zero velocity, so this only
// has to absorb float noise. Anything a real command left behind sits
// millimetres out, not microns.
constexpr float kSettledEpsilon = 1e-5f;

// Does this gait ever have all six feet on the ground at once? A leg lifts when
// its own phase wraps to 0 — at master pymod(-offset, 1) — and lands swing_end
// later, so a window with nothing in the air exists exactly where two successive
// lift-offs are further apart than one swing. A tripod has one at every
// handover; a crawl, whose swings run end to end, has none, and its feet can
// therefore never all be home at the same instant.
bool has_all_down_window(const PhaseOffsets& offsets, float swing_end) {
  // Fixed-size and sorted in place: this runs on the control tick, and the
  // RP2350 does not want a heap allocation there.
  std::array<float, kNumLegs> starts{};
  std::size_t n = 0;
  for (const auto& [name, offset] : offsets.offsets()) {
    (void)name;
    starts[n++] = pymod(-offset, 1.0f);
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

// Per-gait cycle-time bounds derived from swing-phase bounds. Both ends scale by
// 1 / swing_end so the swing-phase foot-velocity envelope is gait-agnostic, and
// so a leg still gets a full min_swing_time..max_swing_time in the air once the
// phase margin has shortened its window.
std::pair<float, float> cycle_time_bounds(const EngineConfig& cfg,
                                          float swing_end) {
  if (swing_end <= 0.0f) {
    return {cfg.max_swing_time, cfg.max_swing_time};
  }
  const float scale = 1.0f / swing_end;
  return {cfg.min_swing_time * scale, cfg.max_swing_time * scale};
}

// How far past its design band a stance anchor may drift before the integrator
// stops it, as a fraction of the band. The band is half a stride — exactly the
// AEP..PEP envelope a steady walk rides — so the grace is only what a leg
// borrows while the command turns under it.
//
// It is also the distance the foot has to shed its ground speed over, so it
// trades excursion against how hard the foot is braked. On this geometry a full
// lateral reversal peaks 14 mm past the band unbounded, and 0.25 gives up 8 mm
// of that for a braking step of ~8% of stance speed per tick — a tenth of what a
// hard clip would do, and well under what a swing already asks of the same leg.
constexpr float kStanceExcursionGrace = 0.25f;

// Ease the outward part of one stance step to zero as the anchor leaves its
// band. Inward and tangential motion is untouched, so a leg carried out recovers
// at full rate the moment the command turns — the bound is a wall, not a spring,
// and it never lags.
//
// The gain is 1 with zero slope at `band`, so ordinary walking — which rides
// exactly on the band at AEP and PEP — is untouched, and float noise at the
// boundary attenuates as O(eps^3) rather than stepping. It is 0 at `ceiling`,
// which the anchor therefore can never pass. The inward/outward test can only
// flip where the radial rate is already zero, so the foot target stays
// velocity-continuous across it.
//
// It shapes the *increment*, never the accumulated state: a soft-clip re-applied
// to the state every tick is not idempotent, and would creep the anchor inward
// even at zero velocity.
std::pair<float, float> ease_outward(float e_x, float e_y, float d_x, float d_y,
                                     float band, float ceiling) {
  const float m = std::hypot(e_x, e_y);
  if (m <= band || ceiling <= band) {
    return {d_x, d_y};
  }
  const float u_x = e_x / m;
  const float u_y = e_y / m;
  const float radial = d_x * u_x + d_y * u_y;
  if (radial <= 0.0f) {
    return {d_x, d_y};  // heading back toward nominal
  }
  const float gain =
      1.0f - ease5(std::min((m - band) / (ceiling - band), 1.0f));
  const float blocked = (1.0f - gain) * radial;
  return {d_x - blocked * u_x, d_y - blocked * u_y};
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
                                           float dt, const StanceBand& bound) {
  if (!in_stance) {
    is_stance_[name] = false;
    return std::nullopt;
  }
  if (!is_stance_[name]) {
    // The touchdown seed is the live AEP, which stride_vector's magnitude clamp
    // already keeps inside the band — saturating it would only put a step back
    // in at the seam.
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

// ─────────────────────────────── SwingPlanner ───────────────────────────────

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
                            std::pair<float, float> v_leg, float swing_time) {
  if (!is_swing_.at(name)) {
    return;
  }
  target_[name] = target;
  v_target_[name] = v_leg;
  swing_time_[name] = swing_time;
}

void SwingPlanner::touchdown(const std::string& name) {
  is_swing_[name] = false;
}

Vec3 SwingPlanner::evaluate(const std::string& name, float phase_in_swing,
                            const SwingProfile& profile) const {
  // The foot's body-frame velocity while planted is -v_leg, so handing that to
  // both ends of the swing makes it leave and meet the ground travelling with
  // the ground: the foot lifts straight up in the world frame and sets straight
  // down again, with no horizontal slip in either direction. The vertical
  // shaping at both ends comes from the profile.
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

// ─────────────────────────────────── Engine ─────────────────────────────────

Engine::Engine(EngineConfig config, std::unique_ptr<Strategy> strategy,
               std::string strategy_name,
               std::map<std::string, Vec3> nominal_stance,
               std::map<std::string, Vec3> initial_stance, float coxa_to_bottom,
               float foot_radius,
               std::map<std::string, LegContext> leg_contexts,
               std::optional<std::map<std::string, kin::LegSpec>> leg_specs,
               std::optional<ReseatGeometryByLeg> reseat_geometry)
    : config_(config),
      strategy_(std::move(strategy)),
      strategy_name_(std::move(strategy_name)),
      coxa_to_bottom_(coxa_to_bottom),
      foot_radius_(foot_radius),
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
  engagement_ = build_engagement();
  initialize_ = build_initialize();

  state_ = EngineState::FOLDED;
  last_targets_ = initial_;
  for (const auto& n : LEG_NAMES) {
    last_stance_[n] = true;
    held_down_[n] = false;
  }
}

float Engine::master_phase() const {
  if (state_ == EngineState::ENGAGING) {
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

bool Engine::start_initialize() {
  // Valid from a cold-start FOLDED or a latched FAULT — recovery reuses the exact
  // same INITIALIZE ladder (initial_ -> nominal_), so "press Start after a fault"
  // is byte-for-byte the startup + initialize path.
  if (state_ != EngineState::FOLDED && state_ != EngineState::FAULT) {
    return false;
  }
  initialize_ = build_initialize();
  state_ = EngineState::INITIALIZE;
  return true;
}

void Engine::enter_fault() {
  if (state_ == EngineState::FAULT) {
    return;  // idempotent — the pipeline may assert the fault every tick
  }
  // Reset to the same clean baseline the constructor establishes for FOLDED so a
  // subsequent start_initialize() runs the ladder from a pristine folded state.
  state_ = EngineState::FAULT;
  last_targets_ = initial_;
  for (const auto& n : LEG_NAMES) {
    last_stance_[n] = true;
    held_down_[n] = false;
  }
  cmd_zero_elapsed_ = 0.0f;
  cmd_gain_ = 1.0f;
  pending_fold_ = false;
  pending_strategy_name_.reset();
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
      initial_, nominal_, coxa_to_bottom_, foot_radius_,
      config_.init_pair_swing_time, config_.init_lift_body_time,
      config_.init_swing_clearance, config_.swing_width,
      config_.touchdown_velocity, config_.touchdown_probe_fraction,
      config_.controller_dt);
}

std::unique_ptr<FoldController> Engine::build_fold() {
  return std::make_unique<FoldController>(
      initial_, nominal_, coxa_to_bottom_, foot_radius_,
      config_.init_pair_swing_time, config_.init_lift_body_time,
      config_.init_swing_clearance, config_.swing_width,
      config_.touchdown_velocity, config_.touchdown_probe_fraction,
      config_.controller_dt);
}

std::unique_ptr<EngagementController> Engine::build_engagement() {
  const float swing_end =
      swing_end_phase(strategy_->duty_factor(), config_.swing_phase_margin);
  const auto [min_cycle_time, max_cycle_time] =
      cycle_time_bounds(config_, swing_end);
  return std::make_unique<EngagementController>(
      nominal_, config_.stride_length, min_cycle_time, max_cycle_time,
      strategy_->duty_factor(), config_.swing_phase_margin,
      config_.swing_profile(), config_.controller_dt);
}

std::unique_ptr<ReseatController> Engine::build_reseat(
    const std::map<std::string, Vec3>& target_stance) {
  // Always reseat from where the feet actually are (last_targets_ is rewritten
  // every tick).
  return std::make_unique<ReseatController>(
      last_targets_, target_stance, config_.reseat_pair_swing_time,
      config_.reseat_pair_dwell_time, config_.reseat_profile(),
      config_.controller_dt);
}

void Engine::commit_new_nominal(const std::map<std::string, Vec3>& new_nominal,
                                float applied_height) {
  for (const auto& n : LEG_NAMES) {
    nominal_[n] = new_nominal.at(n);
    legs_[n].nominal_stance = nominal_[n];
  }
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
  // Ease the command out rather than dropping it. The rate is one debounce from
  // full to nothing, so a released stick reaches zero exactly as the debounce
  // expires and the settle arms onto a command that is already still.
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
    // FAULT holds the folded baseline like FOLDED (servos are limp on the real
    // board; in sim the model settles to the recovery start pose). Recovery is
    // start_initialize(), routed from the FAULT branch of pipeline::tick.
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
                                              *leg_specs_, nominal_);
      } catch (const std::invalid_argument&) {
        // Geometrically infeasible target — drop the reseat silently. One leg
        // out of range aborts all six: a partial re-plant would leave the body
        // resting on a stance it was never solved for.
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
    // A command withdrawn mid-engagement hands the feet straight to the reseat
    // ladder. The engagement cannot be run out at a zero command and cannot be
    // trusted to leave the feet home:
    //
    //  - Its stance legs integrate the *commanded* velocity, so a zero command
    //    freezes each one where the walk had carried it. Nothing brings them
    //    back, and a leg still waiting for its first lift-off never swings at
    //    all.
    //  - Its swing branch, by contrast, evaluates the strategy closed form,
    //    which at a zero stride *is* the nominal stance. So every leg teleports
    //    onto nominal the moment its own phase reaches lift-off — no lift, no
    //    arc — and the rest were declared home at the handoff.
    //
    // On a ripple that was the whole stop: its engagement is a full cycle long
    // (3.4 s), so a short drive usually ends inside it, and every planted foot
    // jumped up to half a stride at once. The ladder below arcs each foot home
    // instead, and skips the ones already there — so an early abort, where
    // nothing has moved yet, still costs nothing. Unlike the settle route there
    // is no all_planted() window to wait for: the engagement's swings overlap
    // whatever is planted, so the ladder lands the airborne feet itself.
    if (cmd_zero) {
      hand_off_to_reseat();
      return tick_reseat(dt);
    }
    auto out = tick_engagement(dt, v_body_xy, omega_z);
    if (engagement_->state() == EngagementState::DONE) {
      clock_->reset(engagement_->exit_master());
      stance_.seed(last_targets_, last_stance_);
      reset_swing_state();
      state_ = EngineState::GAIT;
    }
    return out;
  }

  if (state_ == EngineState::GAIT) {
    // A pending gait change is already folded into should_settle, via
    // cmd_gain_: it eases the command out first rather than entering the settle
    // at walking speed and dropping it there.
    if (should_settle) {
      // Nothing to arm: the clock, the stance integrator and the swing planner
      // all carry straight on, which is what makes a settle abortable for free.
      state_ = EngineState::SETTLING;
      auto out = tick_gait(dt, {0.0f, 0.0f}, 0.0f, /*settling=*/true);
      // The debounce already ran the gait at a zero stride, so the feet may
      // well be home the moment the settle arms.
      finish_or_hand_off_settle();
      return out;
    }
    return tick_gait(dt, v_body_xy, omega_z, /*settling=*/false);
  }

  if (state_ == EngineState::SETTLING) {
    // The clock never stopped and every foot is riding a live AEP, so a command
    // that comes back just carries on walking — no engagement pass to re-enter.
    if (!cmd_zero && !pending_strategy_name_.has_value()) {
      state_ = EngineState::GAIT;
      return tick_gait(dt, v_body_xy, omega_z, /*settling=*/false);
    }
    auto out = tick_gait(dt, {0.0f, 0.0f}, 0.0f, /*settling=*/true);
    finish_or_hand_off_settle();
    return out;
  }

  // Every state has its own branch above (RESEATING among them); this is only
  // reachable if one is added without one.
  return emit_held();
}

std::map<std::string, LegOutput> Engine::tick_gait(
    float dt, std::pair<float, float> v_body_xy, float omega_z, bool settling) {
  // A zero command is not a special case: derive_cycle_time falls back to
  // max_cycle_time, stride_vector collapses to zero, and AEP == PEP == nominal,
  // so the gait steps in place until the debounce arms the settle. Holding the
  // targets instead would freeze an airborne leg mid-arc and put a step in the
  // velocity -> foot-target map at cmd_zero_tol.
  //
  // Settling takes that to its conclusion. cmd_gain_ has already eased the
  // command to a standstill by the time this state is reached, so the zero here
  // costs nothing — it just makes the AEP land on nominal to the last bit.
  if (settling) {
    v_body_xy = {0.0f, 0.0f};
    omega_z = 0.0f;
  } else {
    v_body_xy = {v_body_xy.first * cmd_gain_, v_body_xy.second * cmd_gain_};
    omega_z *= cmd_gain_;
  }
  const float stride_length = config_.stride_length;
  const float swing_end =
      swing_end_phase(strategy_->duty_factor(), config_.swing_phase_margin);
  const float stance_fraction = 1.0f - swing_end;
  const SwingProfile swing_profile = config_.swing_profile();

  const auto leg_velocities = per_leg_planar_velocity(legs_, v_body_xy, omega_z);
  float max_leg_v = 0.0f;
  for (const auto& [name, v] : leg_velocities) {
    (void)name;
    max_leg_v = std::max(max_leg_v, std::hypot(v.first, v.second));
  }

  const auto [min_cycle_time, max_cycle_time] =
      cycle_time_bounds(config_, swing_end);
  // Scaled by 1 / swing_end exactly as cycle_time_bounds scales the walk's, so
  // every gait gets the same airborne time out of one settle_swing_time. Faded
  // in on the same gain as the command, so the clock never changes rate under a
  // foot that is already in the air.
  const float settle_cycle_time =
      config_.settle_swing_time / std::max(swing_end, 1.0e-6f);
  const float walk_cycle_time =
      derive_cycle_time(max_leg_v, config_.stride_length, stance_fraction,
                        min_cycle_time, max_cycle_time);
  const float cycle_time =
      settling ? settle_cycle_time
               : cmd_gain_ * walk_cycle_time +
                     (1.0f - cmd_gain_) * settle_cycle_time;
  const float stance_time = cycle_time * stance_fraction;
  const float swing_time = cycle_time * swing_end;

  // On the ladder's route the gait's job is only to land what is already in the
  // air: no leg that is down may start a swing, or a gait whose swings overlap
  // end to end would never give the ladder the all-planted moment it starts
  // from. A held leg does not move at all — the command is zero, so its stance
  // target was standing still anyway.
  const bool hold_liftoffs = settling && !settle_beats_reseat();

  // A steady walk rides exactly on `stance_band` at AEP and PEP, so the bound is
  // free until the command turns under a planted leg.
  const float stance_band = 0.5f * stride_length;
  const float stance_ceiling = stance_band * (1.0f + kStanceExcursionGrace);

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
      // Its lift-off came due while lift-offs were held, so it sits the window
      // out. The flag outlives the settle on purpose: dropping it the moment a
      // command came back would start the arc at the phase the leg had reached,
      // i.e. part-way along, and jump the foot. Cleared when the phase leaves
      // the swing window and the leg is a plain stance leg again.
      held_down_[name] = true;
      stance = true;
    }
    const Vec3 nominal = nominal_[name];
    // Read off nominal_ every tick so a reseat's new stance is picked up on the
    // next one, with no second copy to keep in step.
    const StanceBand bound{nominal, stance_band, stance_ceiling};

    Vec3 target;
    if (stance) {
      Vec3 touchdown_anchor;
      if (swing_.is_swing(name)) {
        // Touchdown edge: adopt the latched swing target as the new anchor. At a
        // zero stride that target is the leg's nominal stance, which is how a
        // settle re-plants without a re-plant pass.
        touchdown_anchor = swing_.target(name);
        swing_.touchdown(name);
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
      // Re-aim the touchdown end at the live AEP and stance velocity. The
      // stance integrator picks up this target at touchdown and immediately
      // integrates it at the live velocity, so a latched target would show up
      // as a velocity step at the seam whenever cmd_vel moved mid-swing.
      swing_.retarget(name, aep, {v_x, v_y}, std::max(swing_time, 1.0e-9f));
      const float phase_in_swing =
          swing_end > 0.0f ? phases.at(name) / swing_end : 0.0f;
      target = swing_.evaluate(name, phase_in_swing, swing_profile);
      // Keep the stance integrator's per-leg flag in sync.
      stance_.step(name, false, target, {v_x, v_y}, dt, bound);
    }

    out[name] = LegOutput{target, phases.at(name), stance};
  }

  capture_state(out);
  return out;
}

// Standing is a position, not a sequence: every foot planted, on its nominal
// stance. Asking the question this way rather than counting touchdowns since the
// settle armed is what keeps the debounce free — a leg that already re-planted
// while the command was ramping out does not have to do it again to be believed.
bool Engine::all_settled() const {
  for (const auto& n : LEG_NAMES) {
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
    if (!last_stance_.at(n)) {
      return false;
    }
  }
  return true;
}

bool Engine::settle_beats_reseat() const {
  const float swing_end =
      swing_end_phase(strategy_->duty_factor(), config_.swing_phase_margin);
  if (swing_end <= 0.0f) {
    return false;
  }
  // Standing means every foot down on nominal at the same instant, so a gait
  // that never has all six down cannot finish a settle on its own however long
  // it is given.
  if (!has_all_down_window(strategy_->phase_offsets(), swing_end)) {
    return false;
  }
  // Worst case for the gait: the last leg to get its turn is a whole cycle away.
  const float natural = config_.settle_swing_time / swing_end;
  // Worst case for the ladder: one swing spent waiting for the legs already in
  // the air to land, then three mirrored pairs with a dwell between them. The
  // ladder's own landing stage is empty here: this route only reaches it with
  // every foot planted.
  const float pairs = static_cast<float>(PAIR_ORDER.size());
  const float ladder = config_.settle_swing_time +
                       pairs * config_.reseat_pair_swing_time +
                       (pairs - 1.0f) * config_.reseat_pair_dwell_time;
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
  // The gait would take a whole cycle to get round every leg, and on a long duty
  // factor that is several times what the ladder needs. Hand the legs that are
  // still out to the reseat, which lifts three mirrored pairs and skips whatever
  // the settle already brought home. Every foot is planted here, so the ladder
  // starts from rest with nothing in the air to snap down.
  hand_off_to_reseat();
}

void Engine::hand_off_to_reseat() {
  // build_reseat reads last_targets_, so the ladder starts from where the feet
  // actually are. Anything still in the air is landed by the ladder's own first
  // stage, before it lifts a foot that is down — which is what makes this safe
  // to call without an all_planted() check.
  stance_.reset();
  reset_swing_state();
  reseat_ = build_reseat(nominal_);
  reseat_target_stance_ = nominal_;
  reseat_target_height_ = applied_height_;
  state_ = EngineState::RESEATING;
}

void Engine::finish_settling() {
  // Commit a pending gait change at the handoff — the new strategy's phase
  // offsets are only meaningful from a standing start.
  std::optional<std::string> pending = pending_strategy_name_;
  pending_strategy_name_.reset();
  if (pending.has_value() && *pending != strategy_name_) {
    apply_strategy(*pending);
  }
  stance_.reset();
  reset_swing_state();
  state_ = EngineState::STAND;
  // A no-op by construction: the last leg landed on nominal and the other five
  // have been frozen there since their own touchdowns. Stated anyway, so STAND
  // is never entered holding a stale target.
  last_targets_ = nominal_;
  for (const auto& n : LEG_NAMES) last_stance_[n] = true;
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
  cfg.touchdown_velocity = c.touchdown_velocity;
  cfg.touchdown_probe_fraction = c.touchdown_probe_fraction;
  cfg.swing_phase_margin = c.swing_phase_margin;
  cfg.controller_dt = c.controller_dt;
  cfg.cmd_zero_tol = c.cmd_zero_tol;
  cfg.settle_debounce_delay = c.settle_debounce_delay;
  cfg.settle_swing_time = c.settle_swing_time;
  cfg.init_pair_swing_time = c.init_pair_swing_time;
  cfg.init_lift_body_time = c.init_lift_body_time;
  cfg.init_swing_clearance = c.init_swing_clearance;
  cfg.reseat_pose_settle_delay = c.reseat_pose_settle_delay;
  cfg.reseat_height_change_threshold = c.reseat_height_change_threshold;
  cfg.reseat_pair_swing_time = c.reseat_pair_swing_time;
  cfg.reseat_pair_dwell_time = c.reseat_pair_dwell_time;
  cfg.reseat_swing_clearance = c.reseat_swing_clearance;
  return cfg;
}

// The baked constants as std::arrays, so the no-arg builders can delegate to the
// parameterized versions without repeating the constexpr wiring.
namespace {
std::array<kin::LegSpec, kNumLegs> baked_leg_specs() {
  return ::hexa::config::kLegSpecs;
}
}  // namespace

namespace {

// Body-frame foot position per leg for a set of per-leg joint angles.
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

std::array<JointAngles, kNumLegs> standing_pose_from(
    const std::array<kin::LegSpec, kNumLegs>& specs, float coxa_to_bottom,
    float foot_radius, const ::hexa::config::StandingPose& standing) {
  // The foot tip sits its group's tip_reach out from the coxa axis, swivelled by
  // the group's splay, deep enough that the tip touches ground body_height below
  // the body bottom. IK aims at the tip sphere's centre, one radius above that
  // contact point.
  const float target_z = kin::ik_z_for_contact(
      -(coxa_to_bottom + standing.body_height), foot_radius);

  std::array<JointAngles, kNumLegs> out{};
  for (std::size_t i = 0; i < kNumLegs; ++i) {
    const Leg leg = static_cast<Leg>(i);
    const LegGroup group = leg_group(leg);
    const auto& grp = standing.groups[static_cast<std::size_t>(
        group_index(group))];

    // The configured splay is the left leg's and positive means outward, so
    // turning it into a joint angle is two negations: rear legs face the other
    // way down the body, and right legs mirror left ones. Keep in step with
    // gen_config.py's group_splay, which the teleop closed form goes through.
    const float sign = (group == LegGroup::REAR ? -1.0f : 1.0f) *
                       (leg_is_right(leg) ? -1.0f : 1.0f);
    const float th_c = sign * grp.coxa;

    // IK recovers th_c from atan2(y, x) and solves femur/tibia from the radial
    // reach, which is the group's tip_reach whatever the splay — so femur/tibia
    // come out uniform within a group, and differ between groups only where the
    // reaches do. Throws UnreachableTarget if it cannot.
    out[i] = kin::inverse_kinematics(
        Vec3(grp.tip_reach * std::cos(th_c), grp.tip_reach * std::sin(th_c),
             target_z),
        specs[i]);

    for (std::size_t j = 0; j < 3; ++j) {
      const auto& lim = ::hexa::config::kJointLimits[j];
      if (out[i][j] < lim.lower || out[i][j] > lim.upper) {
        static constexpr std::array<const char*, 3> kJointNames = {
            "coxa", "femur", "tibia"};
        throw std::invalid_argument(
            "standing pose " + std::string(kJointNames[j]) + " for " +
            LEG_NAMES[i] + " is " + std::to_string(out[i][j]) +
            " rad, outside the joint limit window [" +
            std::to_string(lim.lower) + ", " + std::to_string(lim.upper) +
            "] rad — check tuning.yaml default_standing_pose." +
            std::string(leg_group_name(group)));
      }
    }
  }
  return out;
}

std::map<std::string, Vec3> nominal_stance_from(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::array<JointAngles, kNumLegs>& standing_pose) {
  return stance_from(specs, standing_pose);
}

std::map<std::string, Vec3> initial_stance_from(
    const std::array<kin::LegSpec, kNumLegs>& specs,
    const std::array<JointAngles, kNumLegs>& initial_pose) {
  return stance_from(specs, initial_pose);
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
  // One snapshot per leg. The coxa component is irrelevant here (the geometry
  // captures depth and tibia lean, both invariant under the leg's swivel, and
  // reseat re-applies each leg's own azimuth when it places the targets), but
  // the lean itself is not shared: legs in different groups reach out different
  // distances and so stand with different femur/tibia pairs.
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

std::array<JointAngles, kNumLegs> standing_pose_from_config() {
  return standing_pose_from(baked_leg_specs(), ::hexa::config::kCoxaToBottom,
                            ::hexa::config::kFootRadius,
                            ::hexa::config::kStandingPose);
}

std::map<std::string, Vec3> nominal_stance_from_config() {
  return nominal_stance_from(baked_leg_specs(), standing_pose_from_config());
}

std::map<std::string, Vec3> initial_stance_from_config() {
  return initial_stance_from(baked_leg_specs(), ::hexa::config::kInitialPose);
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
    const std::array<JointAngles, kNumLegs>& initial_pose,
    float coxa_to_bottom, float foot_radius) {
  auto factory = strategies().find(strategy_name);
  if (factory == strategies().end()) {
    throw std::invalid_argument("unknown strategy: " + strategy_name);
  }
  return std::make_unique<Engine>(
      engine_cfg, factory->second(), strategy_name,
      nominal_stance_from(specs, standing_pose),
      initial_stance_from(specs, initial_pose), coxa_to_bottom, foot_radius,
      build_leg_contexts_from(specs, standing_pose), leg_specs_from(specs),
      reseat_geometry_from(specs, standing_pose));
}

std::unique_ptr<Engine> make_default_engine(const std::string& strategy_name) {
  return make_default_engine(
      strategy_name, baked_leg_specs(), engine_config_from_config(),
      standing_pose_from_config(), ::hexa::config::kInitialPose,
      ::hexa::config::kCoxaToBottom, ::hexa::config::kFootRadius);
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
    case EngineState::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

}  // namespace hexa::gait
