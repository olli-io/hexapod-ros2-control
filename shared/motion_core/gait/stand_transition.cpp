#include "gait/stand_transition.hpp"

#include <array>
#include <stdexcept>

#include "gait/gaits/base.hpp"
#include "gait/kinematics.hpp"  // kin::ik_z_for_contact

namespace hexa::gait {

// ─────────────────────────────── RestPoseMove ───────────────────────────────

RestPoseMove::RestPoseMove(std::map<std::string, Vec3> from_stance,
                           std::map<std::string, Vec3> to_stance,
                           float move_time)
    : move_time_(move_time) {
  require_all_legs(from_stance, "from_stance");
  require_all_legs(to_stance, "to_stance");
  if (move_time <= 0.0f) {
    throw std::invalid_argument("move_time must be positive");
  }
  for (const auto& name : LEG_NAMES) {
    from_[name] = from_stance.at(name);
    to_[name] = to_stance.at(name);
  }
}

std::map<std::string, LegOutput> RestPoseMove::update(float dt) {
  std::map<std::string, LegOutput> out;
  if (done_) {
    for (const auto& name : LEG_NAMES) {
      out[name] = LegOutput{to_.at(name), 1.0f, true};
    }
    return out;
  }

  t_ += dt;
  const float tau = t_ / move_time_;
  // ease5 for the same reason the body ramps use it: the servos start and stop
  // with zero acceleration, so nothing snaps at either end of the move.
  const float s = eased_ramp(tau);
  for (const auto& name : LEG_NAMES) {
    const Vec3& from = from_.at(name);
    const Vec3& to = to_.at(name);
    // Straight chord, no arc: both ends are in the air, so there is no floor to
    // climb over and nothing to probe down onto.
    const Vec3 point = from + (to - from) * s;
    // stance=true throughout — the belly is what is carrying the robot, and a
    // foot reported airborne here would tell the caller a leg is available to
    // plan a step with, which none of them are.
    out[name] = LegOutput{point, tau, true};
  }
  if (tau >= 1.0f) {
    for (const auto& name : LEG_NAMES) {
      out[name] = LegOutput{to_.at(name), 1.0f, true};
    }
    done_ = true;
  }
  return out;
}

// ───────────────────────────── InitializeController ─────────────────────────

InitializeController::InitializeController(
    std::map<std::string, Vec3> folded_stance,
    std::map<std::string, Vec3> initialized_stance,
    std::map<std::string, Vec3> nominal_stance, float coxa_to_bottom,
    float foot_radius, float pair_swing_time, float lift_body_time,
    float unfold_time, float place_clearance, float swing_clearance,
    float swing_width, float touchdown_velocity,
    float touchdown_probe_fraction, float controller_dt)
    : pair_swing_time_(pair_swing_time),
      lift_body_time_(lift_body_time),
      swing_{.clearance = swing_clearance,
             .width = swing_width,
             .touchdown_velocity = touchdown_velocity,
             .touchdown_probe_fraction = touchdown_probe_fraction},
      controller_dt_(controller_dt),
      unfold_(folded_stance, initialized_stance, unfold_time) {
  require_all_legs(initialized_stance, "initialized_stance");
  require_all_legs(nominal_stance, "nominal_stance");
  if (pair_swing_time <= 0.0f) {
    throw std::invalid_argument("pair_swing_time must be positive");
  }
  if (lift_body_time <= 0.0f) {
    throw std::invalid_argument("lift_body_time must be positive");
  }
  // Zero is legal — the feet then land at the end of PLACE_FEET, as they used
  // to. Negative would drive them through the floor the belly rests on.
  if (place_clearance < 0.0f) {
    throw std::invalid_argument("place_clearance must not be negative");
  }
  for (const auto& name : LEG_NAMES) {
    initialized_[name] = initialized_stance.at(name);
    nominal_[name] = nominal_stance.at(name);
  }
  // Per-leg target at the end of PLACE_FEET: standing XY, feet place_clearance
  // above the floor the belly is already resting on — which is -coxa_to_bottom
  // for the contact point, one foot_radius higher for the IK target that
  // produces it.
  //
  // The pairs deliberately stop short rather than landing. A pair that lands
  // has to hold the floor while the next two swing, and any error in the belly
  // height (foot compliance, an uneven floor, IK rounding) is taken as a
  // preload against a leg that cannot yet be unloaded — the robot shuffles as
  // each pair fights the one before it. Held clear, the six arrive unencumbered
  // and meet the floor together, once, in the opening stretch of the LIFT_BODY
  // ramp.
  lift_start_z_ =
      kin::ik_z_for_contact(-coxa_to_bottom + place_clearance, foot_radius);
  for (const auto& name : LEG_NAMES) {
    ground_targets_[name] =
        Vec3(nominal_[name][0], nominal_[name][1], lift_start_z_);
  }
  positions_ = folded_stance;
}

std::map<std::string, LegOutput> InitializeController::update(float dt) {
  if (state_ == InitializeState::UNFOLD) {
    return tick_unfold(dt);
  }
  if (state_ == InitializeState::PLACE_FEET) {
    return tick_place_feet(dt);
  }
  if (state_ == InitializeState::LIFT_BODY) {
    return tick_lift_body(dt);
  }
  return emit_nominal();
}

std::map<std::string, LegOutput> InitializeController::tick_unfold(float dt) {
  auto out = unfold_.update(dt);
  for (const auto& name : LEG_NAMES) {
    positions_[name] = out.at(name).foot_target;
  }
  if (unfold_.done()) {
    state_ = InitializeState::PLACE_FEET;
  }
  return out;
}

std::map<std::string, LegOutput> InitializeController::tick_place_feet(
    float dt) {
  t_in_pair_ += dt;
  const float phase = t_in_pair_ / pair_swing_time_;
  const std::array<std::string, 2>& active = PAIR_ORDER[pair_idx_];

  std::map<std::string, LegOutput> out;
  if (phase >= 1.0f) {
    // Snap the active pair to their ground targets and advance.
    for (const auto& name : active) {
      positions_[name] = ground_targets_[name];
    }
    pair_idx_ += 1;
    t_in_pair_ = 0.0f;
    if (pair_idx_ >= PAIR_ORDER.size()) {
      state_ = InitializeState::LIFT_BODY;
    }
    for (const auto& name : LEG_NAMES) {
      out[name] = LegOutput{positions_[name], 0.0f, true};
    }
    return out;
  }

  // Mid-pair: active legs follow a rest-to-rest swing arc from the initialized
  // pose down to the ground target, descending onto it at the gait's touchdown
  // speed. Endpoint ground velocities pinned to zero. The probe still earns its
  // keep with the target held clear of the floor: place_clearance is the slack
  // it has to absorb an early contact into, and a foot arriving slowly is what
  // keeps that contact — should the floor be higher than the belly says — a
  // touch rather than a stub.
  for (const auto& name : LEG_NAMES) {
    if (name == active[0] || name == active[1]) {
      const Vec3 origin = initialized_[name];
      const Vec3 target = ground_targets_[name];
      const Vec3 point =
          swing_arc(phase, origin, target, identity_y_sign(target),
                    pair_swing_time_, swing_, Vec3::Zero(), Vec3::Zero());
      positions_[name] = point;
      out[name] = LegOutput{point, phase, false};
    } else {
      out[name] = LegOutput{positions_[name], 0.0f, true};
    }
  }
  return out;
}

std::map<std::string, LegOutput> InitializeController::tick_lift_body(
    float dt) {
  // All six feet stay at standing XY; body-frame z ramps from the PLACE_FEET
  // endpoint — the feet held place_clearance above the floor — down to
  // nominal_stance.z. The first place_clearance of that travel is the six feet
  // reaching for the floor and taking the load; the rest is the body rising.
  // lift_ramp, not eased_ramp: the load transfer happens inside the ramp rather
  // than at its start, so it wants the septic's opening (see lift_ramp).
  t_in_lift_ += dt;
  const float tau = t_in_lift_ / lift_body_time_;
  const float s = lift_ramp(tau);
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    const Vec3& nom = nominal_[name];
    const float z = lift_start_z_ + s * (nom[2] - lift_start_z_);
    const Vec3 point(nom[0], nom[1], z);
    positions_[name] = point;
    out[name] = LegOutput{point, tau, true};
  }
  if (tau >= 1.0f) {
    // Snap to nominal so downstream sees no drift and advance to DONE.
    for (const auto& name : LEG_NAMES) {
      positions_[name] = nominal_[name];
      out[name] = LegOutput{nominal_[name], 1.0f, true};
    }
    state_ = InitializeState::DONE;
  }
  return out;
}

std::map<std::string, LegOutput> InitializeController::emit_nominal() const {
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    out[name] = LegOutput{nominal_.at(name), 0.0f, true};
  }
  return out;
}

// ──────────────────────────────── FoldController ────────────────────────────

namespace {
// Reverse of PAIR_ORDER. Belly-resting throughout LIFT_FEET, so weight-bearing
// is not the constraint; reversing it just unwinds the order a reseat would
// have planted the feet in.
const std::array<std::array<std::string, 2>, 3>& pair_order_reversed() {
  static const std::array<std::array<std::string, 2>, 3> reversed = {{
      PAIR_ORDER[2],
      PAIR_ORDER[1],
      PAIR_ORDER[0],
  }};
  return reversed;
}
}  // namespace

FoldController::FoldController(std::map<std::string, Vec3> folded_stance,
                               std::map<std::string, Vec3> initialized_stance,
                               std::map<std::string, Vec3> nominal_stance,
                               float coxa_to_bottom, float foot_radius,
                               float pair_swing_time, float lift_body_time,
                               float tuck_time, float swing_clearance,
                               float swing_width, float touchdown_velocity,
                               float touchdown_probe_fraction,
                               float controller_dt)
    : pair_swing_time_(pair_swing_time),
      lift_body_time_(lift_body_time),
      tuck_time_(tuck_time),
      swing_{.clearance = swing_clearance,
             .width = swing_width,
             .touchdown_velocity = touchdown_velocity,
             .touchdown_probe_fraction = touchdown_probe_fraction},
      controller_dt_(controller_dt) {
  require_all_legs(folded_stance, "folded_stance");
  require_all_legs(initialized_stance, "initialized_stance");
  require_all_legs(nominal_stance, "nominal_stance");
  if (pair_swing_time <= 0.0f) {
    throw std::invalid_argument("pair_swing_time must be positive");
  }
  if (lift_body_time <= 0.0f) {
    throw std::invalid_argument("lift_body_time must be positive");
  }
  if (tuck_time <= 0.0f) {
    throw std::invalid_argument("tuck_time must be positive");
  }
  for (const auto& name : LEG_NAMES) {
    folded_[name] = folded_stance.at(name);
    initialized_[name] = initialized_stance.at(name);
    nominal_[name] = nominal_stance.at(name);
  }
  // LOWER_BODY endpoint == LIFT_FEET swing origin: standing XY, body-on-belly
  // height. The feet come off the floor from where they were standing, so the
  // belly has taken the whole load before the first one lifts.
  lower_end_z_ = kin::ik_z_for_contact(-coxa_to_bottom, foot_radius);
  for (const auto& name : LEG_NAMES) {
    ground_targets_[name] =
        Vec3(nominal_[name][0], nominal_[name][1], lower_end_z_);
  }
  positions_ = nominal_;
}

std::map<std::string, LegOutput> FoldController::update(float dt) {
  if (state_ == FoldState::LOWER_BODY) {
    return tick_lower_body(dt);
  }
  if (state_ == FoldState::LIFT_FEET) {
    return tick_lift_feet(dt);
  }
  if (state_ == FoldState::TUCK) {
    return tick_tuck(dt);
  }
  return emit_folded();
}

std::map<std::string, LegOutput> FoldController::tick_lower_body(float dt) {
  // All six feet stay at standing XY; body-frame z ramps from nominal.z up to
  // lower_end_z (foot closer to body; body lowers onto belly), so the belly
  // reaches the floor exactly as the ramp runs out of speed.
  t_in_lower_ += dt;
  const float tau = t_in_lower_ / lift_body_time_;
  const float s = eased_ramp(tau);
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    const Vec3& nom = nominal_[name];
    const float z = nom[2] + s * (lower_end_z_ - nom[2]);
    const Vec3 point(nom[0], nom[1], z);
    positions_[name] = point;
    out[name] = LegOutput{point, tau, true};
  }
  if (tau >= 1.0f) {
    // Snap to the LOWER_BODY endpoint and advance to LIFT_FEET.
    for (const auto& name : LEG_NAMES) {
      positions_[name] = ground_targets_[name];
      out[name] = LegOutput{ground_targets_[name], 1.0f, true};
    }
    state_ = FoldState::LIFT_FEET;
  }
  return out;
}

std::map<std::string, LegOutput> FoldController::tick_lift_feet(float dt) {
  t_in_pair_ += dt;
  const float phase = t_in_pair_ / pair_swing_time_;
  const std::array<std::string, 2>& active = pair_order_reversed()[pair_idx_];

  std::map<std::string, LegOutput> out;
  if (phase >= 1.0f) {
    // Snap the active pair to their initialized_stance entries and advance.
    for (const auto& name : active) {
      positions_[name] = initialized_[name];
    }
    pair_idx_ += 1;
    t_in_pair_ = 0.0f;
    if (pair_idx_ >= pair_order_reversed().size()) {
      tuck_.emplace(positions_, folded_, tuck_time_);
      state_ = FoldState::TUCK;
    }
    for (const auto& name : LEG_NAMES) {
      out[name] = LegOutput{positions_[name], 0.0f, true};
    }
    return out;
  }

  // Mid-pair: active legs follow a rest-to-rest swing arc from the ground target
  // up to the initialized_stance position. Endpoint ground velocities zero.
  for (const auto& name : LEG_NAMES) {
    if (name == active[0] || name == active[1]) {
      const Vec3 origin = ground_targets_[name];
      const Vec3 target = initialized_[name];
      const Vec3 point =
          swing_arc(phase, origin, target, identity_y_sign(origin),
                    pair_swing_time_, swing_, Vec3::Zero(), Vec3::Zero());
      positions_[name] = point;
      out[name] = LegOutput{point, phase, false};
    } else {
      out[name] = LegOutput{positions_[name], 0.0f, true};
    }
  }
  return out;
}

std::map<std::string, LegOutput> FoldController::tick_tuck(float dt) {
  // The unfold run backwards: all six feet draw in from the initialized pose to
  // the folded one, belly down throughout.
  auto out = tuck_->update(dt);
  for (const auto& name : LEG_NAMES) {
    positions_[name] = out.at(name).foot_target;
  }
  if (tuck_->done()) {
    state_ = FoldState::DONE;
  }
  return out;
}

std::map<std::string, LegOutput> FoldController::emit_folded() const {
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    out[name] = LegOutput{folded_.at(name), 0.0f, true};
  }
  return out;
}

}  // namespace hexa::gait
