#include "gait/stand_transition.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>

#include "gait/gaits/base.hpp"
#include "gait/kinematics.hpp"

namespace hexa::gait {

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
  const float s = eased_ramp(tau);
  for (const auto& name : LEG_NAMES) {
    const Vec3& from = from_.at(name);
    const Vec3& to = to_.at(name);
    // Straight chord: both ends are in the air, nothing to climb over.
    const Vec3 point = from + (to - from) * s;
    // stance=true throughout: the belly carries the robot, and no leg here is
    // available to plan a step with.
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

namespace {

bool is_pair_leg(const std::string& name) {
  for (const auto& parked : PARKED_LEGS) {
    if (parked == name) {
      return true;
    }
  }
  return false;
}

}  // namespace

PairFoldController::PairFoldController(
    PairFoldDirection direction, std::map<std::string, Vec3> held_stance,
    std::map<std::string, Vec3> folded_stance,
    std::map<std::string, Vec3> nominal_stance, float swing_time,
    float dwell_time, float probe_band, const SwingProfile& swing,
    float controller_dt)
    : direction_(direction),
      swing_time_(swing_time),
      dwell_time_(dwell_time),
      swing_(swing),
      controller_dt_(controller_dt) {
  require_all_legs(held_stance, "held_stance");
  require_all_legs(folded_stance, "folded_stance");
  require_all_legs(nominal_stance, "nominal_stance");
  if (swing_time <= 0.0f) {
    throw std::invalid_argument("swing_time must be positive");
  }
  if (dwell_time < 0.0f) {
    throw std::invalid_argument("dwell_time must not be negative");
  }
  if (swing.clearance != 0.0f) {
    // Not a preference. The folded end is always the higher of the two, and its
    // femur sits on its lower joint limit, so a clearance would ask for an
    // unreachable arc over it.
    throw std::invalid_argument("pair fold swing must have zero clearance");
  }
  for (const auto& name : LEG_NAMES) {
    held_[name] = held_stance.at(name);
  }
  for (const auto& name : PARKED_LEGS) {
    if (direction_ == PairFoldDirection::FOLD) {
      // Planted, so the caller's stance is honest for this end.
      origin_[name] = held_stance.at(name);
      chord_end_[name] = folded_stance.at(name);
      final_[name] = folded_stance.at(name);
    } else {
      // NOT held_stance: see the header.
      origin_[name] = folded_stance.at(name);
      final_[name] = nominal_stance.at(name);
      chord_end_[name] = final_.at(name);
    }
    pair_pos_[name] = origin_.at(name);
  }
  // The braked descent, on the way down only: hand the chord over probe_band
  // above the target and cover the rest at the gait's own touchdown speed.
  if (direction_ == PairFoldDirection::UNFOLD && probe_band > 0.0f &&
      swing.touchdown_velocity > 0.0f) {
    set_down_time_ = probe_band / swing.touchdown_velocity;
    for (const auto& name : PARKED_LEGS) {
      chord_end_[name] = final_.at(name) + Vec3{0.0f, 0.0f, probe_band};
    }
  }
}

std::map<std::string, LegOutput> PairFoldController::emit(
    float pair_phase, bool pair_stance) const {
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    if (is_pair_leg(name)) {
      out[name] = LegOutput{pair_pos_.at(name), pair_phase, pair_stance};
    } else {
      // Every corner held exactly where it was handed over, planted. All six
      // every tick: the engine reads the whole map back out.
      out[name] = LegOutput{held_.at(name), 0.0f, true};
    }
  }
  return out;
}

std::map<std::string, LegOutput> PairFoldController::update(float dt) {
  // True while the pair is still standing on the ground, which is only ever the
  // dwell at the head of a fold.
  const bool grounded_at_start = direction_ == PairFoldDirection::FOLD;

  switch (state_) {
    case PairFoldState::DWELL: {
      t_ += dt;
      if (t_ >= dwell_time_) {
        t_ = 0.0f;
        state_ = PairFoldState::MOVE;
      }
      return emit(0.0f, grounded_at_start);
    }

    case PairFoldState::MOVE: {
      t_ += dt;
      const float phase = t_ / swing_time_;
      if (phase >= 1.0f) {
        for (const auto& name : PARKED_LEGS) {
          pair_pos_[name] = chord_end_.at(name);
        }
        t_ = 0.0f;
        state_ = set_down_time_ > 0.0f ? PairFoldState::SET_DOWN
                                       : PairFoldState::DONE;
        return emit(1.0f, state_ == PairFoldState::DONE &&
                              direction_ == PairFoldDirection::UNFOLD);
      }
      for (const auto& name : PARKED_LEGS) {
        const Vec3& origin = origin_.at(name);
        const Vec3& target = chord_end_.at(name);
        // Rest to rest, zero clearance and zero width: swing_arc degenerates to
        // the eased chord between the two ends, which is the whole move.
        pair_pos_[name] =
            swing_arc(phase, origin, target, identity_y_sign(target),
                      swing_time_, swing_, Vec3::Zero(), Vec3::Zero());
      }
      return emit(phase, false);
    }

    case PairFoldState::SET_DOWN: {
      t_ += dt;
      const float s = std::min(t_ / set_down_time_, 1.0f);
      for (const auto& name : PARKED_LEGS) {
        const Vec3& from = chord_end_.at(name);
        const Vec3& to = final_.at(name);
        // Linear, so the contact speed is exactly touchdown_velocity rather
        // than the peak of some ramp.
        pair_pos_[name] = from + (to - from) * s;
      }
      if (s >= 1.0f) {
        state_ = PairFoldState::DONE;
        return emit(1.0f, true);
      }
      return emit(1.0f, false);
    }

    case PairFoldState::DONE:
      break;
  }
  return emit(1.0f, direction_ == PairFoldDirection::UNFOLD);
}

InitializeController::InitializeController(
    LegSet leg_set, std::map<std::string, Vec3> folded_stance,
    std::map<std::string, Vec3> initialized_stance,
    std::map<std::string, Vec3> nominal_stance, float coxa_to_bottom,
    float foot_radius, float pair_swing_time, float lift_body_time,
    float unfold_time, float place_clearance, float swing_clearance,
    float swing_width, float touchdown_velocity,
    float touchdown_probe_fraction, float controller_dt)
    : leg_set_(leg_set),
      rungs_(pair_list(leg_set)),
      pair_swing_time_(pair_swing_time),
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
  // Zero is legal: the feet then land at the end of PLACE_FEET.
  if (place_clearance < 0.0f) {
    throw std::invalid_argument("place_clearance must not be negative");
  }
  for (const auto& name : LEG_NAMES) {
    folded_[name] = folded_stance.at(name);
    initialized_[name] = initialized_stance.at(name);
    nominal_[name] = nominal_stance.at(name);
  }
  // Standing XY, place_clearance above the floor the belly rests on. The pairs
  // stop short rather than landing: a pair that lands has to hold the floor
  // while the next two swing, and any belly-height error is taken as a preload
  // against a leg that cannot yet be unloaded, so the robot shuffles. Held
  // clear, the six meet the floor together inside the LIFT_BODY ramp.
  lift_start_z_ =
      kin::ik_z_for_contact(-coxa_to_bottom + place_clearance, foot_radius);
  for (const auto& name : LEG_NAMES) {
    ground_targets_[name] =
        Vec3(nominal_[name][0], nominal_[name][1], lift_start_z_);
  }
  positions_ = folded_stance;
}

std::map<std::string, LegOutput> InitializeController::update(float dt) {
  std::map<std::string, LegOutput> out;
  if (state_ == InitializeState::UNFOLD) {
    out = tick_unfold(dt);
  } else if (state_ == InitializeState::PLACE_FEET) {
    out = tick_place_feet(dt);
  } else if (state_ == InitializeState::LIFT_BODY) {
    out = tick_lift_body(dt);
  } else {
    out = emit_nominal();
  }
  // The rungs already skip the parked pair; the unfold's chord does not, so the
  // pin is what holds those two legs at the pose they powered up in.
  pin_parked(leg_set_, folded_, out);
  return out;
}

std::map<std::string, LegOutput> InitializeController::tick_unfold(float dt) {
  auto out = unfold_.update(dt);
  for (const auto& name : LEG_NAMES) {
    // The chord is not a parked leg's to walk: positions_ starts it at folded
    // and nothing here moves it off, which is what the pin then emits.
    if (leg_is_parked(leg_set_, name)) {
      continue;
    }
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
  const Rung& active = rungs_[pair_idx_];

  std::map<std::string, LegOutput> out;
  if (phase >= 1.0f) {
    for (const auto& name : active) {
      positions_[name] = ground_targets_[name];
    }
    pair_idx_ += 1;
    t_in_pair_ = 0.0f;
    if (pair_idx_ >= rungs_.size()) {
      state_ = InitializeState::LIFT_BODY;
    }
    for (const auto& name : LEG_NAMES) {
      out[name] = LegOutput{positions_[name], 0.0f, true};
    }
    return out;
  }

  // Mid-pair: a rest-to-rest swing arc down to the ground target at the gait's
  // touchdown speed. The probe still earns its keep with the target held clear:
  // place_clearance is the slack an early contact has to be absorbed into.
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
  // Standing XY throughout; body-frame z ramps from the PLACE_FEET endpoint down
  // to nominal_stance.z. The first place_clearance of that travel is the feet
  // taking the load, which is why this uses lift_ramp and not eased_ramp.
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
    // Snap to nominal so downstream sees no drift.
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

namespace {
// Belly-resting throughout LIFT_FEET, so weight-bearing is not the constraint;
// reversing just unwinds the order a reseat would have planted the feet in.
RungList rungs_reversed(LegSet set) {
  RungList out = pair_list(set);
  std::reverse(out.begin(), out.end());
  return out;
}
}  // namespace

FoldController::FoldController(LegSet leg_set,
                               std::map<std::string, Vec3> folded_stance,
                               std::map<std::string, Vec3> initialized_stance,
                               std::map<std::string, Vec3> nominal_stance,
                               float coxa_to_bottom, float foot_radius,
                               float pair_swing_time, float lift_body_time,
                               float tuck_time, float swing_clearance,
                               float swing_width, float touchdown_velocity,
                               float touchdown_probe_fraction,
                               float controller_dt)
    : leg_set_(leg_set),
      rungs_(rungs_reversed(leg_set)),
      pair_swing_time_(pair_swing_time),
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
  // LOWER_BODY endpoint == LIFT_FEET swing origin, so the belly has taken the
  // whole load before the first foot lifts.
  lower_end_z_ = kin::ik_z_for_contact(-coxa_to_bottom, foot_radius);
  for (const auto& name : LEG_NAMES) {
    ground_targets_[name] =
        Vec3(nominal_[name][0], nominal_[name][1], lower_end_z_);
  }
  positions_ = nominal_;
  // A parked leg is already folded, and TUCK builds its chord out of positions_.
  for (const auto& name : PARKED_LEGS) {
    if (leg_is_parked(leg_set_, name)) {
      positions_[name] = folded_[name];
    }
  }
}

std::map<std::string, LegOutput> FoldController::update(float dt) {
  std::map<std::string, LegOutput> out;
  if (state_ == FoldState::LOWER_BODY) {
    out = tick_lower_body(dt);
  } else if (state_ == FoldState::LIFT_FEET) {
    out = tick_lift_feet(dt);
  } else if (state_ == FoldState::TUCK) {
    out = tick_tuck(dt);
  } else {
    out = emit_folded();
  }
  pin_parked(leg_set_, folded_, out);
  return out;
}

std::map<std::string, LegOutput> FoldController::tick_lower_body(float dt) {
  // Standing XY throughout; z ramps up to lower_end_z, so the belly reaches the
  // floor exactly as the ramp runs out of speed.
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
  const Rung& active = rungs_[pair_idx_];

  std::map<std::string, LegOutput> out;
  if (phase >= 1.0f) {
    for (const auto& name : active) {
      positions_[name] = initialized_[name];
    }
    pair_idx_ += 1;
    t_in_pair_ = 0.0f;
    if (pair_idx_ >= rungs_.size()) {
      tuck_.emplace(positions_, folded_, tuck_time_);
      state_ = FoldState::TUCK;
    }
    for (const auto& name : LEG_NAMES) {
      out[name] = LegOutput{positions_[name], 0.0f, true};
    }
    return out;
  }

  // Mid-pair: a rest-to-rest swing arc up to the initialized pose.
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
  // The unfold run backwards, belly down throughout.
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
