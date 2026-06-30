#include "hexa_gait_cpp/engagement.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "hexa_gait_cpp/validation.hpp"

namespace hexa_gait {

namespace {
// f(tau) = 3 tau^2 - 2 tau^3 clamped to [0, 1]. Smoothstep / Hermite-3.
double smoothstep_env(double tau) {
  if (tau <= 0.0) {
    return 0.0;
  }
  if (tau >= 1.0) {
    return 1.0;
  }
  return tau * tau * (3.0 - 2.0 * tau);
}
}  // namespace

EngagementController::EngagementController(
    std::map<std::string, Vec3> nominal_stance, double stride_length,
    double min_cycle_time, double max_cycle_time, double duty_factor,
    double swing_clearance, double swing_width, double controller_dt)
    : stride_length_(stride_length),
      min_cycle_time_(min_cycle_time),
      max_cycle_time_(max_cycle_time),
      duty_factor_(duty_factor),
      swing_end_(1.0 - duty_factor),
      swing_clearance_(swing_clearance),
      swing_width_(swing_width),
      controller_dt_(controller_dt) {
  require_all_legs(nominal_stance, "nominal_stance");
  for (const auto& name : LEG_NAMES) {
    nominal_[name] = nominal_stance.at(name);
    is_initial_swing_[name] = false;
    first_lift_off_master_[name] = 0.0;
    first_touchdown_master_[name] = 0.0;
    has_lifted_off_[name] = false;
    has_completed_first_swing_[name] = false;
  }
  foot_position_ = nominal_;
  lift_off_position_ = nominal_;
}

void EngagementController::begin(
    const Strategy& strategy,
    const std::map<std::string, LegContext>& leg_contexts) {
  require_all_legs(leg_contexts, "leg_contexts");
  if (strategy.duty_factor() != duty_factor_) {
    throw std::invalid_argument(
        "strategy duty_factor does not match controller duty_factor");
  }

  mode_ = "engage";
  strategy_ = &strategy;
  leg_contexts_ = leg_contexts;
  const auto& offsets = strategy.phase_offsets().offsets();

  // 1e-9 tolerance covers float artefacts when offset and swing_end share a
  // common irrational (e.g. crawl's r_middle at 1/3 vs 1 - 2/3).
  const double boundary = swing_end_ - 1e-9;
  double min_first_touchdown = std::numeric_limits<double>::infinity();
  for (const auto& name : LEG_NAMES) {
    const double o = offsets.at(name);
    if (o < boundary) {
      // Initial-swing: lift off at master = 0 from NOMINAL.
      is_initial_swing_[name] = true;
      first_lift_off_master_[name] = 0.0;
      first_touchdown_master_[name] = swing_end_ - o;
    } else {
      // Initial-stance: grounded until phase = 0 (lift off at master = 1 - o).
      is_initial_swing_[name] = false;
      first_lift_off_master_[name] = 1.0 - o;
      first_touchdown_master_[name] = (1.0 - o) + swing_end_;
    }
    min_first_touchdown =
        std::min(min_first_touchdown, first_touchdown_master_[name]);
  }

  // Smoothstep saturates at the earliest first touchdown.
  smoothstep_window_ = min_first_touchdown;

  master_ = 0.0;
  v_body_x_ = 0.0;
  v_body_y_ = 0.0;
  omega_ = 0.0;
  foot_position_ = nominal_;
  lift_off_position_ = nominal_;
  for (const auto& name : LEG_NAMES) {
    // Initial-swing legs lift off from NOMINAL at master = 0; initial-stance
    // legs snapshot when they cross INITIAL_STANCE -> INITIAL_SWING.
    has_lifted_off_[name] = is_initial_swing_[name];
    has_completed_first_swing_[name] = false;
  }

  state_ = EngagementState::ENGAGING;
}

void EngagementController::begin_resume(
    const Strategy& strategy,
    const std::map<std::string, LegContext>& leg_contexts,
    const std::map<std::string, Vec3>& last_targets,
    const std::map<std::string, bool>& prev_swing_flags, double master_phase) {
  require_all_legs(leg_contexts, "leg_contexts");
  require_all_legs(last_targets, "last_targets");
  if (strategy.duty_factor() != duty_factor_) {
    throw std::invalid_argument(
        "strategy duty_factor does not match controller duty_factor");
  }
  if (!(master_phase >= 0.0 && master_phase < 1.0)) {
    throw std::invalid_argument("master_phase must be in [0, 1)");
  }

  mode_ = "resume";
  strategy_ = &strategy;
  leg_contexts_ = leg_contexts;
  const auto& offsets = strategy.phase_offsets().offsets();

  std::map<std::string, Vec3> lift_off_position = nominal_;
  for (const auto& name : LEG_NAMES) {
    const double phase = pymod(master_phase + offsets.at(name), 1.0);
    auto flag = prev_swing_flags.find(name);
    const bool was_swing = flag != prev_swing_flags.end() && flag->second;
    if (was_swing) {
      // Was airborne: merge arc starts now from the lowered position.
      is_initial_swing_[name] = true;
      first_lift_off_master_[name] = master_phase;
      first_touchdown_master_[name] =
          master_phase + std::max(0.0, swing_end_ - phase);
      lift_off_position[name] = last_targets.at(name);
      has_lifted_off_[name] = true;
    } else {
      // Was stance: integrate stance until phase wraps to 0, then swing.
      is_initial_swing_[name] = false;
      first_lift_off_master_[name] = master_phase + (1.0 - phase);
      first_touchdown_master_[name] =
          first_lift_off_master_[name] + swing_end_;
      has_lifted_off_[name] = false;
    }
  }

  smoothstep_window_ = 1.0;  // unused in resume mode

  master_ = master_phase;
  v_body_x_ = 0.0;
  v_body_y_ = 0.0;
  omega_ = 0.0;
  for (const auto& name : LEG_NAMES) {
    foot_position_[name] = last_targets.at(name);
    has_completed_first_swing_[name] = false;
  }
  lift_off_position_ = lift_off_position;

  state_ = EngagementState::ENGAGING;
}

std::map<std::string, LegOutput> EngagementController::update(
    double dt, std::pair<double, double> v_cmd_xy, double omega_cmd) {
  if (state_ == EngagementState::IDLE) {
    return emit_nominal_stance();
  }

  // 1) Per-leg planar velocity from the commanded body velocity.
  const auto cmd_leg_v =
      per_leg_planar_velocity(leg_contexts_, v_cmd_xy, omega_cmd);
  double max_cmd_leg_v = 0.0;
  for (const auto& [name, v] : cmd_leg_v) {
    (void)name;
    max_cmd_leg_v = std::max(max_cmd_leg_v, std::hypot(v.first, v.second));
  }
  const double cycle_time =
      derive_cycle_time(max_cmd_leg_v, stride_length_, duty_factor_,
                        min_cycle_time_, max_cycle_time_);
  const double stance_time = cycle_time * duty_factor_;

  // 2) Advance master phase. Engage mode clamps at 1.0; resume advances freely.
  if (cycle_time > 0.0) {
    const double advanced = master_ + dt / cycle_time;
    master_ = (mode_ == "engage") ? std::min(advanced, 1.0) : advanced;
  }

  // 3) Body velocity envelope (engage mode only).
  double envelope;
  if (mode_ == "engage" && smoothstep_window_ > 0.0 &&
      master_ < smoothstep_window_) {
    envelope = smoothstep_env(master_ / smoothstep_window_);
  } else {
    envelope = 1.0;
  }
  v_body_x_ = v_cmd_xy.first * envelope;
  v_body_y_ = v_cmd_xy.second * envelope;
  omega_ = omega_cmd * envelope;

  // 4) Per-leg planar velocity at the internal body velocity.
  const auto body_leg_v = per_leg_planar_velocity(
      leg_contexts_, {v_body_x_, v_body_y_}, omega_);

  // 5) Per-leg output.
  const auto& offsets = strategy_->phase_offsets().offsets();
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    const double phase = pymod(master_ + offsets.at(name), 1.0);
    const double first_lift_off = first_lift_off_master_[name];
    const double first_touchdown = first_touchdown_master_[name];

    if (master_ >= first_touchdown) {
      // GAIT_LIKE.
      const bool in_stance = phase >= swing_end_;
      Vec3 foot;
      if (in_stance) {
        const auto& v = body_leg_v.at(name);
        Vec3& fp = foot_position_[name];
        fp = Vec3(fp[0] - v.first * dt, fp[1] - v.second * dt, fp[2]);
        foot = fp;
      } else {
        const auto& v = cmd_leg_v.at(name);
        const Vec3 stride_vec =
            stride_vector(v.first, v.second, stance_time, stride_length_);
        StrideParams stride;
        stride.stride_vector = stride_vec;
        stride.cycle_time = cycle_time;
        stride.duty_factor = duty_factor_;
        stride.swing_clearance = swing_clearance_;
        stride.swing_width = swing_width_;
        stride.controller_dt = controller_dt_;
        foot = strategy_->foot_target(phase, stride, leg_contexts_.at(name));
        foot_position_[name] = foot;
      }
      out[name] = LegOutput{foot, phase, in_stance};
      has_completed_first_swing_[name] = true;
    } else if (master_ >= first_lift_off) {
      // INITIAL_SWING: arc from the lift-off snapshot to the live AEP.
      if (!has_lifted_off_[name]) {
        lift_off_position_[name] = foot_position_[name];
        has_lifted_off_[name] = true;
      }

      const auto& vc = cmd_leg_v.at(name);
      const Vec3 stride_vec =
          stride_vector(vc.first, vc.second, stance_time, stride_length_);
      const Vec3 nominal = nominal_[name];
      const Vec3 aep = live_aep(nominal, stride_vec);

      const double leg_swing_master = master_ - first_lift_off;
      const double leg_swing_duration_master = first_touchdown - first_lift_off;
      double phase_in_swing =
          leg_swing_duration_master > 0.0
              ? leg_swing_master / leg_swing_duration_master
              : 0.0;
      phase_in_swing = std::max(0.0, std::min(phase_in_swing, 1.0));
      const double leg_swing_time = leg_swing_duration_master * cycle_time;

      const auto& vb = body_leg_v.at(name);
      const Vec3 foot =
          swing_arc(phase_in_swing, lift_off_position_[name], aep,
                    swing_clearance_, swing_width_, identity_y_sign(nominal),
                    leg_swing_time, controller_dt_, Vec3::Zero(),
                    Vec3(-vb.first, -vb.second, 0.0));
      foot_position_[name] = foot;
      out[name] = LegOutput{foot, phase, false};
    } else {
      // INITIAL_STANCE: integrate the internal body velocity.
      const auto& vb = body_leg_v.at(name);
      Vec3& fp = foot_position_[name];
      fp = Vec3(fp[0] - vb.first * dt, fp[1] - vb.second * dt, fp[2]);
      out[name] = LegOutput{fp, phase, true};
    }
  }

  if (mode_ == "engage") {
    if (master_ >= 1.0) {
      state_ = EngagementState::DONE;
    }
  } else {
    bool all_done = true;
    for (const auto& name : LEG_NAMES) {
      if (!has_completed_first_swing_[name]) {
        all_done = false;
        break;
      }
    }
    if (all_done) {
      state_ = EngagementState::DONE;
    }
  }

  return out;
}

std::map<std::string, LegOutput> EngagementController::emit_nominal_stance()
    const {
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    out[name] = LegOutput{nominal_.at(name), 0.0, true};
  }
  return out;
}

}  // namespace hexa_gait
