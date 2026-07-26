#include "gait/engagement.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace hexa::gait {

namespace {
// f(tau) = 3 tau^2 - 2 tau^3 clamped to [0, 1]. Smoothstep / Hermite-3.
float smoothstep_env(float tau) {
  if (tau <= 0.0f) {
    return 0.0f;
  }
  if (tau >= 1.0f) {
    return 1.0f;
  }
  return tau * tau * (3.0f - 2.0f * tau);
}
}  // namespace

EngagementController::EngagementController(
    std::map<std::string, Vec3> nominal_stance, float stride_length,
    float min_cycle_time, float max_cycle_time, float duty_factor,
    float swing_phase_margin, const SwingProfile& swing, float controller_dt)
    : stride_length_(stride_length),
      min_cycle_time_(min_cycle_time),
      max_cycle_time_(max_cycle_time),
      duty_factor_(duty_factor),
      swing_end_(swing_end_phase(duty_factor, swing_phase_margin)),
      swing_(swing),
      controller_dt_(controller_dt) {
  require_all_legs(nominal_stance, "nominal_stance");
  for (const auto& name : LEG_NAMES) {
    nominal_[name] = nominal_stance.at(name);
    is_initial_swing_[name] = false;
    first_lift_off_master_[name] = 0.0f;
    first_touchdown_master_[name] = 0.0f;
    has_lifted_off_[name] = false;
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

  strategy_ = &strategy;
  leg_contexts_ = leg_contexts;
  const auto& offsets = strategy.phase_offsets().offsets();

  // 1e-9 tolerance covers float artefacts when an offset and swing_end share a
  // common irrational (e.g. crawl's r_middle at 1/3 vs 1 - 2/3). A non-zero
  // phase margin already pulls swing_end clear of every offset, but the
  // tolerance still has to hold for a zero margin.
  const float boundary = swing_end_ - 1e-9f;
  float min_first_touchdown = std::numeric_limits<float>::infinity();
  for (const auto& name : LEG_NAMES) {
    const float o = offsets.at(name);
    if (o < boundary) {
      // Initial-swing: lift off at master = 0 from NOMINAL.
      is_initial_swing_[name] = true;
      first_lift_off_master_[name] = 0.0f;
      first_touchdown_master_[name] = swing_end_ - o;
    } else {
      // Initial-stance: grounded until phase = 0 (lift off at master = 1 - o).
      is_initial_swing_[name] = false;
      first_lift_off_master_[name] = 1.0f - o;
      first_touchdown_master_[name] = (1.0f - o) + swing_end_;
    }
    min_first_touchdown =
        std::min(min_first_touchdown, first_touchdown_master_[name]);
  }

  // Smoothstep saturates at the earliest first touchdown.
  smoothstep_window_ = min_first_touchdown;

  master_ = 0.0f;
  v_body_x_ = 0.0f;
  v_body_y_ = 0.0f;
  omega_ = 0.0f;
  foot_position_ = nominal_;
  lift_off_position_ = nominal_;
  for (const auto& name : LEG_NAMES) {
    // Initial-swing legs lift off from NOMINAL at master = 0; initial-stance
    // legs snapshot when they cross INITIAL_STANCE -> INITIAL_SWING.
    has_lifted_off_[name] = is_initial_swing_[name];
  }

  state_ = EngagementState::ENGAGING;
}

std::map<std::string, LegOutput> EngagementController::update(
    float dt, std::pair<float, float> v_cmd_xy, float omega_cmd) {
  if (state_ == EngagementState::IDLE) {
    return emit_nominal_stance();
  }

  // 1) Per-leg planar velocity from the commanded body velocity.
  const auto cmd_leg_v =
      per_leg_planar_velocity(leg_contexts_, v_cmd_xy, omega_cmd);
  float max_cmd_leg_v = 0.0f;
  for (const auto& [name, v] : cmd_leg_v) {
    (void)name;
    max_cmd_leg_v = std::max(max_cmd_leg_v, std::hypot(v.first, v.second));
  }
  const float stance_fraction = 1.0f - swing_end_;
  const float cycle_time =
      derive_cycle_time(max_cmd_leg_v, stride_length_, stance_fraction,
                        min_cycle_time_, max_cycle_time_);
  const float stance_time = cycle_time * stance_fraction;

  // 2) Advance master phase. The ladder runs for exactly one cycle, and it stops
  // one step *short* of the wrap rather than landing on it: at master == 1 every
  // phase folds back to pymod(1 + offset) == offset, so every leg whose offset
  // lies inside the swing window would be re-evaluated as though it were that
  // far into a swing it never started — a teleport straight off the stance it is
  // standing on, and at a zero command (the abandoned-engagement case) straight
  // onto nominal. Ending just below the wrap leaves the phases where the feet
  // actually are; the engine picks the gait clock up from exit_master() and its
  // own lift-off logic starts those swings from the foot's real position.
  bool ladder_finished = false;
  if (cycle_time > 0.0f) {
    const float next = master_ + dt / cycle_time;
    if (next >= 1.0f) {
      ladder_finished = true;
    } else {
      master_ = next;
    }
  }

  // 3) Body velocity envelope.
  float envelope;
  if (smoothstep_window_ > 0.0f && master_ < smoothstep_window_) {
    envelope = smoothstep_env(master_ / smoothstep_window_);
  } else {
    envelope = 1.0f;
  }
  v_body_x_ = v_cmd_xy.first * envelope;
  v_body_y_ = v_cmd_xy.second * envelope;
  omega_ = omega_cmd * envelope;

  // 4) Per-leg planar velocity at the internal body velocity.
  const auto body_leg_v =
      per_leg_planar_velocity(leg_contexts_, {v_body_x_, v_body_y_}, omega_);

  // 5) Per-leg output.
  const auto& offsets = strategy_->phase_offsets().offsets();
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    const float phase = pymod(master_ + offsets.at(name), 1.0f);
    const float first_lift_off = first_lift_off_master_[name];
    const float first_touchdown = first_touchdown_master_[name];

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
        stride.swing_end = swing_end_;
        stride.controller_dt = controller_dt_;
        stride.swing = swing_;
        foot = strategy_->foot_target(phase, stride, leg_contexts_.at(name));
        foot_position_[name] = foot;
      }
      out[name] = LegOutput{foot, phase, in_stance};
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

      const float leg_swing_master = master_ - first_lift_off;
      const float leg_swing_duration_master = first_touchdown - first_lift_off;
      float phase_in_swing = leg_swing_duration_master > 0.0f
                                 ? leg_swing_master / leg_swing_duration_master
                                 : 0.0f;
      phase_in_swing = std::max(0.0f, std::min(phase_in_swing, 1.0f));
      const float leg_swing_time = leg_swing_duration_master * cycle_time;

      // The first swing of an engagement starts from a standing foot while the
      // body velocity is still ramping in, so it departs from rest; it still
      // lands on the live stance velocity and with the configured swing shape,
      // so the leg's first touchdown is as soft as every later one.
      const auto& vb = body_leg_v.at(name);
      const Vec3 foot = swing_arc(phase_in_swing, lift_off_position_[name], aep,
                                  identity_y_sign(nominal), leg_swing_time,
                                  swing_, Vec3::Zero(),
                                  Vec3(-vb.first, -vb.second, 0.0f));
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

  if (ladder_finished) {
    state_ = EngagementState::DONE;
  }

  return out;
}

std::map<std::string, LegOutput> EngagementController::emit_nominal_stance()
    const {
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    out[name] = LegOutput{nominal_.at(name), 0.0f, true};
  }
  return out;
}

}  // namespace hexa::gait
