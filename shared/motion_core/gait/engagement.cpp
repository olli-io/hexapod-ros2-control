#include "gait/engagement.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace hexa::gait {

namespace {
float smoothstep_env(float tau) {
  if (tau <= 0.0f) {
    return 0.0f;
  }
  if (tau >= 1.0f) {
    return 1.0f;
  }
  return tau * tau * (3.0f - 2.0f * tau);
}

// How far back down the phase circle the SHIFTING lead-in starts the master
// clock. Half a cycle is runway for any support-shift lead worth having, and
// every leg's phase reaches the lead-in the same way it does mid-walk: by the
// clock coming round to it.
constexpr float kShiftSpan = 0.5f;

// Share of shift_time spent winding the clock up to the first lift-off; the rest
// holds it there while the body arrives. Same split, and the same reason, as the
// reseat ladder's pre-lift hold.
constexpr float kShiftRampFraction = 0.5f;

// The master the wind-up stops at: a whisker short of the lift-off it is walking
// toward, so no leg reads as airborne while every foot is still planted.
constexpr float kShiftMasterEpsilon = 1e-4f;
}  // namespace

EngagementController::EngagementController(
    std::map<std::string, Vec3> nominal_stance, float stride_length,
    float stride_length_radial, float min_cycle_time, float max_cycle_time,
    float duty_factor, float swing_phase_margin, const SwingProfile& swing,
    float controller_dt, float shift_time)
    : stride_length_(stride_length),
      stride_length_radial_(stride_length_radial),
      min_cycle_time_(min_cycle_time),
      max_cycle_time_(max_cycle_time),
      duty_factor_(duty_factor),
      swing_end_(swing_end_phase(duty_factor, swing_phase_margin)),
      swing_(swing),
      controller_dt_(controller_dt),
      shift_time_(shift_time) {
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
  // Deliberately NOT require_all_legs: the caller passes only the legs that
  // walk, so a parked pair is simply absent and every loop below skips it. The
  // engine overlays those legs onto the output.
  if (leg_contexts.empty()) {
    throw std::invalid_argument("leg_contexts must not be empty");
  }
  if (strategy.duty_factor() != duty_factor_) {
    throw std::invalid_argument(
        "strategy duty_factor does not match controller duty_factor");
  }

  strategy_ = &strategy;
  leg_contexts_ = leg_contexts;
  const auto& offsets = strategy.phase_offsets().offsets();

  // The tolerance covers float artefacts when an offset and swing_end share a
  // common irrational (crawl's r_middle at 1/3 vs 1 - 2/3) at a zero margin.
  const float boundary = swing_end_ - 1e-9f;
  float min_first_touchdown = std::numeric_limits<float>::infinity();
  float max_first_lift_off = 0.0f;
  for (const auto& name : LEG_NAMES) {
    if (!walks(name)) {
      continue;
    }
    const float o = offsets.at(name);
    if (o < boundary) {
      // Lift off at master = 0 from nominal.
      is_initial_swing_[name] = true;
      first_lift_off_master_[name] = 0.0f;
      first_touchdown_master_[name] = swing_end_ - o;
    } else {
      // Grounded until phase = 0.
      is_initial_swing_[name] = false;
      first_lift_off_master_[name] = 1.0f - o;
      first_touchdown_master_[name] = (1.0f - o) + swing_end_;
    }
    min_first_touchdown =
        std::min(min_first_touchdown, first_touchdown_master_[name]);
    max_first_lift_off =
        std::max(max_first_lift_off, first_lift_off_master_[name]);
  }

  // The body may not run out from under a leg that has not stepped yet. Before
  // its first lift-off a waiting foot does nothing but integrate the body
  // velocity — there is no swing coming to put it back — so the distance it
  // drags is the envelope's integral times one cycle of travel. Hold that to the
  // half stride the walk's own stance band rides, and the ramp takes whatever
  // window that needs.
  //
  // Travel over a cycle is stride / (1 - swing_end), so in units of the master
  // the budget is 0.5 * (1 - swing_end). Below the smoothstep's own window the
  // integral to L is L - W/2, which gives W directly. The other branch is
  // unreachable: W > L needs L > 1 - swing_end, and the last leg to lift is the
  // one with the smallest offset at or past swing_end, so L never exceeds it.
  //
  // Tripod lands exactly on min_first_touchdown here, which is why a walk that
  // swings its legs in two halves never needed this. A creep waits three
  // quarters of a cycle for its last leg and drags it most of a stride past the
  // band, which on four feet is the polygon the body is standing in.
  const float band_cycles = 0.5f * (1.0f - swing_end_);
  const float drift_window = 2.0f * (max_first_lift_off - band_cycles);
  smoothstep_window_ = std::max(min_first_touchdown, drift_window);

  master_ = 0.0f;
  t_in_shift_ = 0.0f;
  v_body_x_ = 0.0f;
  v_body_y_ = 0.0f;
  omega_ = 0.0f;
  foot_position_ = nominal_;
  lift_off_position_ = nominal_;
  for (const auto& name : LEG_NAMES) {
    // Initial-stance legs snapshot instead when they cross into swing.
    has_lifted_off_[name] = is_initial_swing_[name];
  }

  state_ = shift_time_ > 0.0f ? EngagementState::SHIFTING
                              : EngagementState::ENGAGING;
}

// The lead-in. Every walking foot stays on its nominal stance and the master
// clock winds from -kShiftSpan up to the first lift-off, so the phases the
// posture stack reads are the ones the walk itself is about to produce — a leg
// approaching lift-off, and the support shift carrying the body off it. Feet do
// not move: the body-velocity envelope has not opened yet, so there is nothing
// to integrate.
std::map<std::string, LegOutput> EngagementController::tick_shift(float dt) {
  t_in_shift_ += dt;
  const float ramp = kShiftRampFraction * shift_time_;
  const float wound = ramp > 0.0f ? std::min(1.0f, t_in_shift_ / ramp) : 1.0f;
  master_ = -kShiftSpan * (1.0f - wound) - kShiftMasterEpsilon;

  const auto& offsets = strategy_->phase_offsets().offsets();
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    if (!walks(name)) {
      continue;
    }
    out[name] = LegOutput{foot_position_.at(name),
                          pymod(master_ + offsets.at(name), 1.0f), true};
  }

  if (t_in_shift_ >= shift_time_) {
    master_ = 0.0f;
    state_ = EngagementState::ENGAGING;
  }
  return out;
}

std::map<std::string, LegOutput> EngagementController::update(
    float dt, std::pair<float, float> v_cmd_xy, float omega_cmd) {
  if (state_ == EngagementState::IDLE) {
    return emit_nominal_stance();
  }
  if (state_ == EngagementState::SHIFTING) {
    return tick_shift(dt);
  }

  const auto cmd_leg_v =
      per_leg_planar_velocity(leg_contexts_, v_cmd_xy, omega_cmd);
  float max_cmd_leg_v = 0.0f;
  for (const auto& [name, v] : cmd_leg_v) {
    (void)name;
    max_cmd_leg_v = std::max(max_cmd_leg_v, std::hypot(v.first, v.second));
  }
  const float stance_fraction = 1.0f - swing_end_;
  // The same stride the walk it hands over to will lay down; engaging on the
  // full stride would hand the gait feet already outside its band.
  const float stride_length = effective_stride_length(
      leg_contexts_, cmd_leg_v, stride_length_, stride_length_radial_);
  SwingProfile swing = swing_;
  swing.ride_headroom = kStanceExcursionGrace * 0.5f * stride_length;
  const float cycle_time =
      derive_cycle_time(max_cmd_leg_v, stride_length, stance_fraction,
                        min_cycle_time_, max_cycle_time_);
  const float stance_time = cycle_time * stance_fraction;

  // One cycle, stopping one step *short* of the wrap: at master == 1 every phase
  // folds back to its own offset, teleporting each leg inside the swing window
  // off the stance it is standing on. Ending below the wrap leaves the phases
  // where the feet are, and the engine picks the clock up from exit_master().
  bool ladder_finished = false;
  if (cycle_time > 0.0f) {
    const float next = master_ + dt / cycle_time;
    if (next >= 1.0f) {
      ladder_finished = true;
    } else {
      master_ = next;
    }
  }

  float envelope;
  if (smoothstep_window_ > 0.0f && master_ < smoothstep_window_) {
    envelope = smoothstep_env(master_ / smoothstep_window_);
  } else {
    envelope = 1.0f;
  }
  v_body_x_ = v_cmd_xy.first * envelope;
  v_body_y_ = v_cmd_xy.second * envelope;
  omega_ = omega_cmd * envelope;

  const auto body_leg_v =
      per_leg_planar_velocity(leg_contexts_, {v_body_x_, v_body_y_}, omega_);

  const auto& offsets = strategy_->phase_offsets().offsets();
  std::map<std::string, LegOutput> out;
  for (const auto& name : LEG_NAMES) {
    if (!walks(name)) {
      continue;
    }
    const float phase = pymod(master_ + offsets.at(name), 1.0f);
    const float first_lift_off = first_lift_off_master_[name];
    const float first_touchdown = first_touchdown_master_[name];

    if (master_ >= first_touchdown) {
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
            stride_vector(v.first, v.second, stance_time, stride_length);
        StrideParams stride;
        stride.stride_vector = stride_vec;
        stride.cycle_time = cycle_time;
        stride.swing_end = swing_end_;
        stride.controller_dt = controller_dt_;
        stride.swing = swing;
        foot = strategy_->foot_target(phase, stride, leg_contexts_.at(name));
        foot_position_[name] = foot;
      }
      out[name] = LegOutput{foot, phase, in_stance};
    } else if (master_ >= first_lift_off) {
      // Arc from the lift-off snapshot to the live AEP.
      if (!has_lifted_off_[name]) {
        lift_off_position_[name] = foot_position_[name];
        has_lifted_off_[name] = true;
      }

      const auto& vc = cmd_leg_v.at(name);
      const Vec3 stride_vec =
          stride_vector(vc.first, vc.second, stance_time, stride_length);
      const Vec3 nominal = nominal_[name];
      const Vec3 aep = live_aep(nominal, stride_vec);

      const float leg_swing_master = master_ - first_lift_off;
      const float leg_swing_duration_master = first_touchdown - first_lift_off;
      float phase_in_swing = leg_swing_duration_master > 0.0f
                                 ? leg_swing_master / leg_swing_duration_master
                                 : 0.0f;
      phase_in_swing = std::max(0.0f, std::min(phase_in_swing, 1.0f));
      const float leg_swing_time = leg_swing_duration_master * cycle_time;

      // Departs from rest (the body velocity is still ramping in) but lands on
      // the live stance velocity, so this touchdown is as soft as every later one.
      const auto& vb = body_leg_v.at(name);
      const Vec3 foot = swing_arc(phase_in_swing, lift_off_position_[name], aep,
                                  identity_y_sign(nominal), leg_swing_time,
                                  swing, Vec3::Zero(),
                                  Vec3(-vb.first, -vb.second, 0.0f));
      foot_position_[name] = foot;
      out[name] = LegOutput{foot, phase, false};
    } else {
      // Integrate the internal body velocity.
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

bool EngagementController::walks(const std::string& name) const {
  return leg_contexts_.find(name) != leg_contexts_.end();
}

}  // namespace hexa::gait
