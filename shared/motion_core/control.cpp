#include "control.hpp"

#include <cmath>
#include <stdexcept>

#include "config_generated.hpp"
#include "gait/reversal.hpp"
#include "gait/types.hpp"

namespace hexa::control {

namespace {

float validate_positive(const char* name, float value) {
  if (value <= 0.0f) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
  return value;
}

constexpr float kPi = 3.141592653589793f;
constexpr float kTwoPi = 6.283185307179586f;
constexpr float kPolarEps = 1.0e-6f;

float wrap_pi(float a) { return std::remainder(a, kTwoPi); }

bool is_walking(hexa::gait::EngineState s) {
  return s == hexa::gait::EngineState::ENGAGING ||
         s == hexa::gait::EngineState::GAIT;
}

}  // namespace

BodyVelocityLimiter::BodyVelocityLimiter(float accel_linear, float accel_angular,
                                         float snap_tol_linear,
                                         float snap_tol_angular)
    : accel_linear_(validate_positive("accel_linear", accel_linear)),
      accel_angular_(validate_positive("accel_angular", accel_angular)),
      snap_tol_linear_(snap_tol_linear),
      snap_tol_angular_(snap_tol_angular) {
  if (snap_tol_linear < 0.0f || snap_tol_angular < 0.0f) {
    throw std::invalid_argument("snap_tol_* must be non-negative");
  }
}

void BodyVelocityLimiter::set_accel_linear(float value) {
  accel_linear_ = validate_positive("accel_linear", value);
}

void BodyVelocityLimiter::set_accel_angular(float value) {
  accel_angular_ = validate_positive("accel_angular", value);
}

void BodyVelocityLimiter::reset(float v_x, float v_y, float omega) {
  v_x_ = v_x;
  v_y_ = v_y;
  speed_ = std::hypot(v_x, v_y);
  heading_ = speed_ > kPolarEps ? std::atan2(v_y, v_x) : heading_;
  omega_ = omega;
}

std::tuple<float, float, float> BodyVelocityLimiter::step(float tgt_vx,
                                                          float tgt_vy,
                                                          float tgt_omega,
                                                          float dt) {
  if (dt <= 0.0f) {
    return state();
  }

  const float tgt_speed = std::hypot(tgt_vx, tgt_vy);
  // Neither end of a zero speed has a heading: from rest adopt the target's,
  // toward rest keep our own, so a stop retracts along its line.
  if (speed_ <= kPolarEps && tgt_speed > kPolarEps) {
    heading_ = std::atan2(tgt_vy, tgt_vx);
  }
  const float tgt_heading =
      tgt_speed > kPolarEps ? std::atan2(tgt_vy, tgt_vx) : heading_;
  float signed_tgt = tgt_speed;
  float err = wrap_pi(tgt_heading - heading_);
  // A reversal passes through zero rather than swing round at speed. Anything
  // short of one turns: at 90 degrees stick noise would pick a stop or a turn.
  if (std::cos(err) < hexa::gait::kReversalCos) {
    signed_tgt = -tgt_speed;
    err = wrap_pi(err - kPi);
  }

  // Radial and arc-length components of the step, capped as one vector.
  const float d_speed = signed_tgt - speed_;
  const float d_arc = speed_ * err;
  const float distance = std::hypot(d_speed, d_arc);
  const float max_step_lin = accel_linear_ * dt;
  if (distance <= max_step_lin) {
    v_x_ = tgt_vx;
    v_y_ = tgt_vy;
    speed_ = tgt_speed;
    heading_ = tgt_heading;
  } else {
    const float scale = max_step_lin / distance;
    speed_ += scale * d_speed;
    heading_ = wrap_pi(heading_ + scale * err);
    v_x_ = speed_ * std::cos(heading_);
    v_y_ = speed_ * std::sin(heading_);
    // Past zero: fold the sign into the heading. Same point, canonical state.
    if (speed_ < 0.0f) {
      speed_ = -speed_;
      heading_ = wrap_pi(heading_ + kPi);
    }
  }
  if (speed_ < snap_tol_linear_) {
    v_x_ = 0.0f;
    v_y_ = 0.0f;
    speed_ = 0.0f;
  }

  const float d_omega = tgt_omega - omega_;
  const float max_step_ang = accel_angular_ * dt;
  if (std::fabs(d_omega) <= max_step_ang) {
    omega_ = tgt_omega;
  } else {
    omega_ += std::copysign(max_step_ang, d_omega);
  }
  if (std::fabs(omega_) < snap_tol_angular_) {
    omega_ = 0.0f;
  }

  return state();
}

Control::Control(const ::hexa::config::ControlConfig& control,
                 const hexa::gait::VelocityCaps& caps,
                 const std::map<std::string, hexa::Vec3>& nominal_stance,
                 const std::map<std::string, hexa::gait::LegContext>& legs,
                 float stride_length, float stride_length_radial,
                 const std::string& default_gait)
    : caps_(caps),
      stance_xy_all_(nominal_stance),
      legs_all_(legs),
      stance_xy_(nominal_stance),
      legs_(legs),
      stride_length_(stride_length),
      stride_length_radial_(stride_length_radial),
      vmax_ramp_time_linear_(control.vmax_ramp_time_linear),
      vmax_ramp_time_angular_(control.vmax_ramp_time_angular),
      active_gait_(default_gait),
      limiter_(accel_linear_for(active_gait_),
               accel_angular_for(active_gait_), control.snap_tol_linear,
               control.snap_tol_angular) {}

float Control::accel_linear_for(const std::string& gait) const {
  return caps_.linear_max(gait) / vmax_ramp_time_linear_;
}

float Control::accel_angular_for(const std::string& gait) const {
  return caps_.angular_max(gait) / vmax_ramp_time_angular_;
}

void Control::rearm_limiter() {
  limiter_.set_accel_linear(accel_linear_for(active_gait_));
  limiter_.set_accel_angular(accel_angular_for(active_gait_));
}

void Control::set_gait(const std::string& gait) {
  if (gait == active_gait_) {
    return;
  }
  active_gait_ = gait;
  rearm_limiter();
}

namespace {

// The walking subset of a full-stance map, for whichever legs `set` walks.
template <typename Map>
Map walking_subset(const Map& all, hexa::gait::LegSet set) {
  Map out;
  for (const auto& [name, v] : all) {
    if (!hexa::gait::leg_is_parked(set, name)) {
      out[name] = v;
    }
  }
  return out;
}

}  // namespace

void Control::set_leg_set(hexa::gait::LegSet set) {
  if (set == leg_set_) {
    return;
  }
  leg_set_ = set;
  stance_xy_ = walking_subset(stance_xy_all_, leg_set_);
  legs_ = walking_subset(legs_all_, leg_set_);
}

void Control::set_preset(
    const hexa::gait::VelocityCaps& caps,
    const std::map<std::string, hexa::Vec3>& nominal_stance,
    const std::map<std::string, hexa::gait::LegContext>& legs,
    float stride_length, float stride_length_radial) {
  caps_ = caps;
  stance_xy_all_ = nominal_stance;
  legs_all_ = legs;
  stance_xy_ = walking_subset(stance_xy_all_, leg_set_);
  legs_ = walking_subset(legs_all_, leg_set_);
  stride_length_ = stride_length;
  stride_length_radial_ = stride_length_radial;
  // The accel caps are the new preset's linear/angular maxima over the ramp
  // times, so the limiter is re-armed even though the gait has not changed.
  rearm_limiter();
}

std::tuple<float, float, float> Control::shape(
    float v_x, float v_y, float omega_z, hexa::gait::EngineState engine_state,
    float dt) {
  // Reset on leaving the walking set, so each STAND -> ENGAGING starts clean.
  if (!have_engine_state_ || engine_state != engine_state_) {
    const bool was_walking = have_engine_state_ && is_walking(engine_state_);
    const bool now_walking = is_walking(engine_state);
    if (was_walking && !now_walking) {
      limiter_.reset();
    }
    engine_state_ = engine_state;
    have_engine_state_ = true;
  }

  // Derate the linear cap by what the radial budget lets this heading lay down,
  // or the stick tops out at the isotropic cap while the engine shortens the
  // stride underneath it and the planted feet scrub.
  //
  // Read off the *requested* command deliberately: scale_to_envelope cuts the
  // linear and angular parts by different factors, so on a mixed command the
  // engine's own effective stride differs slightly. The engine's governs where
  // feet go; this one only decides where the stick runs out.
  const float ratio = stride_ratio_for(v_x, v_y, omega_z);
  auto [sx, sy, sw] = hexa::gait::scale_to_envelope(
      v_x, v_y, omega_z, stance_xy_, caps_.linear_max(active_gait_) * ratio,
      caps_.yaw_bias(active_gait_));
  return limiter_.step(sx, sy, sw, dt);
}

float Control::stride_ratio_for(float v_x, float v_y, float omega_z) const {
  if (stride_length_ <= 0.0f || legs_.empty()) {
    return 1.0f;
  }
  return hexa::gait::effective_stride_length(legs_, {v_x, v_y}, omega_z,
                                             stride_length_,
                                             stride_length_radial_) /
         stride_length_;
}

}  // namespace hexa::control
