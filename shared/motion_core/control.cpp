#include "control.hpp"

#include <cmath>
#include <stdexcept>

#include "config_generated.hpp"
#include "gait/types.hpp"

namespace hexa::control {

namespace {

float validate_positive(const char* name, float value) {
  if (value <= 0.0f) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
  return value;
}

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
  omega_ = omega;
}

std::tuple<float, float, float> BodyVelocityLimiter::step(float tgt_vx,
                                                          float tgt_vy,
                                                          float tgt_omega,
                                                          float dt) {
  if (dt <= 0.0f) {
    return state();
  }

  const float dx = tgt_vx - v_x_;
  const float dy = tgt_vy - v_y_;
  const float distance = std::hypot(dx, dy);
  const float max_step_lin = accel_linear_ * dt;
  if (distance <= max_step_lin) {
    v_x_ = tgt_vx;
    v_y_ = tgt_vy;
  } else {
    const float scale = max_step_lin / distance;
    v_x_ += scale * dx;
    v_y_ += scale * dy;
  }
  if (std::hypot(v_x_, v_y_) < snap_tol_linear_) {
    v_x_ = 0.0f;
    v_y_ = 0.0f;
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

void Control::set_gait(const std::string& gait) {
  if (gait == active_gait_) {
    return;
  }
  active_gait_ = gait;
  limiter_.set_accel_linear(accel_linear_for(gait));
  limiter_.set_accel_angular(accel_angular_for(gait));
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
