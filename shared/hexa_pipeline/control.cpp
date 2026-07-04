#include "control.hpp"

#include <cmath>
#include <stdexcept>

#include "config_generated.hpp"  // kControl, kDefaultGait, kLegSpecs
#include "gait/types.hpp"        // LEG_NAMES

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

  // Linear pair: vectorial slew capped at accel_linear * dt.
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

  // Angular: scalar slew capped at accel_angular * dt.
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
                 const std::array<::hexa::config::LegSpec, hexa::kNumLegs>&
                     leg_specs,
                 const std::string& default_gait)
    : caps_(caps),
      vmax_ramp_time_linear_(control.vmax_ramp_time_linear),
      vmax_ramp_time_angular_(control.vmax_ramp_time_angular),
      active_gait_(default_gait),
      limiter_(accel_linear_for(active_gait_), accel_angular(),
               control.snap_tol_linear, control.snap_tol_angular) {
  for (int i = 0; i < hexa::kNumLegs; ++i) {
    leg_mounts_[hexa::gait::LEG_NAMES[static_cast<std::size_t>(i)]] =
        leg_specs[static_cast<std::size_t>(i)].mount_xyz;
  }
}

// tuning.yaml's control_node default_gait is kept aligned with teleop's
// kDefaultGait (both "tripod") and the engine's make_default_engine default.
Control::Control()
    : Control(::hexa::config::kControl,
              hexa::gait::load_velocity_caps_from_config(),
              ::hexa::config::kLegSpecs,
              std::string(::hexa::config::kDefaultGait)) {}

float Control::accel_linear_for(const std::string& gait) const {
  return caps_.linear_max(gait) / vmax_ramp_time_linear_;
}

float Control::accel_angular() const {
  return caps_.angular_max / vmax_ramp_time_angular_;
}

void Control::set_gait(const std::string& gait) {
  if (gait == active_gait_) {
    return;
  }
  active_gait_ = gait;
  limiter_.set_accel_linear(accel_linear_for(gait));
}

std::tuple<float, float, float> Control::shape(
    float v_x, float v_y, float omega_z, hexa::gait::EngineState engine_state,
    float dt) {
  // Reset the limiter on the engine leaving the walking set so each STAND ->
  // ENGAGING starts clean (mirrors control_node's /gait/state edge handling).
  if (!have_engine_state_ || engine_state != engine_state_) {
    const bool was_walking = have_engine_state_ && is_walking(engine_state_);
    const bool now_walking = is_walking(engine_state);
    if (was_walking && !now_walking) {
      limiter_.reset();
    }
    engine_state_ = engine_state;
    have_engine_state_ = true;
  }

  auto [sx, sy, sw] = hexa::gait::scale_to_envelope(
      v_x, v_y, omega_z, leg_mounts_, caps_.linear_max(active_gait_),
      caps_.angular_max, caps_.yaw_bias(active_gait_));
  return limiter_.step(sx, sy, sw, dt);
}

}  // namespace hexa::control
