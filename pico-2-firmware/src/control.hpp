// Velocity shaping — scale-to-envelope + constant-max-accel slew (plan part 07).
//
// Float fork of hexa_control's body_velocity_limiter.py + control_node.py. Sits
// between the teleop mapping and the gait engine: every tick it cuts the
// commanded (v_x, v_y, omega_z) to the active gait's per-leg foot-speed
// envelope (scale_to_envelope, from the part-06 gait limits) and then rate-caps
// the change toward that target so a stick release ramps smoothly to a stop.
//
// The two ROS nodes collapse into one in-process Control: it owns the limiter,
// the active gait, and the leg-mount geometry, and it recomputes the linear
// acceleration cap on a gait switch (so the ramp time stays constant across
// gaits) and resets the limiter when the engine leaves the walking set
// ({ENGAGING, GAIT}) so each STAND -> ENGAGING starts clean.
#pragma once

#include <map>
#include <string>
#include <tuple>

#include "gait/engine.hpp"   // hexa::gait::EngineState
#include "gait/limits.hpp"   // VelocityCaps, scale_to_envelope
#include "vec3.hpp"

namespace hexa::control {

// Vectorial rate-cap slew on the published body velocity — port of
// body_velocity_limiter.py. Each step() advances (v_x, v_y, omega_z) toward the
// target by at most accel_linear*dt on the planar linear vector and
// accel_angular*dt on the yaw scalar, snapping sub-tolerance dribble to zero.
class BodyVelocityLimiter {
 public:
  BodyVelocityLimiter(float accel_linear, float accel_angular,
                      float snap_tol_linear = 1.0e-4f,
                      float snap_tol_angular = 1.0e-4f);

  float accel_linear() const { return accel_linear_; }
  void set_accel_linear(float value);
  float accel_angular() const { return accel_angular_; }
  void set_accel_angular(float value);

  std::tuple<float, float, float> state() const {
    return {v_x_, v_y_, omega_};
  }
  void reset(float v_x = 0.0f, float v_y = 0.0f, float omega = 0.0f);

  // Advance one tick toward `target` and return the new (v_x, v_y, omega_z).
  std::tuple<float, float, float> step(float tgt_vx, float tgt_vy,
                                       float tgt_omega, float dt);

 private:
  float accel_linear_;
  float accel_angular_;
  float snap_tol_linear_;
  float snap_tol_angular_;
  float v_x_ = 0.0f;
  float v_y_ = 0.0f;
  float omega_ = 0.0f;
};

// Full velocity-shaping stage. Build from the baked config, then call shape()
// once per tick with the raw teleop command and the current engine state.
class Control {
 public:
  Control();

  // Cut to the active gait's envelope, then rate-cap toward it. Resets the
  // limiter on the engine leaving the walking set. Returns the shaped triple to
  // feed straight into Engine::update.
  std::tuple<float, float, float> shape(float v_x, float v_y, float omega_z,
                                        hexa::gait::EngineState engine_state,
                                        float dt);

  // Switch the active gait (recomputes the linear accel cap). No-op if `gait`
  // is already active; unknown names throw std::out_of_range via the caps.
  void set_gait(const std::string& gait);
  const std::string& active_gait() const { return active_gait_; }

  const BodyVelocityLimiter& limiter() const { return limiter_; }

 private:
  float accel_linear_for(const std::string& gait) const;
  float accel_angular() const;

  hexa::gait::VelocityCaps caps_;
  std::map<std::string, hexa::Vec3> leg_mounts_;
  float vmax_ramp_time_linear_;
  float vmax_ramp_time_angular_;
  std::string active_gait_;
  BodyVelocityLimiter limiter_;
  bool have_engine_state_ = false;
  hexa::gait::EngineState engine_state_ = hexa::gait::EngineState::FOLDED;
};

}  // namespace hexa::control
