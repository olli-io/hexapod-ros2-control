// Velocity shaping between the teleop mapping and the gait engine: every tick
// cuts the commanded (v_x, v_y, omega_z) to the active gait's per-leg foot-speed
// envelope, then rate-caps the change toward that target. Recomputes both
// acceleration caps on a gait switch, so the ramp times stay constant across
// gaits, and resets the limiter when the engine leaves {ENGAGING, GAIT}.
#pragma once

#include <map>
#include <string>
#include <tuple>

#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "gait/limits.hpp"
#include "vec3.hpp"

namespace hexa::control {

// Vectorial rate-cap slew: each step() advances (v_x, v_y, omega_z) toward the
// target by at most accel_linear*dt on the planar vector and accel_angular*dt on
// the yaw scalar, snapping sub-tolerance dribble to zero.
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

class Control {
 public:
  // Explicit tuning/geometry (hexa_locomotion's runtime-YAML path):
  // nominal_stance supplies the lever arms the envelope cut works through, and
  // legs + the stride knobs are what the radial budget prices a heading against.
  Control(const ::hexa::config::ControlConfig& control,
          const hexa::gait::VelocityCaps& caps,
          const std::map<std::string, hexa::Vec3>& nominal_stance,
          const std::map<std::string, hexa::gait::LegContext>& legs,
          float stride_length, float stride_length_radial,
          const std::string& default_gait);

  // Cut to the active gait's envelope, then rate-cap toward it. The result feeds
  // straight into Engine::update.
  std::tuple<float, float, float> shape(float v_x, float v_y, float omega_z,
                                        hexa::gait::EngineState engine_state,
                                        float dt);

  // Recomputes both accel caps; the angular one is per-gait too, being the
  // gait's linear cap over the stance radius. Unknown names throw
  // std::out_of_range via the caps.
  void set_gait(const std::string& gait);
  const std::string& active_gait() const { return active_gait_; }

  // Drop the parked pair from the lever arms and the radial budget. Those legs
  // lay nothing down, and theirs are the longest arms on the chassis, so pricing
  // a yaw command against them would cut the stick for a foot that is in the
  // air. Cheap to call every tick: a no-op unless the set actually changed.
  void set_leg_set(hexa::gait::LegSet set);

  // Re-seed everything a preset owns: the caps its stride and swing time
  // produced, the footprint its feet stand on, and the stride the radial budget
  // prices a heading against. Called by the pipeline on every applied preset
  // change, which is the one thing that moves any of them.
  void set_preset(const hexa::gait::VelocityCaps& caps,
                  const std::map<std::string, hexa::Vec3>& nominal_stance,
                  const std::map<std::string, hexa::gait::LegContext>& legs,
                  float stride_length, float stride_length_radial);

  const BodyVelocityLimiter& limiter() const { return limiter_; }

 private:
  // Push the active gait's accel caps into the limiter. Both the gait and the
  // preset move them, so both go through here.
  void rearm_limiter();
  float accel_linear_for(const std::string& gait) const;
  float accel_angular_for(const std::string& gait) const;

  // The share of stride_length the radial budget lets this heading lay down;
  // 1 when nothing binds.
  float stride_ratio_for(float v_x, float v_y, float omega_z) const;

  hexa::gait::VelocityCaps caps_;
  // The full six-leg stance, and the walking subset shape() actually prices
  // against. The full one is kept so a leg-set change can rebuild the subset.
  //
  // Re-seeded by set_preset, which is the only thing that moves the footprint
  // between two declared stances. A reseat that moves the nominal stance WITHIN
  // one preset (a body-height change) is still not reflected here.
  std::map<std::string, hexa::Vec3> stance_xy_all_;
  std::map<std::string, hexa::gait::LegContext> legs_all_;
  std::map<std::string, hexa::Vec3> stance_xy_;
  std::map<std::string, hexa::gait::LegContext> legs_;
  hexa::gait::LegSet leg_set_ = hexa::gait::LegSet::HEXAPOD;
  float stride_length_;
  float stride_length_radial_;
  float vmax_ramp_time_linear_;
  float vmax_ramp_time_angular_;
  std::string active_gait_;
  BodyVelocityLimiter limiter_;
  bool have_engine_state_ = false;
  hexa::gait::EngineState engine_state_ = hexa::gait::EngineState::FOLDED;
};

}  // namespace hexa::control
