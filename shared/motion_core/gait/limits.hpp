// Velocity caps derived from the baked gait config — single source of truth.
// Float fork of limits.hpp (plan part 06). Both caps are per-gait: the linear
// cap is the per-leg foot-speed ceiling the gait saturates at, and the angular
// cap is that same ceiling divided by the outermost foot's stance radius — the
// lever arm that turns a body yaw rate into foot speed. Neither is a tunable;
// stride_length, min_swing_time, duty factor and the standing pose set them.
// scale_to_envelope cuts (v_x, v_y, omega_z) so the implied per-leg planar speed
// never exceeds linear_max, which is also what bounds omega_z.
#pragma once

#include <map>
#include <string>
#include <tuple>

#include "gait/types.hpp"

namespace hexa::gait {

// Per-gait linear caps, per-gait angular caps, and per-gait yaw bias. Lookups by
// gait name throw std::out_of_range on unknown names (fail fast).
struct VelocityCaps {
  std::map<std::string, float> linear_max_by_gait;
  std::map<std::string, float> angular_max_by_gait;
  std::map<std::string, float> yaw_bias_by_gait;

  float linear_max(const std::string& gait) const {
    return linear_max_by_gait.at(gait);
  }
  float angular_max(const std::string& gait) const {
    return angular_max_by_gait.at(gait);
  }
  float yaw_bias(const std::string& gait) const {
    return yaw_bias_by_gait.at(gait);
  }
};

// Planar radius of the outermost foot at the standing pose. This is the lever
// arm a body yaw rate acts through, so linear_max / this is the fastest yaw the
// gait can actually lay down. nominal_stance maps leg name to the standing foot
// position; z is ignored. Throws std::invalid_argument if the map is empty or
// every foot sits on the body axis (a degenerate stance has no yaw authority).
float outer_stance_radius(const std::map<std::string, Vec3>& nominal_stance);

// Build per-gait caps from the baked config: config::kGaits carries the linear
// cap and yaw bias, and r_outer (from outer_stance_radius on the standing pose)
// turns the linear cap into the angular one. Mirrors the double
// load_velocity_caps that reads tuning.yaml. The angular cap is never baked — it
// is derived here and in hexa_locomotion's runtime loader through the same
// helper, so the two agree exactly. r_outer is a parameter rather than looked up
// here so this stays free of the IK/engine translation units.
VelocityCaps load_velocity_caps_from_config(float r_outer);

// Cut the velocity triple to fit the gait envelope. Returns (v_x, v_y, omega_z).
// stance_xy maps leg name to the leg's standing foot position (r_x, r_y, r_z);
// r_z is ignored. There is no separate angular clamp: bounding every leg's foot
// speed to linear_max bounds omega_z to linear_max / outer_stance_radius on its
// own.
std::tuple<float, float, float> scale_to_envelope(
    float v_x, float v_y, float omega_z,
    const std::map<std::string, Vec3>& stance_xy, float linear_max,
    float yaw_bias);

}  // namespace hexa::gait
