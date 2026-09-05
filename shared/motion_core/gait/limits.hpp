// Velocity caps derived from the gait config. The linear cap is the per-leg
// foot-speed ceiling the gait saturates at; the angular cap is that ceiling
// divided by the outermost foot's stance radius. Neither is a tunable —
// stride_length, min_swing_time, duty factor and the standing pose set them.
#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <tuple>

#include "gait/types.hpp"

namespace hexa::gait {

// Lookups throw std::out_of_range on an unknown gait name (fail fast).
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

// Planar radius of the outermost foot at the standing pose — the lever arm a
// body yaw rate acts through, so linear_max / this is the fastest yaw the gait
// can lay down. Throws std::invalid_argument on a stance with no yaw authority
// (empty, or every foot on the body axis).
float outer_stance_radius(const std::map<std::string, Vec3>& nominal_stance);

// One preset's caps, from the baked table's row for it. The angular cap is
// never baked — hexa_locomotion's runtime loader derives it through this same
// division, so the two agree exactly. r_outer is a parameter so this stays free
// of the IK/engine translation units, and it is the PRESET's: each stands its
// outermost foot at its own radius.
VelocityCaps load_velocity_caps_from_config(std::size_t preset, float r_outer);

// Cut the velocity triple to fit the gait envelope. Returns (v_x, v_y, omega_z).
// No separate angular clamp: bounding every leg's foot speed to linear_max
// bounds omega_z to linear_max / outer_stance_radius on its own.
std::tuple<float, float, float> scale_to_envelope(
    float v_x, float v_y, float omega_z,
    const std::map<std::string, Vec3>& stance_xy, float linear_max,
    float yaw_bias);

}  // namespace hexa::gait
