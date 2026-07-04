#include "gait/gaits/base.hpp"

#include <cmath>

#include "gait/trajectory.hpp"

namespace hexa::gait {

std::map<std::string, std::pair<float, float>> per_leg_planar_velocity(
    const std::map<std::string, LegContext>& leg_contexts,
    std::pair<float, float> v_body_xy, float omega_z) {
  std::map<std::string, std::pair<float, float>> out;
  for (const auto& [name, leg] : leg_contexts) {
    const float r_x = leg.mount_xyz[0];
    const float r_y = leg.mount_xyz[1];
    const float v_x = v_body_xy.first - omega_z * r_y;
    const float v_y = v_body_xy.second + omega_z * r_x;
    out[name] = {v_x, v_y};
  }
  return out;
}

Vec3 stride_vector(float v_x, float v_y, float stance_time,
                   float stride_length) {
  float sx = v_x * stance_time;
  float sy = v_y * stance_time;
  const float magnitude = std::hypot(sx, sy);
  if (magnitude > stride_length && magnitude > 0.0f) {
    const float scale = stride_length / magnitude;
    sx *= scale;
    sy *= scale;
  }
  return Vec3(sx, sy, 0.0f);
}

float derive_cycle_time(float max_leg_v, float stride_length, float duty_factor,
                        float min_cycle_time, float max_cycle_time) {
  if (max_leg_v <= 0.0f) {
    return max_cycle_time;
  }
  const float raw = stride_length / (max_leg_v * duty_factor);
  if (raw < min_cycle_time) {
    return min_cycle_time;
  }
  if (raw > max_cycle_time) {
    return max_cycle_time;
  }
  return raw;
}

Vec3 live_aep(const Vec3& nominal, const Vec3& stride_vec) {
  return nominal + 0.5f * stride_vec;
}

int identity_y_sign(const Vec3& nominal_stance) {
  return nominal_stance[1] > 0.0f ? 1 : -1;
}

Vec3 swing_arc(float phase_in_swing, const Vec3& swing_origin,
               const Vec3& target, float swing_clearance, float swing_width,
               int identity_y_sign, float swing_time, float controller_dt,
               std::optional<Vec3> swing_origin_velocity,
               std::optional<Vec3> swing_target_velocity) {
  const Vec3 stride = target - swing_origin;

  const Vec3 velocity_in =
      swing_origin_velocity ? *swing_origin_velocity : (-stride / swing_time);

  // Synthesise an equivalent stride_vector so any caller-supplied target
  // velocity is honoured without touching the lower-level node generator.
  const Vec3 secondary_stride =
      swing_target_velocity ? (-(*swing_target_velocity) * swing_time) : stride;

  const float swing_delta_t = controller_dt / swing_time;
  const float stance_delta_t = swing_delta_t;  // rest-to-rest symmetric join

  const BezierNodes primary = generate_primary_swing_control_nodes(
      swing_origin, velocity_in, target, swing_clearance, swing_width,
      identity_y_sign, controller_dt, swing_delta_t);
  const BezierNodes secondary = generate_secondary_swing_control_nodes(
      primary, target, secondary_stride, controller_dt, swing_delta_t,
      stance_delta_t);

  if (phase_in_swing < 0.5f) {
    const float local = phase_in_swing / 0.5f;
    return quartic_bezier(primary, local);
  }
  const float local = (phase_in_swing - 0.5f) / 0.5f;
  return quartic_bezier(secondary, local);
}

}  // namespace hexa::gait
