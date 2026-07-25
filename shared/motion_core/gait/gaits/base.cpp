#include "gait/gaits/base.hpp"

#include <algorithm>
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

float swing_end_phase(float duty_factor, float margin_fraction) {
  const float nominal = std::max(0.0f, 1.0f - duty_factor);
  return nominal * (1.0f - std::clamp(margin_fraction, 0.0f, 0.4f));
}

float derive_cycle_time(float max_leg_v, float stride_length,
                        float stance_fraction, float min_cycle_time,
                        float max_cycle_time) {
  if (max_leg_v <= 0.0f || stance_fraction <= 0.0f) {
    return max_cycle_time;
  }
  const float raw = stride_length / (max_leg_v * stance_fraction);
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

namespace {
// Horizontal part of a velocity; the ramps and the transfer arc take their
// vertical shaping from the SwingProfile, never from the caller's velocity.
Vec3 planar(const Vec3& v) { return Vec3(v[0], v[1], 0.0f); }

// Share of the swing each ground-matched ramp occupies.
//
// This is deliberately fixed here rather than configured. The ramps buy zero
// scrub near the ground, but they are expensive twice over: every second they
// take is a second the transfer arc does not have, and the arc must also cover
// the ground the body travels during them. The two compound, so this share —
// not the ramp's height or speed — is what governs how hard the foot is thrown
// through the air. At a quarter of the swing each, the peak tip speed doubles
// against the same gait with the ramps off; at this value it is within a few
// percent of it.
constexpr float kRampSwingShare = 0.06f;

// Largest share of the apex height the ramps may span. See the clamp site.
constexpr float kMaxRampFraction = 0.10f;
}  // namespace

Vec3 swing_arc(float phase_in_swing, const Vec3& swing_origin,
               const Vec3& target, int identity_y_sign, float swing_time,
               const SwingProfile& profile,
               std::optional<Vec3> origin_ground_velocity,
               std::optional<Vec3> target_ground_velocity) {
  const Vec3 stride = target - swing_origin;
  const Vec3 v_ground_in = planar(
      origin_ground_velocity ? *origin_ground_velocity : (-stride / swing_time));
  const Vec3 v_ground_out = planar(
      target_ground_velocity ? *target_ground_velocity : (-stride / swing_time));

  // The ramps always take the same slice of the swing, so the transfer arc's
  // budget cannot be eaten by a tuning value. The height they span is a share of
  // the apex height; the vertical speed that implies is whatever it has to be.
  float ramp_time = 0.0f;
  float ramp_clearance = 0.0f;
  if (profile.ramp_clearance_fraction > 0.0f && profile.clearance > 0.0f) {
    // Clamped so that every settable value is a safe one. Past roughly this
    // share the band is tall enough that crossing it inside the ramp's slice of
    // the swing demands a vertical speed which, carried into the transfer arc as
    // its entry tangent, throws the foot as hard as an over-long ramp would —
    // the same defect arriving through the vertical channel instead of the
    // horizontal one.
    ramp_clearance =
        std::clamp(profile.ramp_clearance_fraction, 0.0f, kMaxRampFraction) *
        profile.clearance;
    ramp_time = kRampSwingShare * swing_time;
  }

  const float arc_time = swing_time - 2.0f * ramp_time;
  const bool ramped = ramp_time > 0.0f && arc_time > 0.0f;

  Vec3 arc_origin = swing_origin;
  Vec3 arc_target = target;
  Vec3 velocity_in = v_ground_in;
  Vec3 velocity_out = v_ground_out - Vec3(0.0f, 0.0f, profile.touchdown_velocity);
  BezierNodes liftoff_ramp;
  BezierNodes touchdown_ramp;

  if (ramped) {
    liftoff_ramp = generate_liftoff_ramp_control_nodes(
        swing_origin, v_ground_in, ramp_time, ramp_clearance);
    touchdown_ramp = generate_touchdown_ramp_control_nodes(
        target, v_ground_out, ramp_time, ramp_clearance,
        profile.touchdown_velocity);
    arc_origin = liftoff_ramp[4];
    arc_target = touchdown_ramp[0];
    // The transfer arc inherits the ramps' exit / entry velocities, so the joins
    // are continuous in position and velocity without any extra shaping.
    velocity_in = quartic_bezier_dot(liftoff_ramp, 1.0f) / ramp_time;
    velocity_out = quartic_bezier_dot(touchdown_ramp, 0.0f) / ramp_time;
  }

  // Apex height is measured from the ground, not from the transfer arc's
  // endpoints, so swing_clearance keeps meaning "how high the foot lifts".
  const float ground_z = std::max(swing_origin[2], target[2]);
  const float apex_clearance =
      std::max(0.0f, ground_z + profile.clearance -
                         std::max(arc_origin[2], arc_target[2]));

  // Guard the split so neither half of the transfer arc collapses.
  const float apex_fraction = std::clamp(profile.apex_fraction, 0.05f, 0.95f);
  const float ascent_time = apex_fraction * arc_time;
  const float descent_time = arc_time - ascent_time;

  const BezierNodes primary = generate_primary_swing_control_nodes(
      arc_origin, velocity_in, arc_target, apex_clearance, profile.width,
      identity_y_sign, ascent_time);
  const BezierNodes secondary = generate_secondary_swing_control_nodes(
      primary, arc_target, velocity_out, ascent_time, descent_time);

  // Walk the four segments on the shared swing clock.
  float elapsed = std::clamp(phase_in_swing, 0.0f, 1.0f) * swing_time;
  if (ramped) {
    if (elapsed < ramp_time) {
      return quartic_bezier(liftoff_ramp, elapsed / ramp_time);
    }
    elapsed -= ramp_time;
  }
  if (elapsed < ascent_time) {
    return quartic_bezier(primary, elapsed / ascent_time);
  }
  elapsed -= ascent_time;
  if (elapsed < descent_time || !ramped) {
    return quartic_bezier(secondary,
                          std::min(elapsed / descent_time, 1.0f));
  }
  elapsed -= descent_time;
  return quartic_bezier(touchdown_ramp, std::min(elapsed / ramp_time, 1.0f));
}

}  // namespace hexa::gait
