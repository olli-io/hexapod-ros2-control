#include "gait/gaits/base.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hexa::gait {

std::map<std::string, std::pair<float, float>> per_leg_planar_velocity(
    const std::map<std::string, LegContext>& leg_contexts,
    std::pair<float, float> v_body_xy, float omega_z) {
  std::map<std::string, std::pair<float, float>> out;
  for (const auto& [name, leg] : leg_contexts) {
    // Lever arm is the *foot*, not the hip: the stride is applied at
    // nominal_stance, so a hip lever undershoots the yaw by ~2.4x here.
    const float r_x = leg.nominal_stance[0];
    const float r_y = leg.nominal_stance[1];
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

RadialAxis radial_axis(const LegContext& leg) {
  const float d_x = leg.nominal_stance[0] - leg.mount_xyz[0];
  const float d_y = leg.nominal_stance[1] - leg.mount_xyz[1];
  const float r = std::hypot(d_x, d_y);
  if (r <= 0.0f) {
    return RadialAxis{};
  }
  return RadialAxis{d_x / r, d_y / r, r};
}

float radial_stride_budget(const RadialAxis& axis, float d_x, float d_y,
                           float stride_length_radial) {
  const float unbounded = std::numeric_limits<float>::infinity();
  const float r_outer = axis.tip_reach;
  if (r_outer <= 0.0f || stride_length_radial <= 0.0f) {
    return unbounded;
  }
  // The reach the near end may close to; a budget that reaches the coxa axis is
  // no budget at all.
  const float m = r_outer - 0.5f * stride_length_radial;
  if (m <= 0.0f) {
    return unbounded;
  }
  const float d = std::hypot(d_x, d_y);
  if (d <= 0.0f) {
    return unbounded;
  }
  const float c =
      -std::fabs((d_x * axis.u_x + d_y * axis.u_y) / d);
  const float disc = c * c - 1.0f + (m * m) / (r_outer * r_outer);
  if (disc <= 0.0f) {
    return unbounded;
  }
  // a = 1/2: half the stride either side of nominal.
  return 2.0f * r_outer * (-c - std::sqrt(disc));
}

float effective_stride_length(
    const std::map<std::string, LegContext>& legs,
    const std::map<std::string, std::pair<float, float>>& leg_velocities,
    float stride_length, float stride_length_radial) {
  // The documented disable — it falls out of the arithmetic too, but only to
  // within a square root's last bit.
  if (stride_length_radial >= stride_length) {
    return stride_length;
  }

  float max_leg_v = 0.0f;
  for (const auto& [name, v] : leg_velocities) {
    (void)name;
    max_leg_v = std::max(max_leg_v, std::hypot(v.first, v.second));
  }
  if (max_leg_v <= 0.0f) {
    return stride_length;
  }

  float allowed = stride_length;
  for (const auto& [name, leg] : legs) {
    const auto it = leg_velocities.find(name);
    if (it == leg_velocities.end()) {
      continue;
    }
    const float v_x = it->second.first;
    const float v_y = it->second.second;
    const float speed = std::hypot(v_x, v_y);
    if (speed <= 0.0f) {
      continue;
    }
    const float budget =
        radial_stride_budget(radial_axis(leg), v_x, v_y, stride_length_radial);
    // The leg lays down speed / max_leg_v of the tick's stride.
    allowed = std::min(allowed, budget * (max_leg_v / speed));
  }
  return allowed;
}

float effective_stride_length(const std::map<std::string, LegContext>& legs,
                              std::pair<float, float> v_body_xy, float omega_z,
                              float stride_length, float stride_length_radial) {
  return effective_stride_length(
      legs, per_leg_planar_velocity(legs, v_body_xy, omega_z), stride_length,
      stride_length_radial);
}

std::pair<float, float> ease_outward(float e_x, float e_y, float d_x, float d_y,
                                     float band, float ceiling) {
  const float m = std::hypot(e_x, e_y);
  if (m <= band || ceiling <= band) {
    return {d_x, d_y};
  }
  const float u_x = e_x / m;
  const float u_y = e_y / m;
  const float radial = d_x * u_x + d_y * u_y;
  if (radial <= 0.0f) {
    return {d_x, d_y};  // heading back toward nominal
  }
  const float gain =
      1.0f - ease5(std::min((m - band) / (ceiling - band), 1.0f));
  const float blocked = (1.0f - gain) * radial;
  return {d_x - blocked * u_x, d_y - blocked * u_y};
}

float swing_end_phase(float duty_factor, float margin_fraction) {
  const float nominal = std::max(0.0f, 1.0f - duty_factor);
  return nominal * (1.0f - std::clamp(margin_fraction, 0.0f, 0.5f));
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
// The vertical shaping comes from the SwingProfile, never from the caller.
Vec3 planar(const Vec3& v) { return Vec3(v[0], v[1], 0.0f); }

// One half of the septic smoothstep; ease7 reflects it into the other.
float ease7_poly(float u) {
  const float u2 = u * u;
  return u2 * u2 * (35.0f + u * (-84.0f + u * (70.0f - 20.0f * u)));
}

// Unit-amplitude lateral profile: two halves of ease5 joined at the midpoint.
// Evaluated on the *blend*, not on a clock, so it unwinds with the blend's own
// O(t^4) departure at the seams; on a clock it dragged the foot sideways at
// touchdown under a lateral command.
float bump(float t) {
  return t < 0.5f ? ease5(2.0f * t) : ease5(2.0f * (1.0f - t));
}

// Perspective time warp: monotone, fixes both ends, crosses one half at
// t = apex_time. Feeding it to the horizontal blend puts the swing over the
// spatial midpoint of its travel at the apex without moving the track. The
// seams survive it — ease7's vanishing derivatives vanish through any smooth
// warp — and with apex_time <= 0.5 it only ever slows the tail.
float apex_warp(float t, float apex_time) {
  const float k = apex_time / (1.0f - apex_time);
  return t / (t + k * (1.0f - t));
}

// Quintic-Hermite basis for a prescribed derivative at the *end* of the
// interval. It is <= 0 across [0, 1], so a negative end slope only lifts the
// curve: the climb cannot dip below the lift-off level and needs no clamp.
float hermite_end_slope(float u) {
  return u * u * u * (-4.0f + u * (7.0f - 3.0f * u));
}

// How high the foot rides above the blended base. The climb takes apex_time of
// the swing and the descent the rest.
//
// Climb is the quintic lift plus a mirrored Hermite term giving a definite
// lift-off speed (zero would creep the loaded foot off the ground inside one
// servo step); 2 * clearance / climb_time is the largest monotone climb.
// Descent is a plain quintic, so the foot meets the ground at zero speed.
//
// Heights are measured from the blended base, which is the touchdown level for
// every walking swing.
float swing_height(float t, float clearance, float apex_time) {
  if (t < apex_time) {
    const float u = t / apex_time;
    return clearance * (ease5(u) - 2.0f * hermite_end_slope(1.0f - u));
  }
  return clearance * (1.0f - ease5((t - apex_time) / (1.0f - apex_time)));
}

// Swing progress at which the foot is `height` (< clearance) above the base, on
// the climb or on the descent. Each side is monotone, so bisection; 24 halvings
// put it below float resolution.
float height_crossing(float height, float clearance, float apex_time,
                      bool climb) {
  float lo = climb ? 0.0f : apex_time;
  float hi = climb ? apex_time : 1.0f;
  for (int i = 0; i < 24; ++i) {
    const float mid = 0.5f * (lo + hi);
    ((swing_height(mid, clearance, apex_time) < height) == climb ? lo : hi) =
        mid;
  }
  return climb ? hi : lo;
}

// Measured from the higher of the two ends, so swing_clearance keeps meaning
// "how high the foot lifts" on a swing between two heights. A profile asking
// for no lift gets no shaping either — half a height gap would swing the
// reseat landing up off a start it was told to descend from.
float arc_clearance(const Vec3& swing_origin, const Vec3& target,
                    const SwingProfile& profile) {
  if (profile.clearance <= 0.0f) {
    return 0.0f;
  }
  const float ground_z = std::max(swing_origin[2], target[2]);
  return std::max(0.0f, ground_z + profile.clearance -
                            0.5f * (swing_origin[2] + target[2]));
}

// Swing progress at which the straight lift ends and the straight landing
// starts; {0, 1} when the profile asks for neither.
std::pair<float, float> straight_segments(float clearance,
                                          const SwingProfile& profile) {
  float lift_end = 0.0f;
  float land_start = 1.0f;
  if (clearance > 0.0f) {
    const float cap = kMaxStraightShare * clearance;
    if (profile.lift_height > 0.0f) {
      lift_end = height_crossing(std::min(profile.lift_height, cap), clearance,
                                 profile.apex_time, true);
    }
    if (profile.land_height > 0.0f) {
      land_start = height_crossing(std::min(profile.land_height, cap),
                                   clearance, profile.apex_time, false);
    }
  }
  return {lift_end, land_start};
}
}  // namespace

float swing_land_start(const Vec3& swing_origin, const Vec3& target,
                       const SwingProfile& profile) {
  return straight_segments(arc_clearance(swing_origin, target, profile),
                           profile)
      .second;
}

// Evaluate from the nearer end: near u = 1 the order-100 terms cancel down to
// ~1 and the float noise jitters the foot at the touchdown seam.
float ease7(float u) {
  return u <= 0.5f ? ease7_poly(u) : 1.0f - ease7_poly(1.0f - u);
}

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

  const float t = std::clamp(phase_in_swing, 0.0f, 1.0f);

  const float clearance = arc_clearance(swing_origin, target, profile);
  const float apex_time = profile.apex_time;

  // The blend runs between the straight lift and the straight landing, and
  // still crosses one half at apex_time so the apex stays over the spatial
  // midpoint.
  const auto [lift_end, land_start] = straight_segments(clearance, profile);
  const float span = land_start - lift_end;
  const float s = std::clamp((t - lift_end) / span, 0.0f, 1.0f);
  const float blend = ease7(apex_warp(s, (apex_time - lift_end) / span));

  // The two ground lines: where a foot planted at lift-off would have got to,
  // and where the landing foot would have come from. ease7 pins the swing to the
  // first at t = 0 and the second at t = 1 in position, velocity *and*
  // acceleration — the continuity stance needs at both seams.
  const Vec3 from_liftoff = swing_origin + v_ground_in * (swing_time * t);
  const Vec3 to_touchdown = target - v_ground_out * (swing_time * (1.0f - t));
  Vec3 point = (1.0f - blend) * from_liftoff + blend * to_touchdown;

  if (clearance > 0.0f) {
    point[2] += swing_height(t, clearance, apex_time);
  }

  // Lateral bulge, on the lift's spatial symmetry rather than its height, so it
  // survives a zero-clearance swing (the pause descent).
  point[1] +=
      (identity_y_sign > 0 ? profile.width : -profile.width) * bump(blend);

  return point;
}

}  // namespace hexa::gait
