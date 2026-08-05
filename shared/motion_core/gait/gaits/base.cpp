#include "gait/gaits/base.hpp"

#include <algorithm>
#include <cmath>

namespace hexa::gait {

std::map<std::string, std::pair<float, float>> per_leg_planar_velocity(
    const std::map<std::string, LegContext>& leg_contexts,
    std::pair<float, float> v_body_xy, float omega_z) {
  std::map<std::string, std::pair<float, float>> out;
  for (const auto& [name, leg] : leg_contexts) {
    // Lever arm is the *foot*, not the hip: the stride vector this velocity
    // produces is applied at nominal_stance, so omega x r has to be the
    // tangential velocity there or the realized yaw comes out short by
    // |r_stance| / |r_mount| (~2.4x on this geometry).
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
// Horizontal part of a velocity; the vertical shaping comes from the
// SwingProfile, never from the caller's velocity.
Vec3 planar(const Vec3& v) { return Vec3(v[0], v[1], 0.0f); }

// Septic smoothstep: ease5 plus a vanishing third derivative at both ends.
//
// This is what replaces the old ground-matched ramps. The swing's horizontal
// track is a blend between the two ground lines (see swing_arc), so the blend
// weight's derivatives at the ends are exactly the foot's departure from ground
// speed there. All three vanishing means the foot leaves and meets the ground
// travelling at the ground velocity, and pulls away from it only as O(t^4) —
// the same protection the ramps gave inside their band, but continuous, with no
// segment to time and no height to configure.
float ease7(float u) {
  const float u2 = u * u;
  return u2 * u2 * (35.0f + u * (-84.0f + u * (70.0f - 20.0f * u)));
}

// Unit-amplitude lift profile: 0 at both ends, 1 at `apex`, built from two
// halves of ease5. Slope and curvature vanish at the ends and at the apex, so
// the foot peels off the ground rather than stepping off it, and the two halves
// join smoothly at the top.
float bump(float t, float apex) {
  return t < apex ? ease5(t / apex) : 1.0f - ease5((t - apex) / (1.0f - apex));
}

// Quintic-Hermite basis function for a prescribed derivative at the *end* of
// the interval; zero value, slope and curvature at u = 0, zero value and
// curvature at u = 1, unit slope at u = 1.
//
// It is <= 0 across [0, 1], so scaling it by a negative end slope only ever
// lifts the curve: the descent cannot dip below the touchdown level however
// large touchdown_velocity is, and needs no clamp.
float hermite_end_slope(float u) {
  return u * u * u * (-4.0f + u * (7.0f - 3.0f * u));
}

// Largest share of the whole swing the constant-velocity probe may take, and
// the largest share of the apex height it may stand at.
//
// The time cap is what keeps the probe a *descent*. Its duration is
// probe_height / touchdown_velocity, so a probe asked for at too low a speed
// stops being a short drop onto the target and becomes a long, flat sweep
// towards it — the foot arrives at probe height early and then travels the rest
// of the way to the AEP skimming the ground, which is both useless and exactly
// what a swing is for avoiding. Held to the tail of the swing, the horizontal
// blend has already converged (ease7 is past 0.99 by t = 0.85), so the probe is
// very nearly pure vertical motion.
//
// The height cap keeps the braking segment from collapsing onto a probe that
// would then have to shed the whole step height at once.
constexpr float kMaxProbeSwingFraction = 0.15f;
constexpr float kMaxProbeHeightFraction = 0.5f;

// How high the foot rides above the ground it is stepping between, as a
// function of swing progress. Climb and descent are shaped independently — this
// is the asymmetry: the foot is asked to break contact briskly and to arrive
// slowly.
//
// Climb: the quintic lift, plus a mirrored Hermite term that hands the foot a
// prescribed *upward* speed at t = 0. The reflected argument puts the basis
// function's unit slope at the start of the segment instead of its end, and its
// value and slope both vanish at u = 1, so the apex is untouched.
//
// Descent: the quintic brake down to `probe_height`, then a straight line at
// exactly `touchdown_velocity` the rest of the way. The braked part carries the
// same Hermite term the descent has always had, scaled to the shortened
// segment, so it arrives at the probe travelling at exactly the probe's speed —
// the seam matches in position and velocity both.
//
// The probe is the point of the split. A curve that merely *approaches*
// touchdown_velocity only reaches it in the limit at ground level: a fraction of
// a millimetre up it is still descending far faster, so a foot that meets the
// ground early — which on real servos it always does — lands hard. A straight
// segment lands at the same speed anywhere inside its height.
//
// Heights here are measured from the blended base, which equals the touchdown
// level exactly whenever the two ends of the swing are at the same height. They
// are for every walking swing (the stance integrator never moves a foot in z,
// and the AEP is planar), which is the case this exists for. A swing that steps
// between two different heights — a reseat that is also changing body height —
// keeps its endpoints and its clearance, but the base is still settling under
// the probe, so its last stretch is not held to the probe speed.
float swing_height(float t, float apex, float clearance, float swing_time,
                   float liftoff_velocity, float touchdown_velocity,
                   float probe_height) {
  if (t < apex) {
    const float climb_time = apex * swing_time;
    const float u = t / apex;
    return clearance * ease5(u) -
           liftoff_velocity * climb_time * hermite_end_slope(1.0f - u);
  }

  const float descent_time = (1.0f - apex) * swing_time;
  float probe_time = 0.0f;
  float probe_z = 0.0f;
  if (touchdown_velocity > 0.0f && probe_height > 0.0f) {
    probe_time = std::min(
        std::min(probe_height, kMaxProbeHeightFraction * clearance) /
            touchdown_velocity,
        kMaxProbeSwingFraction * swing_time);
    // Re-derive the height from the time actually granted, so whichever clamp
    // bit, the brake still hands over at exactly the probe's speed.
    probe_z = touchdown_velocity * probe_time;
  }

  const float elapsed = (t - apex) * swing_time;
  const float brake_time = descent_time - probe_time;
  if (elapsed >= brake_time) {
    return touchdown_velocity * (descent_time - elapsed);
  }
  const float w = brake_time > 0.0f ? elapsed / brake_time : 1.0f;
  return probe_z + (clearance - probe_z) * (1.0f - ease5(w)) -
         touchdown_velocity * brake_time * hermite_end_slope(w);
}
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

  const float t = std::clamp(phase_in_swing, 0.0f, 1.0f);
  const float blend = ease7(t);

  // The two ground lines: where a foot planted at lift-off would have got to by
  // now, and where the foot about to touch down would have come from had it been
  // planted all along. Blending between them with ease7 pins the swing to the
  // first at t = 0 and the second at t = 1 in position, velocity *and*
  // acceleration, which is exactly the continuity stance needs at both seams.
  const Vec3 from_liftoff = swing_origin + v_ground_in * (swing_time * t);
  const Vec3 to_touchdown = target - v_ground_out * (swing_time * (1.0f - t));
  Vec3 point = (1.0f - blend) * from_liftoff + blend * to_touchdown;

  // Guard the split so neither half of the arc collapses.
  const float apex_fraction = std::clamp(profile.apex_fraction, 0.05f, 0.95f);
  const float lift = bump(t, apex_fraction);

  // Apex height is measured from the ground the foot is stepping between, so
  // swing_clearance keeps meaning "how high the foot lifts".
  const float ground_z = std::max(swing_origin[2], target[2]);
  const float apex_z = (1.0f - ease7(apex_fraction)) * swing_origin[2] +
                       ease7(apex_fraction) * target[2];
  const float apex_clearance =
      std::max(0.0f, ground_z + profile.clearance - apex_z);

  // Asymmetric by design: a brisk break from the ground, and a descent that
  // brakes early and then probes down the last stretch at a steady, slow speed.
  // See swing_height.
  point[2] += swing_height(t, apex_fraction, apex_clearance, swing_time,
                           profile.liftoff_velocity,
                           profile.touchdown_velocity,
                           profile.touchdown_probe_height);

  // Lateral arch. Shares the lift profile's shape rather than its height, so it
  // survives a swing with zero clearance (the pause descent).
  point[1] += (identity_y_sign > 0 ? profile.width : -profile.width) * lift;

  return point;
}

}  // namespace hexa::gait
