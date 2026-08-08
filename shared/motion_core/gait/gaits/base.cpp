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
// The swing's horizontal track is a blend between the two ground lines (see
// swing_arc), so the blend weight's derivatives at the ends are exactly the
// foot's departure from ground speed there. All three vanishing means the foot
// leaves and meets the ground travelling at the ground velocity, and pulls away
// from it only as O(t^4) — no segment to time, no height to configure.
//
// The blend runs in *real time*, deliberately. Any reparameterization that
// finishes the horizontal travel early leaves the foot riding the moving
// ground line for the rest of the swing, which in the body frame carries it
// beyond the AEP by ground_speed x time-remaining — workspace this robot's
// front coxa splay does not have. ease7 spreads the travel over the whole
// swing, so that excursion stays a few millimetres.
float ease7_poly(float u) {
  const float u2 = u * u;
  return u2 * u2 * (35.0f + u * (-84.0f + u * (70.0f - 20.0f * u)));
}

// Evaluate from the nearer end: near u = 1 the polynomial's order-100 terms
// cancel down to ~1, leaving float noise big enough to jitter the foot by
// micrometres right at the touchdown seam. Reflecting keeps both tails exact.
float ease7(float u) {
  return u <= 0.5f ? ease7_poly(u) : 1.0f - ease7_poly(1.0f - u);
}

// Unit-amplitude lateral profile: two halves of ease5 joined at the midpoint.
// Peaks where the horizontal blend crosses one half (ease7(0.5) = 0.5), so the
// bulge is symmetric about the middle of the travel, and flat at the ends —
// the sideways bulge has no touchdown speed to deliver, and any end slope
// would hand the foot a lateral velocity at the seams with stance.
float bump(float t) {
  return t < 0.5f ? ease5(2.0f * t) : ease5(2.0f * (1.0f - t));
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
// very nearly pure vertical motion relative to the ground.
//
// The height cap keeps the braking segment from collapsing onto a probe that
// would then have to shed the whole step height at once.
constexpr float kMaxProbeSwingFraction = 0.15f;
constexpr float kMaxProbeHeightFraction = 0.5f;

// How high the foot rides above the blended base, as a function of swing
// progress. The apex is pinned to t = 0.5 — the geometric midpoint of the
// shaped travel, since ease7(0.5) = 0.5 — so the arc is symmetric in space
// however the two halves are eased in time. There is no timing knob in the
// geometry: the split fell out of removing swing_apex_fraction, and the
// lift-off speed is derived below.
//
// Climb: the quintic lift, plus a mirrored Hermite term that hands the foot a
// definite upward speed at t = 0 (zero would creep it off the ground inside a
// single servo step while it still carries weight). The speed is not a knob:
// 4 * clearance / swing_time — the end slope of the parabola through the same
// apex, and provably the largest value for which the climb stays monotone. It
// scales with step_height over swing time, so higher steps and quicker swings
// break contact faster.
//
// Descent: the quintic brake down to `probe_height`, then a straight line at
// exactly `touchdown_velocity` the rest of the way. The braked part carries a
// Hermite term scaled to the shortened segment, so it arrives at the probe
// travelling at exactly the probe's speed — the seam matches in position and
// velocity both.
//
// The probe is the point of the split. A curve that merely *approaches*
// touchdown_velocity only reaches it in the limit at ground level: a fraction
// of a millimetre up it is still descending faster, so a foot that meets the
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
float swing_height(float t, float clearance, float swing_time,
                   float touchdown_velocity, float probe_height) {
  const float liftoff_velocity = 4.0f * clearance / swing_time;
  const float half_time = 0.5f * swing_time;
  if (t < 0.5f) {
    const float u = 2.0f * t;
    return clearance * ease5(u) -
           liftoff_velocity * half_time * hermite_end_slope(1.0f - u);
  }

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

  const float elapsed = (t - 0.5f) * swing_time;
  const float brake_time = half_time - probe_time;
  if (elapsed >= brake_time) {
    return touchdown_velocity * (half_time - elapsed);
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

  // Apex height is measured from the ground the foot is stepping between, so
  // swing_clearance keeps meaning "how high the foot lifts". The blended base
  // under the apex is the endpoint midpoint (ease7(0.5) = 0.5), so on a swing
  // between two heights the lift's amplitude absorbs half the height gap and
  // the arc tops out at the higher end plus the profile's clearance. A profile
  // asking for no lift at all (the reseat landing) gets no shaping either —
  // lifting over half the height gap would swing the foot up off a start it
  // was told to descend from — and rides the plain eased blend instead.
  if (profile.clearance > 0.0f) {
    const float ground_z = std::max(swing_origin[2], target[2]);
    const float clearance =
        std::max(0.0f, ground_z + profile.clearance -
                           0.5f * (swing_origin[2] + target[2]));
    point[2] += swing_height(t, clearance, swing_time,
                             profile.touchdown_velocity,
                             profile.touchdown_probe_height);
  }

  // Lateral bulge. Shares the lift's symmetry rather than its height, so it
  // survives a swing with zero clearance (the pause descent).
  point[1] += (identity_y_sign > 0 ? profile.width : -profile.width) * bump(t);

  return point;
}

}  // namespace hexa::gait
