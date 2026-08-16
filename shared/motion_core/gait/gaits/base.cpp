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
  // The reach the near end of the stride may close to. A budget large enough to
  // pull the foot onto the coxa axis is no budget at all.
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
  // a = 1/2: the stride is laid down as half of itself either side of nominal.
  return 2.0f * r_outer * (-c - std::sqrt(disc));
}

float effective_stride_length(
    const std::map<std::string, LegContext>& legs,
    const std::map<std::string, std::pair<float, float>>& leg_velocities,
    float stride_length, float stride_length_radial) {
  // The documented disable. It also falls out of the arithmetic, but only to
  // within a square root's last bit, and this is a value people will set.
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
    // The leg lays down speed / max_leg_v of the tick's stride, so the stride
    // its own budget affords is that much longer.
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

// One half of the septic smoothstep; ease7 reflects it into the other.
//
// Where the vanishing third derivative earns its keep in a swing: the
// horizontal track is a blend between the two ground lines (see swing_arc), so
// the blend weight's derivatives at the ends are exactly the foot's departure
// from ground speed there. All three vanishing means the foot leaves and meets
// the ground travelling at the ground velocity, and pulls away from it only as
// O(t^4) — no segment to time, no height to configure.
float ease7_poly(float u) {
  const float u2 = u * u;
  return u2 * u2 * (35.0f + u * (-84.0f + u * (70.0f - 20.0f * u)));
}

// Unit-amplitude lateral profile: two halves of ease5 joined at the midpoint.
// Evaluated on the *blend*, not on a clock, so the bulge is a pure function of
// along-track progress: it peaks at half-travel, and — via the chain rule —
// its unwind rate carries the blend's own O(t^4) departure from ground speed
// at both seams. On a clock the unwind ran through the whole descent, and
// under a lateral command (bulge collinear with the travel) it added to the
// blend's residual on one side of the body: that middle foot skated sideways
// at ~0.2 m/s millimetres off the floor, dragging its tip at every touchdown.
float bump(float t) {
  return t < 0.5f ? ease5(2.0f * t) : ease5(2.0f * (1.0f - t));
}

// Perspective time warp: monotone, C-infinity, fixes both ends, and crosses
// one half at t = apex_time. Feeding the horizontal blend warped time makes
// the swing pass the spatial midpoint of its travel at the apex, wherever the
// probe put the apex in *time* — the track through space never changes, only
// the schedule along it. The chain rule keeps the seams intact: ease7's first
// three derivatives vanish at the ends, so they still vanish through any
// smooth warp. It runs on the travel clock, so apex_time arrives divided by
// travel_end: exactly 0.5 — the identity — when the ride takes the whole
// probe, below it when the headroom grants less. apex_time <= 0.5 either way,
// so the warp only ever *slows* the tail (w'(1) = apex_time / (1 - apex_time)
// <= 1), which shrinks the body-frame excursion beyond the AEP that the
// blend's span exists to bound.
float apex_warp(float t, float apex_time) {
  const float k = apex_time / (1.0f - apex_time);
  return t / (t + k * (1.0f - t));
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

// Largest share of the apex height the constant-velocity probe may stand at.
// Keeps the braking segment from collapsing onto a probe that would then have
// to shed the whole step height at once. The share-of-swing cap is
// kMaxProbeFraction (base.hpp), on the configured fraction itself.
constexpr float kMaxProbeHeightFraction = 0.5f;

// The probe's duration in seconds, after its caps: the configured share of
// the swing (hard-capped at kMaxProbeFraction) and the height cap above.
float granted_probe_time(float clearance, float swing_time,
                         float touchdown_velocity, float probe_fraction) {
  if (touchdown_velocity <= 0.0f || probe_fraction <= 0.0f ||
      clearance <= 0.0f) {
    return 0.0f;
  }
  return std::min(std::min(probe_fraction, kMaxProbeFraction) * swing_time,
                  kMaxProbeHeightFraction * clearance / touchdown_velocity);
}

// How long before touchdown the horizontal travel may finish, leaving the
// foot riding the touchdown ground line: world-frame stationary over its
// landing point while the probe descends, so an early contact anywhere the
// ride covers lands without horizontal slip.
//
// The grant is the lesser of two meters — and their product is probe_time^2,
// so it can never exceed the probe:
//
//  - headroom / speed. The parked foot sits beyond the touchdown target by
//    ground_speed x ride_time in the body frame, and profile.ride_headroom is
//    the most overshoot the stance envelope affords.
//  - speed x probe_time^2 / headroom. The ride is granted in proportion to
//    the slip it prevents, which also vanishes with the ground speed. This
//    keeps slow swings on the un-ridden schedule: granted the whole probe,
//    they compress their travel into a mid-swing peak the walking yardstick
//    never reaches (the settle's swings lift at an eased-out crawl and were
//    the first to show it), and it makes zero speed the smooth limit rather
//    than a special case — every rest-to-rest ladder keeps the old arc
//    exactly.
//
// Both meters read the *faster* of the swing's two seam speeds. The origin
// speed is latched for the whole swing, so under a settle — the live end
// easing to zero beneath an airborne foot — the grant is constant and the
// travel clock never shifts under the foot; and the live end still bounds the
// overshoot at <= headroom whichever branch binds. Gated on the probe:
// without one the arc lands at the very end of the swing, where the blend's
// own tail already meets the ground track.
float granted_ride_time(float ride_headroom, float origin_speed,
                        float target_speed, float probe_time) {
  if (ride_headroom <= 0.0f || probe_time <= 0.0f) {
    return 0.0f;
  }
  const float speed = std::max(origin_speed, target_speed);
  if (speed <= 0.0f) {
    return 0.0f;
  }
  return std::min(speed * probe_time * probe_time / ride_headroom,
                  ride_headroom / speed);
}

// How high the foot rides above the blended base, as a function of swing
// progress. There is no timing knob in the geometry: the probe takes its
// share off the tail of the swing, and the climb and the brake split what
// remains evenly — apex_time = (1 - probe_share) / 2. A taller probe therefore
// pushes the whole bell of the arc earlier instead of squeezing the brake, so
// the schedule always fits the swing_time envelope, and a zero probe restores
// the even split. The apex stays over the spatial midpoint of the travel
// regardless: swing_arc warps the horizontal blend's clock to cross
// half-travel at apex_time.
//
// Climb: the quintic lift, plus a mirrored Hermite term that hands the foot a
// definite upward speed at t = 0 (zero would creep it off the ground inside a
// single servo step while it still carries weight). The speed is not a knob:
// 2 * clearance / climb_time — the end slope of the parabola through the same
// apex, and provably the largest value for which the climb stays monotone.
//
// Descent: the quintic brake down to the probe, then a straight line at
// exactly `touchdown_velocity` the rest of the way. The braked part carries a
// Hermite term scaled to its segment, so it arrives at the probe travelling at
// exactly the probe's speed — the seam matches in position and velocity both.
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
float swing_height(float t, float apex_time, float clearance, float swing_time,
                   float touchdown_velocity, float probe_time) {
  const float climb_time = apex_time * swing_time;
  if (t < apex_time) {
    const float liftoff_velocity = 2.0f * clearance / climb_time;
    const float u = t / apex_time;
    return clearance * ease5(u) -
           liftoff_velocity * climb_time * hermite_end_slope(1.0f - u);
  }

  // The height the probe covers, from the time actually granted, so whichever
  // cap bit, the brake still hands over at exactly the probe's speed.
  const float probe_z = touchdown_velocity * probe_time;
  const float descent_time = (1.0f - apex_time) * swing_time;
  const float elapsed = (t - apex_time) * swing_time;
  const float brake_time = descent_time - probe_time;
  if (elapsed >= brake_time) {
    return touchdown_velocity * (descent_time - elapsed);
  }
  const float w = brake_time > 0.0f ? elapsed / brake_time : 1.0f;
  return probe_z + (clearance - probe_z) * (1.0f - ease5(w)) -
         touchdown_velocity * brake_time * hermite_end_slope(w);
}
}  // namespace

// Evaluate from the nearer end: near u = 1 the polynomial's order-100 terms
// cancel down to ~1, leaving float noise big enough to jitter the foot by
// micrometres right at the touchdown seam. Reflecting keeps both tails exact.
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

  // Apex height is measured from the ground the foot is stepping between, so
  // swing_clearance keeps meaning "how high the foot lifts". The blended base
  // under the apex is the endpoint midpoint (the warp crosses half-travel at
  // the apex), so on a swing between two heights the lift's amplitude absorbs
  // half the height gap and the arc tops out at the higher end plus the
  // profile's clearance. A profile asking for no lift at all (the reseat
  // landing) gets no shaping either — lifting over half the height gap would
  // swing the foot up off a start it was told to descend from — and rides the
  // plain eased blend instead.
  float clearance = 0.0f;
  if (profile.clearance > 0.0f) {
    const float ground_z = std::max(swing_origin[2], target[2]);
    clearance = std::max(0.0f, ground_z + profile.clearance -
                                   0.5f * (swing_origin[2] + target[2]));
  }
  const float probe_time =
      granted_probe_time(clearance, swing_time, profile.touchdown_velocity,
                         profile.touchdown_probe_fraction);
  const float apex_time = 0.5f * (1.0f - probe_time / swing_time);
  // The travel clock: the blend's full span, ending where the ride begins.
  // ride_time <= probe_time <= 0.4 * swing_time, so travel_end >= 0.6; a zero
  // ride makes u == t and the warp argument apex_time exactly — the un-ridden
  // arc, bit for bit.
  const float ride_time =
      granted_ride_time(profile.ride_headroom, v_ground_in.norm(),
                        v_ground_out.norm(), probe_time);
  const float travel_end = 1.0f - ride_time / swing_time;
  const float u = std::min(t / travel_end, 1.0f);
  const float tau = apex_warp(u, apex_time / travel_end);
  const float blend = ease7(tau);

  // The two ground lines: where a foot planted at lift-off would have got to by
  // now, and where the foot about to touch down would have come from had it been
  // planted all along. Blending between them with ease7 pins the swing to the
  // first at t = 0 and the second from travel_end on (the ride — at a granted
  // ride the swing *is* the touchdown line for the rest of the descent) in
  // position, velocity *and* acceleration, which is exactly the continuity
  // stance needs at both seams.
  const Vec3 from_liftoff = swing_origin + v_ground_in * (swing_time * t);
  const Vec3 to_touchdown = target - v_ground_out * (swing_time * (1.0f - t));
  Vec3 point = (1.0f - blend) * from_liftoff + blend * to_touchdown;

  if (clearance > 0.0f) {
    point[2] += swing_height(t, apex_time, clearance, swing_time,
                             profile.touchdown_velocity, probe_time);
  }

  // Lateral bulge. Shares the lift's spatial symmetry rather than its height,
  // so it survives a swing with zero clearance (the pause descent).
  point[1] +=
      (identity_y_sign > 0 ? profile.width : -profile.width) * bump(blend);

  return point;
}

}  // namespace hexa::gait
