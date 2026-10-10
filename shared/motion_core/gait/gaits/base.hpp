// Gait strategy interface and shared swing-arc helper. A Strategy is a pure
// function (phase, stride, leg) -> foot_target; the engine owns the clock and
// the per-leg pause / engagement state.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <utility>

#include "gait/clock.hpp"
#include "gait/types.hpp"

namespace hexa::gait {

// One leg's geometry, body-frame except mount_yaw. nominal_stance is the foot
// position at a zero cmd_vel.
struct LegContext {
  std::string name;
  Vec3 mount_xyz = Vec3::Zero();
  float mount_yaw = 0.0f;
  Vec3 nominal_stance = Vec3::Zero();
};

// A leg's radial axis in the ground plane: unit vector from coxa mount out to
// nominal stance, plus the reach along it. Moves when a reseat commits a new
// nominal, so read it off the leg each tick rather than caching it.
struct RadialAxis {
  float u_x = 0.0f;
  float u_y = 0.0f;
  // |nominal_stance_xy - mount_xy|; zero for a degenerate leg (constrains
  // nothing).
  float tip_reach = 0.0f;
};

RadialAxis radial_axis(const LegContext& leg);

// How far past its design band (half a stride) a stance anchor may drift before
// the integrator stops it, as a fraction of the band. Also the distance the foot
// brakes over. A tripod planted across a fast 90-degree turn leaves the band at
// once; at 0.25 the brake held it to a quarter of the commanded speed for half a
// second. 2.0 halves that dip and keeps ~19 mm of reach (probe_turn.cpp).
constexpr float kStanceExcursionGrace = 2.0f;

// Shape of one swing, independent of where the foot is travelling. Bundled so
// the engine, the engagement controller and the strategies agree on the defaults.
struct SwingProfile {
  // Apex height above the ground (step_height).
  float clearance = 0.0f;
  // Sideways shift of the arc; 0 = straight fore/aft.
  float width = 0.0f;
  // Share of the swing spent climbing, in (0, 0.5]. The rest is the descent, so
  // a lower value gives a slower approach to the ground and a faster lift-off.
  float apex_time = 0.5f;
  // Height the foot climbs straight up from its lift-off point, ground frame,
  // before any horizontal travel. Capped at kMaxStraightShare of the clearance.
  float lift_height = 0.0f;
  // Height from which the foot descends straight down onto its touchdown
  // ground line, ground frame, after all horizontal travel. Same cap.
  float land_height = 0.0f;
};

// Largest share of the clearance each straight segment may take, so the climb
// and the descent keep room to carry the travel over the apex.
constexpr float kMaxStraightShare = 0.5f;

// Per-tick stride for one leg; stride_vector is the body-frame displacement the
// foot covers over one stance (AEP -> PEP).
struct StrideParams {
  Vec3 stride_vector = Vec3::Zero();
  float cycle_time = 0.0f;
  float swing_end = 0.0f;
  float controller_dt = 0.0f;
  SwingProfile swing;
};

class Strategy {
 public:
  virtual ~Strategy() = default;
  virtual const PhaseOffsets& phase_offsets() const = 0;
  virtual float duty_factor() const = 0;
  // Skipped by the teleop D-pad rotation unless allow_unstable_gaits.
  virtual bool unstable() const = 0;
  // Which legs this gait walks. Defaulted, so a six-leg strategy says nothing.
  virtual LegSet leg_set() const { return LegSet::HEXAPOD; }
  virtual Vec3 foot_target(float phase, const StrideParams& stride,
                           const LegContext& leg) const = 0;
};

// v_leg = v_body + omega x r_stance, body frame. The lever arm is the leg's
// nominal_stance (where the stride is laid down), not its mount.
std::map<std::string, std::pair<float, float>> per_leg_planar_velocity(
    const std::map<std::string, LegContext>& leg_contexts,
    std::pair<float, float> v_body_xy, float omega_z);

// Per-leg stride displacement, magnitude-clamped to the tick's effective stride
// (see effective_stride_length).
Vec3 stride_vector(float v_x, float v_y, float stance_time, float stride_length);

// The longest stride this leg may take along the unit direction (d_x, d_y)
// before its coxa-to-foot reach closes by more than stride_length_radial. The
// half-stride is laid down about the nominal stance, so the constraint is
//
//   |R * r + a * L * d| >= R - stride_length_radial / 2,   a = 1/2
//
// solved (not projected — projection overcharges a leg whose tangential
// component restores some reach) as the near root of a quadratic in L, with
// c = d . r:
//
//   L = (R / a) * (-c - sqrt(c^2 - 1 + m^2 / R^2)),   m = R - radial / 2
//
// c is taken *unsigned*: every leg is assumed to be the one tucking, so the
// budget cannot differ between d and -d, which the reversal ladder relies on.
// Returns +infinity when the closure can never bind.
float radial_stride_budget(const RadialAxis& axis, float d_x, float d_y,
                           float stride_length_radial);

// One stride scalar for the whole tick: stride_length cut to the tightest
// per-leg radial budget. The stride has to be common — every planted foot rides
// the same body, so per-leg strides would only scrub them against each other.
//
// Each budget is divided by that leg's share of the fastest leg's speed, which
// is what the leg actually lays down; under pure translation every share is 1
// and this is a plain minimum, and a leg barely moving relaxes to no constraint
// with no epsilon to pick.
float effective_stride_length(
    const std::map<std::string, LegContext>& legs,
    const std::map<std::string, std::pair<float, float>>& leg_velocities,
    float stride_length, float stride_length_radial);

// Same, from a command rather than a per-leg velocity map.
float effective_stride_length(const std::map<std::string, LegContext>& legs,
                              std::pair<float, float> v_body_xy, float omega_z,
                              float stride_length, float stride_length_radial);

// Phase at which swing ends and stance resumes. The nominal window [0, 1 - beta)
// is shortened by margin_fraction of its own length at the touchdown end, so
// every handover has a stretch with all six feet planted. Taking the margin
// entirely at that end keeps phase = 0 meaning lift-off, which the rest of the
// stack relies on. margin_fraction is clamped to [0, 0.5].
float swing_end_phase(float duty_factor, float margin_fraction);

// Pick cycle_time so the fastest leg's stride equals stride_length, clamped to
// [min_cycle_time, max_cycle_time]. stance_fraction is 1 - swing_end_phase(),
// not the nominal duty factor, once a margin is in play.
float derive_cycle_time(float max_leg_v, float stride_length,
                        float stance_fraction, float min_cycle_time,
                        float max_cycle_time);

// The knee: the leg speed where derive_cycle_time reaches max_cycle_time. Above
// it the stride is pinned at stride_length; below it the stride shortens.
float knee_speed(float stride_length, float stance_fraction,
                 float max_cycle_time);

Vec3 live_aep(const Vec3& nominal, const Vec3& stride_vec);

// How far a planted foot may drift from its leg's nominal stance. `band` is the
// AEP..PEP envelope a steady walk rides exactly; `ceiling` is the hard limit.
// Between them the outward rate eases to zero, where a hard clip would kink the
// foot target.
struct StanceBand {
  Vec3 nominal = Vec3::Zero();
  float band = 0.0f;
  float ceiling = 0.0f;
};

// Ease the outward part of one stance step to zero as the foot leaves its band.
// Inward and tangential motion is untouched, so a leg carried out recovers at
// full rate the moment the command turns: a wall, not a spring.
//
// The gain is 1 with zero slope at `band` — ordinary walking rides exactly there
// at AEP and PEP — and 0 at `ceiling`, which the foot can therefore never pass. It
// shapes the *increment*, never the accumulated state: re-clipping the state every
// tick would creep the foot inward even at zero velocity.
//
// Shared, because the walk is not the only thing that carries a planted foot at
// the commanded velocity: the engagement ladder does too, and a command turned
// under it walks a just-landed foot straight back out past its own touchdown.
std::pair<float, float> ease_outward(float e_x, float e_y, float d_x, float d_y,
                                     float band, float ceiling);

int identity_y_sign(const Vec3& nominal_stance);

// Quintic smoothstep: zero slope and curvature at both ends.
inline float ease5(float u) {
  return u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
}

// Septic smoothstep: ease5 plus a vanishing third derivative at both ends, so
// the curve departs as O(t^4). Undefined outside [0, 1] — callers clamp.
float ease7(float u);

// Evaluate the swing trajectory at phase_in_swing in [0, 1). One curve, not a
// chain of segments.
//
// The horizontal track is a septic-eased blend between the two ground lines —
// where a foot planted at lift-off would have got to, and where the landing foot
// would have come from — so it leaves and meets the ground at exactly the ground
// velocity and only pulls away as O(t^4), and never scrubs while it may still be
// touching.
//
// The horizontal blend is held at zero until the climb reaches
// profile.lift_height and at one once the descent passes profile.land_height,
// so the foot rises straight up off its lift-off point and sets straight down
// onto its touchdown point, both in the ground frame. The blend runs between.
//
// The vertical is eased independently with its apex at profile.apex_time and
// over the spatial midpoint of the travel (the blend's clock is warped to cross
// half-travel there). The lift-off speed is 2 * clearance / climb_time, and the
// foot meets the ground at zero vertical speed.
//
// origin/target_ground_velocity are the horizontal stance velocities at the two
// ends; nullopt defaults to -stride / swing_time, and Vec3::Zero() gives a
// rest-to-rest move (pause, reseat, the initialize ladder).
Vec3 swing_arc(float phase_in_swing, const Vec3& swing_origin,
               const Vec3& target, int identity_y_sign, float swing_time,
               const SwingProfile& profile,
               std::optional<Vec3> origin_ground_velocity = std::nullopt,
               std::optional<Vec3> target_ground_velocity = std::nullopt);

// Swing progress from which swing_arc's horizontal is the touchdown ground line
// alone (1 without a straight landing). From there a caller that knows the live
// body velocity may carry the foot itself, which stays ground-fixed when the
// command changes under the landing; the arc's line only does for a constant one.
float swing_land_start(const Vec3& swing_origin, const Vec3& target,
                       const SwingProfile& profile);

}  // namespace hexa::gait
