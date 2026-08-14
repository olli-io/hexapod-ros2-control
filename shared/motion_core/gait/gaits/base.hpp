// Gait strategy interface and shared swing-arc helper. Float fork of
// gaits/base.hpp (plan part 06).
//
// A Strategy is a pure function (phase, stride, leg) -> foot_target. It carries
// no state, performs no I/O, and reads no clocks. The engine owns the phase
// clock and per-leg pause / engagement state.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <utility>

#include "gait/clock.hpp"
#include "gait/types.hpp"

namespace hexa::gait {

// Geometric description of one leg as the engine sees it. All fields are
// body-frame quantities except mount_yaw. nominal_stance is the foot position
// when cmd_vel is zero (the visual standing pose).
struct LegContext {
  std::string name;
  Vec3 mount_xyz = Vec3::Zero();
  float mount_yaw = 0.0f;
  Vec3 nominal_stance = Vec3::Zero();
};

// A leg's own radial axis in the ground plane: the unit vector from its coxa
// mount out to its nominal stance, and the reach along it. A property of the
// standing pose, so it moves only when a reseat commits a new nominal — read it
// off the leg each tick rather than caching it.
struct RadialAxis {
  float u_x = 0.0f;
  float u_y = 0.0f;
  // |nominal_stance_xy - mount_xy|; zero for a degenerate leg, which then
  // constrains nothing.
  float tip_reach = 0.0f;
};

RadialAxis radial_axis(const LegContext& leg);

// How far past its design band a stance anchor may drift before the integrator
// stops it, as a fraction of the band. The band is half a stride — exactly the
// AEP..PEP envelope a steady walk rides — so the grace is only what a leg
// borrows while the command turns under it.
//
// It is also the distance the foot has to shed its ground speed over, so it
// trades excursion against how hard the foot is braked. On this geometry a full
// lateral reversal peaks 14 mm past the band unbounded, and 0.25 gives up 8 mm
// of that for a braking step of ~8% of stance speed per tick — a tenth of what a
// hard clip would do, and well under what a swing already asks of the same leg.
//
// The swing's touchdown ride shares the same envelope: its headroom beyond the
// live AEP is grace x band (see SwingProfile::ride_headroom), and the AEP is
// never further than the band from nominal, so a parked foot stays inside the
// same ceiling a stance anchor may drift to.
//
// Lives here beside SwingProfile rather than in the engine: the engagement
// shapes its own swings and needs the same headroom, and both halves of the
// envelope — the band's ceiling and the ride's overshoot — read this one number.
constexpr float kStanceExcursionGrace = 0.25f;

// Largest share of the swing the constant-velocity probe may take. The climb
// and the brake split the rest evenly, so at the cap each still keeps 30% of
// the swing — the brake never collapses however large the probe.
constexpr float kMaxProbeFraction = 0.4f;

// Shape of one swing, independent of where the foot is travelling. Bundled so
// the engine, the engagement controller and the strategies all shape a swing the
// same way instead of each defaulting a different subset.
struct SwingProfile {
  // Apex height above the ground (step_height).
  float clearance = 0.0f;
  // Sideways shift of the arc; 0 = straight fore/aft.
  float width = 0.0f;
  // Vertical speed the foot still carries as it meets the ground, and the speed
  // of the probe below. Shapes only the descent's approach — the track through
  // space never moves.
  float touchdown_velocity = 0.0f;
  // Share of the swing spent in the straight probe at exactly
  // touchdown_velocity, rather than still easing as it arrives. The probe's
  // height — the band a foot may meet the ground anywhere inside and still land
  // at the intended speed, which is what has to beat the servos' position
  // resolution — is touchdown_velocity * fraction * swing_time, so it always
  // fits in the swing whatever the timing config. The probe's time comes off
  // the tail of the swing and the climb and brake split the remainder, so a
  // taller probe also moves the apex earlier and buys the whole descent time.
  // Clamped to kMaxProbeFraction; 0 restores a zero-speed landing.
  float touchdown_probe_fraction = 0.0f;
  // How far beyond the touchdown target the arc may park the foot to ride the
  // touchdown ground line. Riding the line makes the foot world-frame
  // stationary over its landing point while the probe descends, so an early
  // contact in the ridden part of the band lands without horizontal slip; the
  // price is parking past the target by ground speed times the time ridden,
  // which is what this headroom meters. Because the live AEP never sits more
  // than half a stride from the nominal stance (stride_vector's magnitude
  // clamp), headroom = grace x half-stride keeps every parked foot inside the
  // stance integrator's ceiling — whatever the command did mid-swing. The
  // grant also tapers away with the ground speed (see granted_ride_time), so
  // slow and rest-to-rest swings keep the un-ridden schedule. 0 disables the
  // ride and the blend spans the whole swing.
  float ride_headroom = 0.0f;

  // The configured probe share after its hard cap.
  float probe_fraction() const {
    return touchdown_probe_fraction < kMaxProbeFraction
               ? touchdown_probe_fraction
               : kMaxProbeFraction;
  }

  // The probe band's height for a swing of `swing_time` seconds.
  float probe_band(float swing_time) const {
    return touchdown_velocity * probe_fraction() * swing_time;
  }

  // Swing progress at which the probe begins. From here on the foot is inside
  // the band it may meet the ground anywhere within, so this is where a swing
  // stops being free to move over the ground and starts having to answer for it.
  float probe_start() const { return 1.0f - probe_fraction(); }
};

// Per-tick stride description for one leg. stride_vector is the body-frame
// displacement the foot covers during one full stance phase (AEP -> PEP).
struct StrideParams {
  Vec3 stride_vector = Vec3::Zero();
  float cycle_time = 0.0f;
  // Phase at which swing ends and stance resumes; see swing_end_phase().
  float swing_end = 0.0f;
  float controller_dt = 0.0f;
  SwingProfile swing;
};

// A gait strategy maps (phase, stride, leg) to a body-frame foot target.
class Strategy {
 public:
  virtual ~Strategy() = default;
  virtual const PhaseOffsets& phase_offsets() const = 0;
  virtual float duty_factor() const = 0;
  // True for gaits that are inherently less stable than the rest of the
  // registry. The teleop D-pad rotation skips these unless allow_unstable_gaits.
  virtual bool unstable() const = 0;
  virtual Vec3 foot_target(float phase, const StrideParams& stride,
                           const LegContext& leg) const = 0;
};

// Linear cmd plus tangential yaw contribution at each foot:
// v_leg = v_body + omega x r_stance, in the body frame, for every leg. The
// lever arm is the leg's nominal_stance (where the stride is actually laid
// down), not its mount — see the note in the definition.
std::map<std::string, std::pair<float, float>> per_leg_planar_velocity(
    const std::map<std::string, LegContext>& leg_contexts,
    std::pair<float, float> v_body_xy, float omega_z);

// Per-leg stride displacement, magnitude-clamped to the tick's effective stride
// (see effective_stride_length).
Vec3 stride_vector(float v_x, float v_y, float stance_time, float stride_length);

// The longest stride this leg may take along the unit direction (d_x, d_y)
// before its own coxa-to-foot reach closes by more than stride_length_radial.
//
// A stride is laid down as +/- half of itself about the nominal stance, so what
// a leg spends is not the stride but the reach it gives up at the near end:
//
//   |R * r + a * L * d| >= R - stride_length_radial / 2,   a = 1/2
//
// Solved rather than projected. The half-stride's tangential component partly
// restores the reach, so projecting the stride onto r overstates what a leg
// gives up — 92 mm against the true 95 mm for a rear leg walking forward. The
// one leg that gets none of that relief is the one travelling straight out
// along its own axis (d parallel to r), which is a middle leg walking sideways:
// exactly the case this budget exists to price. Projecting instead would charge
// the rear legs for a forward walk they have always managed.
//
// With c = d . r the closure is a quadratic in L, and the near root is
//
//   L = (R / a) * (-c - sqrt(c^2 - 1 + m^2 / R^2)),   m = R - radial / 2
//
// c is taken *unsigned* — every leg is assumed to be the one tucking, never the
// one extending. The signed form makes the budget differ between d and -d on a
// fore/aft-asymmetric stance, which would break the reversal ladder's premise
// that the stride is the one the legs have been *and* will be walking.
//
// Returns +infinity when the closure can never bind (non-positive
// discriminant), so a slack budget and a purely tangential direction — every
// leg under pure yaw — leave the stride untouched.
float radial_stride_budget(const RadialAxis& axis, float d_x, float d_y,
                           float stride_length_radial);

// One stride scalar for the whole tick: stride_length, cut to the tightest
// per-leg radial budget.
//
// The stride has to be common. Every planted foot is rigidly attached to the
// same body and translates at the same velocity, so per-leg strides would only
// scrub the feet against each other; the per-leg part is the derivation, not
// the result. Each leg contributes a constraint and the tightest one wins.
//
// Each budget is divided by that leg's share of the fastest leg's speed,
// because that share is what the leg actually lays down: in the distance-locked
// band stance_time is the tick's stride over max_leg_v, so a leg at half the
// peak speed takes half the stride. Under pure translation all six shares are 1
// and this is a plain minimum; it only bites when yaw skews the leg speeds. It
// also means a leg barely moving relaxes to no constraint on its own, with no
// epsilon to pick.
//
// Returns stride_length exactly when nothing is moving, when the budget is
// slack, or when no leg binds.
float effective_stride_length(
    const std::map<std::string, LegContext>& legs,
    const std::map<std::string, std::pair<float, float>>& leg_velocities,
    float stride_length, float stride_length_radial);

// Same, for callers holding a command rather than a per-leg velocity map.
float effective_stride_length(const std::map<std::string, LegContext>& legs,
                              std::pair<float, float> v_body_xy, float omega_z,
                              float stride_length, float stride_length_radial);

// Phase at which swing ends and stance resumes.
//
// The strategy's nominal swing window is [0, 1 - beta). margin_fraction shortens
// it at the touchdown end by that share of its own length, so every leg lands
// before its neighbour lifts off and each handover has a stretch with all six
// feet planted. Taking the whole margin at the touchdown end (rather than
// splitting it across both) leaves phase = 0 meaning lift-off, which the rest of
// the stack — the engagement schedule, docs/leg-phases.md, the posture
// animations' master-phase offsets — already relies on. The two are otherwise
// equivalent: shifting the window inside the cycle only relabels master phase,
// so the stance-count profile depends solely on the window's length.
//
// margin_fraction is clamped to [0, 0.4]; the window can never collapse.
float swing_end_phase(float duty_factor, float margin_fraction);

// Pick cycle_time so the fastest leg's stride equals stride_length, clamped to
// [min_cycle_time, max_cycle_time]. stance_fraction is the share of the cycle
// the foot spends planted — 1 - swing_end_phase(), not the nominal duty factor,
// once a margin is in play.
float derive_cycle_time(float max_leg_v, float stride_length,
                        float stance_fraction, float min_cycle_time,
                        float max_cycle_time);

// Touchdown target in the body frame: nominal + 1/2 * stride_vec.
Vec3 live_aep(const Vec3& nominal, const Vec3& stride_vec);

// +1 if the nominal foot sits at positive y, else -1.
int identity_y_sign(const Vec3& nominal_stance);

// Quintic smoothstep. 0 -> 1 with zero slope and zero curvature at both ends.
inline float ease5(float u) {
  return u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
}

// Evaluate the swing trajectory at phase_in_swing in [0, 1).
//
// One curve, not a chain of segments. The horizontal track is a septic-eased
// blend between the two ground lines — where a foot planted at lift-off would
// have got to, and where the foot about to land would have come from. Because
// the blend weight's first three derivatives vanish at both ends, the foot
// leaves and meets the ground travelling at exactly the ground velocity with
// zero horizontal acceleration, and only pulls away from it as O(t^4). That is
// what keeps it from scrubbing while it may still be touching.
//
// The blend spans the swing up to the touchdown ride. Finishing the travel
// early rides the moving ground line for the remainder, which sweeps the foot
// beyond the AEP by ground speed times the time left — so the ride is metered
// against profile.ride_headroom, and with none the blend spans the whole swing
// and the body-frame excursion outside the AEP..PEP envelope stays a few
// millimetres. Where the headroom affords it, the travel
// instead completes at the probe's start and the foot descends the whole band
// already moving with the ground: an early contact anywhere inside it lands
// without slip — the horizontal analog of what the probe does for the
// vertical.
//
// The vertical is eased independently, with its apex over the spatial midpoint
// of the travel — the blend's clock is warped to cross half-travel at the
// apex — so the arc is symmetric in space and no knob can displace the apex
// along the track. The apex's *time* is derived, not configured: the straight
// probe (which meets the ground at exactly profile.touchdown_velocity) takes
// touchdown_probe_fraction off the tail of the swing, and the climb and the
// braked descent split the remaining time evenly, so a taller probe shifts
// the bell of the arc earlier and the schedule always fits swing_time. The
// lift-off speed is likewise derived (2 * clearance / climb_time, the largest
// monotone climb).
//
// origin/target_ground_velocity are the stance velocities at the two ends —
// horizontal only, the profile supplies the vertical shaping. nullopt defaults
// to the analytical -stride / swing_time; pass Vec3::Zero() for a rest-to-rest
// move (pause, reseat, the initialize ladder).
Vec3 swing_arc(float phase_in_swing, const Vec3& swing_origin,
               const Vec3& target, int identity_y_sign, float swing_time,
               const SwingProfile& profile,
               std::optional<Vec3> origin_ground_velocity = std::nullopt,
               std::optional<Vec3> target_ground_velocity = std::nullopt);

}  // namespace hexa::gait
