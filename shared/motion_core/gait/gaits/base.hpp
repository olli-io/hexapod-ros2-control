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

  // The probe band's height for a swing of `swing_time` seconds.
  float probe_band(float swing_time) const {
    const float f = touchdown_probe_fraction < kMaxProbeFraction
                        ? touchdown_probe_fraction
                        : kMaxProbeFraction;
    return touchdown_velocity * f * swing_time;
  }
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

// Per-leg stride displacement, magnitude-clamped to stride_length.
Vec3 stride_vector(float v_x, float v_y, float stance_time, float stride_length);

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
