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

// Shape of one swing, independent of where the foot is travelling. Bundled so
// the engine, the engagement controller and the strategies all shape a swing the
// same way instead of each defaulting a different subset.
struct SwingProfile {
  // Apex height above the ground (step_height).
  float clearance = 0.0f;
  // Sideways shift of the arc; 0 = straight fore/aft.
  float width = 0.0f;
  // Share of the transfer arc spent climbing to the apex.
  float apex_fraction = 0.5f;
  // Height of the ground-matched lift-off / touchdown ramps, as a share of the
  // apex height above. 0 disables them and reproduces the plain two-curve arc.
  //
  // A fraction rather than a distance so it keeps its meaning when step height
  // is retuned: the ramps are always the same slice of the arc. The ramps'
  // duration is not configurable — it is a fixed share of the swing (see
  // kRampSwingShare), because that share is what governs how much time and
  // distance the ramps cost the transfer arc. The vertical speed the foot
  // leaves and meets the ground at therefore falls out of the gait rather than
  // being set: a bigger band or a quicker swing means a brisker departure.
  float ramp_clearance_fraction = 0.0f;
  // Vertical speed the foot still carries as it meets the ground.
  float touchdown_velocity = 0.0f;
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

// Linear cmd plus tangential yaw contribution at each hip:
// v_leg = v_body + omega x r, in the body frame, for every leg.
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

// Evaluate the swing trajectory at phase_in_swing in [0, 1).
//
// With profile.ramp_clearance > 0 the swing runs in four segments: a
// ground-matched lift-off ramp, the two-curve transfer arc, and a ground-matched
// touchdown ramp. On the ramps the foot's horizontal velocity is exactly the
// ground velocity, so it cannot scrub while it is still close enough to the
// ground to be touching it; the horizontal speed-up happens only once the
// lift-off ramp has concluded. With ramp_clearance = 0 the ramps vanish and this
// is the plain two-curve arc.
//
// origin/target_ground_velocity are the stance velocities at the two ends —
// horizontal only, the profile supplies the vertical shaping. nullopt defaults
// to the analytical -stride / swing_time; pass Vec3::Zero() for a rest-to-rest
// move (pause, reseat, the initialize ladder).
//
// profile.apex_fraction is the share of the transfer arc spent climbing to the
// apex. 0.5 splits it evenly. Below 0.5 the touchdown half runs longer, which
// stretches its Bezier and widens its node separation: a longer, larger-radius
// descent that reaches the ground more gently for the same step height.
Vec3 swing_arc(float phase_in_swing, const Vec3& swing_origin,
               const Vec3& target, int identity_y_sign, float swing_time,
               const SwingProfile& profile,
               std::optional<Vec3> origin_ground_velocity = std::nullopt,
               std::optional<Vec3> target_ground_velocity = std::nullopt);

}  // namespace hexa::gait
