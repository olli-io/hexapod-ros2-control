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

// Per-tick stride description for one leg. stride_vector is the body-frame
// displacement the foot covers during one full stance phase (AEP -> PEP).
struct StrideParams {
  Vec3 stride_vector = Vec3::Zero();
  float cycle_time = 0.0f;
  float duty_factor = 0.0f;
  float swing_clearance = 0.0f;
  float swing_width = 0.0f;
  float controller_dt = 0.0f;
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

// Pick cycle_time so the fastest leg's stride equals stride_length, clamped to
// [min_cycle_time, max_cycle_time].
float derive_cycle_time(float max_leg_v, float stride_length, float duty_factor,
                        float min_cycle_time, float max_cycle_time);

// Touchdown target in the body frame: nominal + 1/2 * stride_vec.
Vec3 live_aep(const Vec3& nominal, const Vec3& stride_vec);

// +1 if the nominal foot sits at positive y, else -1.
int identity_y_sign(const Vec3& nominal_stance);

// Evaluate the two-curve swing trajectory at phase_in_swing in [0, 1).
// swing_origin_velocity defaults (nullopt) to the analytical lift-off velocity
// -stride / swing_time; pass Vec3::Zero() for a rest-to-rest move.
// swing_target_velocity (nullopt) defaults to the same.
//
// swing_apex_fraction is the share of swing_time spent climbing to the apex.
// 0.5 splits the swing evenly. Below 0.5 the touchdown half runs longer, which
// stretches its Bezier and widens its node separation: a longer, larger-radius
// descent that reaches the ground more gently for the same step height.
Vec3 swing_arc(float phase_in_swing, const Vec3& swing_origin,
               const Vec3& target, float swing_clearance, float swing_width,
               int identity_y_sign, float swing_time,
               std::optional<Vec3> swing_origin_velocity = std::nullopt,
               std::optional<Vec3> swing_target_velocity = std::nullopt,
               float swing_apex_fraction = 0.5f);

}  // namespace hexa::gait
