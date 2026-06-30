// Gait strategy interface and shared swing-arc helper. Port of gaits/base.py.
//
// A Strategy is a pure function (phase, stride, leg) -> foot_target. It carries
// no state, performs no I/O, and reads no clocks. The engine owns the phase
// clock and per-leg pause / engagement state.
//
// swing_arc packages the two quartic-Bezier swing curves from trajectory into a
// single phase_in_swing -> foot_target helper, reused by both the normal
// swing-phase evaluation and the PauseController Z-only descents.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <utility>

#include "hexa_gait_cpp/clock.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {

// Geometric description of one leg as the engine sees it. All fields are
// body-frame quantities except mount_yaw. nominal_stance is the foot position
// when cmd_vel is zero (the visual standing pose).
struct LegContext {
  std::string name;
  Vec3 mount_xyz = Vec3::Zero();
  double mount_yaw = 0.0;
  Vec3 nominal_stance = Vec3::Zero();
};

// Per-tick stride description for one leg. stride_vector is the body-frame
// displacement the foot covers during one full stance phase (AEP -> PEP).
struct StrideParams {
  Vec3 stride_vector = Vec3::Zero();
  double cycle_time = 0.0;
  double duty_factor = 0.0;
  double swing_clearance = 0.0;
  double swing_width = 0.0;
  double controller_dt = 0.0;
};

// A gait strategy maps (phase, stride, leg) to a body-frame foot target.
// Abstract base for the structural Protocol in the Python package.
class Strategy {
 public:
  virtual ~Strategy() = default;
  virtual const PhaseOffsets& phase_offsets() const = 0;
  virtual double duty_factor() const = 0;
  // True for gaits that are inherently less stable than the rest of the
  // registry. The teleop D-pad rotation skips these unless allow_unstable_gaits
  // is set.
  virtual bool unstable() const = 0;
  virtual Vec3 foot_target(double phase, const StrideParams& stride,
                           const LegContext& leg) const = 0;
};

// Linear cmd plus tangential yaw contribution at each hip:
// v_leg = v_body + omega x r, in the body frame, for every leg.
std::map<std::string, std::pair<double, double>> per_leg_planar_velocity(
    const std::map<std::string, LegContext>& leg_contexts,
    std::pair<double, double> v_body_xy, double omega_z);

// Per-leg stride displacement, magnitude-clamped to stride_length.
Vec3 stride_vector(double v_x, double v_y, double stance_time,
                   double stride_length);

// Pick cycle_time so the fastest leg's stride equals stride_length, clamped to
// [min_cycle_time, max_cycle_time].
double derive_cycle_time(double max_leg_v, double stride_length,
                         double duty_factor, double min_cycle_time,
                         double max_cycle_time);

// Touchdown target in the body frame: nominal + 1/2 * stride_vec.
Vec3 live_aep(const Vec3& nominal, const Vec3& stride_vec);

// +1 if the nominal foot sits at positive y, else -1.
int identity_y_sign(const Vec3& nominal_stance);

// Evaluate the two-curve swing trajectory at phase_in_swing in [0, 1).
// swing_origin_velocity defaults (nullopt) to the analytical lift-off velocity
// -stride / swing_time; pass Vec3::Zero() for a rest-to-rest move.
// swing_target_velocity (nullopt) defaults to the same; the engagement
// controller passes -v_leg so swing -> stance has no velocity step.
Vec3 swing_arc(double phase_in_swing, const Vec3& swing_origin,
               const Vec3& target, double swing_clearance, double swing_width,
               int identity_y_sign, double swing_time, double controller_dt,
               std::optional<Vec3> swing_origin_velocity = std::nullopt,
               std::optional<Vec3> swing_target_velocity = std::nullopt);

}  // namespace hexa_gait
