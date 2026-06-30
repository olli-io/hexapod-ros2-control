#include "hexa_gait_cpp/gaits/common.hpp"

#include "hexa_gait_cpp/trajectory.hpp"

namespace hexa_gait {

const PhaseOffsets& metachronal_offsets() {
  // Realizes lift-offs at: r_rear 0, r_middle 1/3, r_front 2/3 (rear -> middle
  // -> front); l_rear 1/2, l_middle 5/6, l_front 1/6 (same wave + 1/2 cycle).
  static const PhaseOffsets offsets({
      {"r_rear", 0.0},
      {"r_middle", 2.0 / 3.0},
      {"r_front", 1.0 / 3.0},
      {"l_rear", 1.0 / 2.0},
      {"l_middle", 1.0 / 6.0},
      {"l_front", 5.0 / 6.0},
  });
  return offsets;
}

Vec3 phased_foot_target(double phase, const StrideParams& stride,
                        const LegContext& leg) {
  const Vec3 nominal = leg.nominal_stance;
  const Vec3 stride_vec = stride.stride_vector;

  const Vec3 pep = nominal - 0.5 * stride_vec;
  const Vec3 aep = nominal + 0.5 * stride_vec;

  const double swing_end = 1.0 - stride.duty_factor;
  if (phase < swing_end) {
    const double phase_in_swing = swing_end > 0.0 ? phase / swing_end : 0.0;
    const double swing_time = stride.cycle_time * (1.0 - stride.duty_factor);
    return swing_arc(phase_in_swing, pep, aep, stride.swing_clearance,
                     stride.swing_width, identity_y_sign(nominal), swing_time,
                     stride.controller_dt);
  }

  const double stance_phase = (phase - swing_end) / stride.duty_factor;
  const BezierNodes stance_nodes =
      generate_stance_control_nodes(aep, stride_vec);
  return quartic_bezier(stance_nodes, stance_phase);
}

}  // namespace hexa_gait
