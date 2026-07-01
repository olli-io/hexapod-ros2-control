#include "hexa_gait_cpp/gaits/registry.hpp"

#include "hexa_gait_cpp/gaits/base.hpp"
#include "hexa_gait_cpp/trajectory.hpp"

namespace hexa_gait {
namespace {

// ── Offset tables (function-local statics avoid static-init-order issues) ──

const PhaseOffsets& tripod_offsets() {
  // Tripod A (l_front, r_middle, l_rear) lifts off at master 0.0; tripod B
  // (r_front, l_middle, r_rear) at master 0.5.
  static const PhaseOffsets offsets({
      {"l_front", 0.0},
      {"r_middle", 0.0},
      {"l_rear", 0.0},
      {"r_front", 0.5},
      {"l_middle", 0.5},
      {"r_rear", 0.5},
  });
  return offsets;
}

const PhaseOffsets& tetrapod_offsets() {
  // Three diagonal pairs swing together at offsets 0, 1/3, 2/3 (Wilson Type II).
  static const PhaseOffsets offsets({
      {"l_front", 0.0},
      {"r_middle", 0.0},
      {"r_front", 1.0 / 3.0},
      {"l_rear", 1.0 / 3.0},
      {"l_middle", 2.0 / 3.0},
      {"r_rear", 2.0 / 3.0},
  });
  return offsets;
}

const PhaseOffsets& surf_offsets() {
  // Tripod-clustered metachronal staggering; stagger 1/10, just below the
  // beta=5/8 stability cliff at 1/8. Offsets are the mirror of lift-off times.
  static const PhaseOffsets offsets({
      {"r_rear", 0.0},
      {"l_middle", 1.0 / 10.0},
      {"r_front", 2.0 / 10.0},
      {"l_rear", 1.0 / 2.0},
      {"r_middle", 1.0 / 2.0 + 1.0 / 10.0},
      {"l_front", 1.0 / 2.0 + 2.0 / 10.0},
  });
  return offsets;
}

// Wilson's posterior->anterior protraction wave with the contralateral side
// half a cycle out of phase. Offsets are the mirror of lift-off times. Shared
// by crawl and ripple (they differ only in duty factor). Realizes lift-offs at:
// r_rear 0, r_middle 1/3, r_front 2/3 (rear -> middle -> front); l_rear 1/2,
// l_middle 5/6, l_front 1/6 (same wave + 1/2 cycle).
const PhaseOffsets& metachronal_offsets() {
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

// ── Shared foot-target evaluator (all strategies delegate here) ──

// Pure (phase, stride, leg) -> body-frame target. Swing window is
// [0, 1 - beta); stance window is [1 - beta, 1). Stance is a quartic Bezier
// from AEP toward PEP at constant tip velocity; swing is the two-curve
// swing_arc.
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

// ── Strategy classes (thin config holders; all share phased_foot_target) ──

class Tripod : public Strategy {
 public:
  const PhaseOffsets& phase_offsets() const override { return tripod_offsets(); }
  double duty_factor() const override { return 0.5; }
  bool unstable() const override { return false; }
  Vec3 foot_target(double phase, const StrideParams& stride,
                   const LegContext& leg) const override {
    return phased_foot_target(phase, stride, leg);
  }
};

class Tetrapod : public Strategy {
 public:
  const PhaseOffsets& phase_offsets() const override {
    return tetrapod_offsets();
  }
  double duty_factor() const override { return 2.0 / 3.0; }
  bool unstable() const override { return false; }
  Vec3 foot_target(double phase, const StrideParams& stride,
                   const LegContext& leg) const override {
    return phased_foot_target(phase, stride, leg);
  }
};

class Surf : public Strategy {
 public:
  const PhaseOffsets& phase_offsets() const override { return surf_offsets(); }
  double duty_factor() const override { return 5.0 / 8.0; }
  bool unstable() const override { return true; }
  Vec3 foot_target(double phase, const StrideParams& stride,
                   const LegContext& leg) const override {
    return phased_foot_target(phase, stride, leg);
  }
};

class Crawl : public Strategy {
 public:
  const PhaseOffsets& phase_offsets() const override {
    return metachronal_offsets();
  }
  double duty_factor() const override { return 2.0 / 3.0; }
  bool unstable() const override { return true; }
  Vec3 foot_target(double phase, const StrideParams& stride,
                   const LegContext& leg) const override {
    return phased_foot_target(phase, stride, leg);
  }
};

class Ripple : public Strategy {
 public:
  const PhaseOffsets& phase_offsets() const override {
    return metachronal_offsets();
  }
  double duty_factor() const override { return 5.0 / 6.0; }
  bool unstable() const override { return false; }
  Vec3 foot_target(double phase, const StrideParams& stride,
                   const LegContext& leg) const override {
    return phased_foot_target(phase, stride, leg);
  }
};

}  // namespace

const std::map<std::string, StrategyFactory>& strategies() {
  static const std::map<std::string, StrategyFactory> registry = {
      {"tripod", [] { return std::make_unique<Tripod>(); }},
      {"surf", [] { return std::make_unique<Surf>(); }},
      {"tetrapod", [] { return std::make_unique<Tetrapod>(); }},
      {"crawl", [] { return std::make_unique<Crawl>(); }},
      {"ripple", [] { return std::make_unique<Ripple>(); }},
  };
  return registry;
}

}  // namespace hexa_gait
