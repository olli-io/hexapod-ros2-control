#include "gait/gaits/registry.hpp"

#include "gait/gaits/base.hpp"
#include "gait/trajectory.hpp"

namespace hexa::gait {
namespace {

// Offset tables; function-local statics avoid static-init-order issues.

const PhaseOffsets& tripod_offsets() {
  static const PhaseOffsets offsets({
      {"l_front", 0.0f},
      {"r_middle", 0.0f},
      {"l_rear", 0.0f},
      {"r_front", 0.5f},
      {"l_middle", 0.5f},
      {"r_rear", 0.5f},
  });
  return offsets;
}

const PhaseOffsets& tetrapod_offsets() {
  // Three diagonal pairs swing together at offsets 0, 1/3, 2/3 (Wilson Type II).
  static const PhaseOffsets offsets({
      {"l_front", 0.0f},
      {"r_middle", 0.0f},
      {"r_front", 1.0f / 3.0f},
      {"l_rear", 1.0f / 3.0f},
      {"l_middle", 2.0f / 3.0f},
      {"r_rear", 2.0f / 3.0f},
  });
  return offsets;
}

const PhaseOffsets& surf_offsets() {
  // Tripod-clustered metachronal staggering; stagger 1/10, just below the
  // beta=5/8 stability cliff at 1/8. Offsets are the mirror of lift-off times.
  static const PhaseOffsets offsets({
      {"r_rear", 0.0f},
      {"l_middle", 1.0f / 10.0f},
      {"r_front", 2.0f / 10.0f},
      {"l_rear", 1.0f / 2.0f},
      {"r_middle", 1.0f / 2.0f + 1.0f / 10.0f},
      {"l_front", 1.0f / 2.0f + 2.0f / 10.0f},
  });
  return offsets;
}

// Wilson's posterior->anterior protraction wave, contralateral side half a cycle
// out of phase. Crawl and ripple differ only in duty factor.
const PhaseOffsets& metachronal_offsets() {
  static const PhaseOffsets offsets({
      {"r_rear", 0.0f},
      {"r_middle", 2.0f / 3.0f},
      {"r_front", 1.0f / 3.0f},
      {"l_rear", 1.0f / 2.0f},
      {"l_middle", 1.0f / 6.0f},
      {"l_front", 5.0f / 6.0f},
  });
  return offsets;
}

// Lateral-sequence creep on the four corner legs; the middle pair is parked.
// A leg lifts at master pymod(-offset, 1), so the offsets run the mirror of the
// lift order: left rear, left front, right rear, right front — each hind
// followed by the fore on its own side. Reading the table as the lift order
// instead gives the diagonal sequence, whose static margin is negative on this
// chassis. The middles are present at 0 only because PhaseOffsets insists on
// all six — colliding with l_rear makes a missed parked-leg filter fail loudly
// instead of walking a tucked leg.
const PhaseOffsets& quad_walk_offsets() {
  static const PhaseOffsets offsets({
      {"l_rear", 0.0f},
      {"r_front", 1.0f / 4.0f},
      {"r_rear", 1.0f / 2.0f},
      {"l_front", 3.0f / 4.0f},
      {"l_middle", 0.0f},
      {"r_middle", 0.0f},
  });
  return offsets;
}

// The other one-leg-at-a-time order for the same four corners: a perimeter
// sequence walking round the chassis, right front, left front, left rear, right
// rear. Same mirror as quad_walk — that lift order needs the offsets r_front 0,
// r_rear 1/4, l_rear 1/2, l_front 3/4 — and the middles are at 0 for the same
// fail-loudly reason. Unlike quad_walk it lifts the two fores back to back, so
// its handovers hand the body across the chassis rather than up one side.
const PhaseOffsets& quad_canter_offsets() {
  static const PhaseOffsets offsets({
      {"r_front", 0.0f},
      {"r_rear", 1.0f / 4.0f},
      {"l_rear", 1.0f / 2.0f},
      {"l_front", 3.0f / 4.0f},
      {"l_middle", 0.0f},
      {"r_middle", 0.0f},
  });
  return offsets;
}

// Shared by every strategy. Swing is [0, stride.swing_end) on swing_arc; stance
// is the rest, a quartic Bezier from AEP toward PEP at constant tip velocity.
Vec3 phased_foot_target(float phase, const StrideParams& stride,
                        const LegContext& leg) {
  const Vec3 nominal = leg.nominal_stance;
  const Vec3 stride_vec = stride.stride_vector;

  const Vec3 pep = nominal - 0.5f * stride_vec;
  const Vec3 aep = nominal + 0.5f * stride_vec;

  const float swing_end = stride.swing_end;
  const float stance_fraction = 1.0f - swing_end;
  const float stance_time = stride.cycle_time * stance_fraction;

  if (phase < swing_end) {
    const float phase_in_swing = swing_end > 0.0f ? phase / swing_end : 0.0f;
    const float swing_time = stride.cycle_time * swing_end;
    // The foot leaves and rejoins the ground at the stance tip velocity — the
    // stride over *stance* time. swing_arc's default would overstate it by
    // stance_time / swing_time, which is 1 only for an unmargined tripod.
    const Vec3 v_ground =
        stance_time > 0.0f ? (-stride_vec / stance_time) : Vec3::Zero();
    return swing_arc(phase_in_swing, pep, aep, identity_y_sign(nominal),
                     swing_time, stride.swing, v_ground, v_ground);
  }

  const float stance_phase = (phase - swing_end) / stance_fraction;
  const BezierNodes stance_nodes = generate_stance_control_nodes(aep, stride_vec);
  return quartic_bezier(stance_nodes, stance_phase);
}

class Tripod : public Strategy {
 public:
  const PhaseOffsets& phase_offsets() const override { return tripod_offsets(); }
  float duty_factor() const override { return 0.5f; }
  bool unstable() const override { return false; }
  Vec3 foot_target(float phase, const StrideParams& stride,
                   const LegContext& leg) const override {
    return phased_foot_target(phase, stride, leg);
  }
};

class Tetrapod : public Strategy {
 public:
  const PhaseOffsets& phase_offsets() const override {
    return tetrapod_offsets();
  }
  float duty_factor() const override { return 2.0f / 3.0f; }
  bool unstable() const override { return false; }
  Vec3 foot_target(float phase, const StrideParams& stride,
                   const LegContext& leg) const override {
    return phased_foot_target(phase, stride, leg);
  }
};

class Surf : public Strategy {
 public:
  const PhaseOffsets& phase_offsets() const override { return surf_offsets(); }
  float duty_factor() const override { return 5.0f / 8.0f; }
  bool unstable() const override { return true; }
  Vec3 foot_target(float phase, const StrideParams& stride,
                   const LegContext& leg) const override {
    return phased_foot_target(phase, stride, leg);
  }
};

class Crawl : public Strategy {
 public:
  const PhaseOffsets& phase_offsets() const override {
    return metachronal_offsets();
  }
  float duty_factor() const override { return 2.0f / 3.0f; }
  bool unstable() const override { return true; }
  Vec3 foot_target(float phase, const StrideParams& stride,
                   const LegContext& leg) const override {
    return phased_foot_target(phase, stride, leg);
  }
};

class Ripple : public Strategy {
 public:
  const PhaseOffsets& phase_offsets() const override {
    return metachronal_offsets();
  }
  float duty_factor() const override { return 5.0f / 6.0f; }
  bool unstable() const override { return false; }
  Vec3 foot_target(float phase, const StrideParams& stride,
                   const LegContext& leg) const override {
    return phased_foot_target(phase, stride, leg);
  }
};

// The four corners creeping one leg at a time. beta = 3/4 puts the lift-offs a
// quarter cycle apart, which is more than the 0.22 swing window, so exactly one
// foot is ever airborne and every handover has all four down for 0.03 of a
// cycle. Not the six-leg `tetrapod` gait, which is a different animal: there
// "tetrapod" counts feet on the ground, here it counts legs.
class QuadWalk : public Strategy {
 public:
  const PhaseOffsets& phase_offsets() const override {
    return quad_walk_offsets();
  }
  float duty_factor() const override { return 3.0f / 4.0f; }
  bool unstable() const override { return false; }
  LegSet leg_set() const override { return LegSet::QUADRUPED; }
  Vec3 foot_target(float phase, const StrideParams& stride,
                   const LegContext& leg) const override {
    return phased_foot_target(phase, stride, leg);
  }
};

// The same creep on the perimeter order. Everything but the offsets is
// quad_walk's: same leg set, same duty factor, so the same swing window, the
// same velocity cap and the same support-shift machinery carry it.
class QuadCanter : public Strategy {
 public:
  const PhaseOffsets& phase_offsets() const override {
    return quad_canter_offsets();
  }
  float duty_factor() const override { return 3.0f / 4.0f; }
  bool unstable() const override { return false; }
  LegSet leg_set() const override { return LegSet::QUADRUPED; }
  Vec3 foot_target(float phase, const StrideParams& stride,
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
      {"quad_walk", [] { return std::make_unique<QuadWalk>(); }},
      {"quad_canter", [] { return std::make_unique<QuadCanter>(); }},
  };
  return registry;
}

}  // namespace hexa::gait
