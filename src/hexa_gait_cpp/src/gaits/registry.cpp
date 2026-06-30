#include "hexa_gait_cpp/gaits/registry.hpp"

#include "hexa_gait_cpp/gaits/common.hpp"

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
