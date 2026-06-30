// Phase clock for the gait engine. Port of clock.py.
//
// A GaitClock owns the engine's master phase in [0, 1) and projects it through
// per-leg phase offsets. Strategies stay pure functions of phase: the clock is
// the only place where time enters the gait chain.
#pragma once

#include <map>
#include <string>

#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {

// Per-leg cycle start, relative to the master phase, in [0, 1).
class PhaseOffsets {
 public:
  // Validates that every leg is present and each offset is in [0, 1).
  // Throws std::invalid_argument otherwise (mirrors __post_init__).
  explicit PhaseOffsets(std::map<std::string, double> offsets);

  const std::map<std::string, double>& offsets() const { return offsets_; }
  double at(const std::string& leg) const { return offsets_.at(leg); }

 private:
  std::map<std::string, double> offsets_;
};

// Master phase clock with per-leg projections. advance() integrates the master
// phase modulo one cycle; phases() returns each leg's (master + offset) mod 1.
class GaitClock {
 public:
  explicit GaitClock(PhaseOffsets offsets);

  double master() const { return master_; }
  void reset(double master = 0.0);
  void advance(double dt, double cycle_time);
  std::map<std::string, double> phases() const;

 private:
  PhaseOffsets offsets_;
  double master_ = 0.0;
};

}  // namespace hexa_gait
