#include "gait/clock.hpp"

#include <stdexcept>

namespace hexa::gait {

PhaseOffsets::PhaseOffsets(std::map<std::string, float> offsets)
    : offsets_(std::move(offsets)) {
  for (const auto& name : LEG_NAMES) {
    if (offsets_.find(name) == offsets_.end()) {
      throw std::invalid_argument("PhaseOffsets missing leg: " + name);
    }
  }
  for (const auto& [name, value] : offsets_) {
    if (!(value >= 0.0f && value < 1.0f)) {
      throw std::invalid_argument("PhaseOffsets[" + name + "] not in [0, 1)");
    }
  }
}

namespace {
// pymod is half-open only in exact arithmetic: a tiny negative argument rounds
// up to exactly 1.0f. Folding that back to zero is the same phase.
float wrapped(float value) {
  const float m = pymod(value, 1.0f);
  return m < 1.0f ? m : 0.0f;
}
}  // namespace

PhaseOffsets PhaseOffsets::reversed() const {
  std::map<std::string, float> out;
  for (const auto& [name, value] : offsets_) {
    out[name] = wrapped(-value);
  }
  return PhaseOffsets(std::move(out));
}

GaitClock::GaitClock(PhaseOffsets offsets) : offsets_(std::move(offsets)) {}

void GaitClock::reset(float master) {
  if (!(master >= 0.0f && master < 1.0f)) {
    throw std::invalid_argument("master must be in [0, 1)");
  }
  master_ = master;
}

void GaitClock::advance(float dt, float cycle_time) {
  if (cycle_time <= 0.0f) {
    throw std::invalid_argument("cycle_time must be positive");
  }
  master_ = pymod(master_ + dt / cycle_time, 1.0f);
}

void GaitClock::mirror(float about) {
  master_ = wrapped(about - master_);
  offsets_ = offsets_.reversed();
}

std::map<std::string, float> GaitClock::phases() const {
  std::map<std::string, float> out;
  for (const auto& name : LEG_NAMES) {
    out[name] = pymod(master_ + offsets_.at(name), 1.0f);
  }
  return out;
}

}  // namespace hexa::gait
