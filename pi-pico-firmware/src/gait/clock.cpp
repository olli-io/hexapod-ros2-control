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

std::map<std::string, float> GaitClock::phases() const {
  std::map<std::string, float> out;
  for (const auto& name : LEG_NAMES) {
    out[name] = pymod(master_ + offsets_.at(name), 1.0f);
  }
  return out;
}

}  // namespace hexa::gait
