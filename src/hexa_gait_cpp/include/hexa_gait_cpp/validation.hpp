// Small shared validation helper: every controller checks that a per-leg map
// covers all six legs and raises with the missing names listed (mirrors the
// `set(LEG_NAMES) - set(...)` checks throughout the Python package).
#pragma once

#include <map>
#include <stdexcept>
#include <string>

#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {

template <typename Value>
void require_all_legs(const std::map<std::string, Value>& m,
                      const std::string& what) {
  std::string missing;
  for (const auto& name : LEG_NAMES) {
    if (m.find(name) == m.end()) {
      if (!missing.empty()) {
        missing += ", ";
      }
      missing += name;
    }
  }
  if (!missing.empty()) {
    throw std::invalid_argument(what + " missing legs: " + missing);
  }
}

}  // namespace hexa_gait
