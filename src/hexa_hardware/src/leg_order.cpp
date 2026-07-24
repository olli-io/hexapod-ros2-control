#include "hexa_hardware/leg_order.hpp"

#include <stdexcept>
#include <unordered_map>

namespace hexa_hardware {

namespace {

// "l_rear_femur_joint" -> "l_rear". The 6-leg × 3-segment naming is fixed, so
// an unrecognised suffix is a config error rather than something to guess at.
std::string leg_of(const std::string& joint_name) {
  static const char* kSuffixes[] = {"_coxa_joint", "_femur_joint",
                                    "_tibia_joint"};
  for (const char* suffix : kSuffixes) {
    const std::string s(suffix);
    if (joint_name.size() > s.size() &&
        joint_name.compare(joint_name.size() - s.size(), s.size(), s) == 0) {
      return joint_name.substr(0, joint_name.size() - s.size());
    }
  }
  throw std::runtime_error("hexa_hardware: joint '" + joint_name +
                           "' is not a <leg>_{coxa,femur,tibia}_joint");
}

}  // namespace

std::vector<LegGroup> build_leg_order(const std::vector<std::string>& joint_names,
                                      const std::vector<PinEntry>& pin_order) {
  std::vector<LegGroup> legs;
  std::unordered_map<std::string, std::size_t> seen;  // leg name -> legs index

  // pin_order is ascending, so the first time a leg is seen carries its lowest
  // pin — appending in that order already sorts the legs by lowest pin, and each
  // group's indices stay ascending.
  for (std::size_t i = 0; i < pin_order.size(); ++i) {
    const std::size_t joint_idx = pin_order[i].joint_idx;
    if (joint_idx >= joint_names.size()) {
      throw std::runtime_error("hexa_hardware: pin entry references joint " +
                               std::to_string(joint_idx) + " out of range");
    }
    const std::string leg = leg_of(joint_names[joint_idx]);
    const auto it = seen.find(leg);
    if (it == seen.end()) {
      seen.emplace(leg, legs.size());
      legs.push_back(LegGroup{leg, {i}});
    } else {
      legs[it->second].pin_order_idx.push_back(i);
    }
  }
  return legs;
}

}  // namespace hexa_hardware
