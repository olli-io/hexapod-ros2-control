// Strategy registry: name -> zero-arg factory, looked up by set_strategy.
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "gait/gaits/base.hpp"

namespace hexa::gait {

using StrategyFactory = std::function<std::unique_ptr<Strategy>()>;

// Keyed by the names the rest of the system uses (tripod, surf, ...).
const std::map<std::string, StrategyFactory>& strategies();

}  // namespace hexa::gait
