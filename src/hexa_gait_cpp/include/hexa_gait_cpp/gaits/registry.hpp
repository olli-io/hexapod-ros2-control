// Strategy registry: name -> zero-arg factory. Port of gaits/__init__.py's
// STRATEGIES. The engine looks up by name (from /gait/params) when set_strategy
// is called. Adding a new gait is: define the class in registry.cpp and add an
// entry to the map.
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "hexa_gait_cpp/gaits/base.hpp"

namespace hexa_gait {

using StrategyFactory = std::function<std::unique_ptr<Strategy>()>;

// The registered gait strategies, keyed by the same names the rest of the
// system uses (tripod, surf, tetrapod, crawl, ripple).
const std::map<std::string, StrategyFactory>& strategies();

}  // namespace hexa_gait
