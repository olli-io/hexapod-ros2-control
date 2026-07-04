// Strategy registry: name -> zero-arg factory. Float fork of gaits/registry.hpp
// (plan part 06). The engine looks up by name when set_strategy is called.
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "gait/gaits/base.hpp"

namespace hexa::gait {

using StrategyFactory = std::function<std::unique_ptr<Strategy>()>;

// The registered gait strategies, keyed by the same names the rest of the
// system uses (tripod, surf, tetrapod, crawl, ripple).
const std::map<std::string, StrategyFactory>& strategies();

}  // namespace hexa::gait
