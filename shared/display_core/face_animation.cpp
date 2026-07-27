#include "face_animation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hexa::display {

FaceAnimation::FaceAnimation(std::string name, double period_s,
                             std::vector<FaceAnimationStep> steps,
                             std::optional<std::pair<double, double>> repeat_range_s,
                             double gaze_ease_s, double gaze_scale)
    : name_(std::move(name)),
      period_s_(period_s),
      steps_(std::move(steps)),
      repeat_range_s_(repeat_range_s),
      gaze_ease_s_(gaze_ease_s),
      gaze_scale_(gaze_scale) {
    if (gaze_ease_s_ < 0.0) {
        throw std::invalid_argument(name_ + ": gaze_ease_s must be >= 0");
    }
    if (gaze_scale_ <= 0.0 || gaze_scale_ > 1.0) {
        throw std::invalid_argument(name_ + ": gaze_scale must be in (0, 1]");
    }
    if (steps_.empty()) {
        throw std::invalid_argument(name_ + ": animation needs at least one step");
    }
    double last = -1.0;
    for (const auto& step : steps_) {
        if (step.at_s < last) {
            throw std::invalid_argument(name_ + ": steps must be time-ordered");
        }
        last = step.at_s;
    }
    if (last >= period_s_) {
        throw std::invalid_argument(name_ + ": last step is past the period");
    }
    if (repeat_range_s_) {
        const double lo = repeat_range_s_->first;
        const double hi = repeat_range_s_->second;
        if (lo > hi) {
            throw std::invalid_argument(name_ + ": repeat_range_s is not ordered (min, max)");
        }
        if (lo < period_s_) {
            throw std::invalid_argument(name_ + ": repeat_range_s min is shorter than the cycle");
        }
    }
}

std::int64_t stepCountAt(const FaceAnimation& animation, double elapsed_s) {
    if (elapsed_s < 0.0) {
        return 0;
    }
    // Mirrors Python divmod(elapsed_s, period): integer cycle count + remainder.
    const double cycle = std::floor(elapsed_s / animation.period_s());
    const double t_in = elapsed_s - cycle * animation.period_s();
    std::int64_t in_cycle = 0;
    for (const auto& step : animation.steps()) {
        if (step.at_s <= t_in) {
            ++in_cycle;
        }
    }
    return static_cast<std::int64_t>(cycle) *
               static_cast<std::int64_t>(animation.steps().size()) +
           in_cycle;
}

std::pair<std::vector<FaceAnimationStep>, std::int64_t>
dueSteps(const FaceAnimation& animation, double elapsed_s, std::int64_t fired_count) {
    const std::int64_t target = stepCountAt(animation, elapsed_s);
    const std::int64_t n = static_cast<std::int64_t>(animation.steps().size());
    const std::int64_t start = std::max(fired_count, target - n);
    std::vector<FaceAnimationStep> steps;
    for (std::int64_t k = start; k < target; ++k) {
        steps.push_back(animation.steps()[static_cast<std::size_t>(k % n)]);
    }
    return {std::move(steps), target};
}

const FaceAnimation& breathing() {
    // gaze_ease_s matches the 1.2 s step spacing, so the dip and the return
    // chain into one continuous motion; the tail to the 4.8 s period rests the
    // eyes at center between breaths. Half-amplitude keeps the dip a breath,
    // not a glance at the floor.
    static const FaceAnimation kBreathing(
        "breathing", 4.8,
        {
            {0.0, GazeDirection::DOWN, false},
            {1.2, GazeDirection::CENTER, false},
        },
        std::nullopt, 1.2, 0.5);
    return kBreathing;
}

const FaceAnimation& idling() {
    // Mirrors the firmware test sequence: look left, blink, look right, look up,
    // look down, recenter, blink; the tail to the 3.04 s period lets the last
    // blink play out. The eyes rest at CENTER between bursts; repeat_range_s
    // spaces those bursts 5-10 s apart.
    static const FaceAnimation kIdling(
        "idling", 3.04,
        {
            {0.0, GazeDirection::LEFT, false},
            {0.44, std::nullopt, true},
            {0.8, GazeDirection::RIGHT, false},
            {1.24, GazeDirection::UP, false},
            {1.68, GazeDirection::DOWN, false},
            {2.12, GazeDirection::CENTER, false},
            {2.48, std::nullopt, true},
        },
        std::make_pair(5.0, 10.0));
    return kIdling;
}

const std::map<std::string, const FaceAnimation*>& faceAnimations() {
    static const std::map<std::string, const FaceAnimation*> kRegistry = {
        {breathing().name(), &breathing()},
        {idling().name(), &idling()},
    };
    return kRegistry;
}

const FaceAnimation* findFaceAnimation(const std::string& name) {
    const auto& registry = faceAnimations();
    const auto it = registry.find(name);
    return it == registry.end() ? nullptr : it->second;
}

}  // namespace hexa::display
