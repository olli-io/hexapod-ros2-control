#include "face_animation_runner.hpp"

#include <utility>

namespace hexa_display {

FaceAnimationRunner::FaceAnimationRunner(UniformFn uniform, GazeSink on_gaze,
                                         BlinkSink on_blink)
    : uniform_(std::move(uniform)),
      on_gaze_(std::move(on_gaze)),
      on_blink_(std::move(on_blink)) {}

const FaceAnimation* FaceAnimationRunner::update(const std::optional<std::string>& name,
                                                 double now,
                                                 double idling_start_delay_s) {
    if (!name) {
        pending_.reset();
        active_ = nullptr;
        return nullptr;
    }
    if (active_ != nullptr && active_->name() == *name) {
        return active_;
    }
    if (pending_ != name) {
        pending_ = name;
        pending_since_ = now;
    }
    const double delay = (*name == idling().name()) ? idling_start_delay_s : 0.0;
    if (now - pending_since_ < delay) {
        active_ = nullptr;
        return nullptr;
    }
    active_ = findFaceAnimation(*name);
    if (active_ != nullptr) {
        startCycle(*active_, now);
    }
    return active_;
}

void FaceAnimationRunner::startCycle(const FaceAnimation& animation, double now) {
    start_t_ = now;
    fired_ = 0;
    if (!animation.repeat_range_s()) {
        next_start_t_ = now;
    } else {
        const auto [lo, hi] = *animation.repeat_range_s();
        next_start_t_ = now + uniform_(lo, hi);
    }
}

void FaceAnimationRunner::run(const FaceAnimation& animation, double now) {
    if (animation.repeat_range_s() && now - start_t_ >= animation.period_s()) {
        // The burst has played out; rest with the eyes at center until the next
        // randomized start instead of looping back.
        if (now < next_start_t_) {
            return;
        }
        startCycle(animation, now);
    }
    auto [steps, fired] = dueSteps(animation, now - start_t_, fired_);
    fired_ = fired;
    for (const auto& step : steps) {
        if (step.blink) {
            on_blink_();
        }
        if (step.gaze) {
            on_gaze_(*step.gaze);
        }
    }
}

}  // namespace hexa_display
