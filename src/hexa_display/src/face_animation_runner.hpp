// The face-animation clock.
//
// Pure (no rclcpp): the stateful half of running a FaceAnimation, extracted
// from the display node so it is unit-testable standalone. It owns the elapsed
// clock, the fired-step counter, the pending-animation debounce, and the
// randomized rest interval between idling bursts. Randomness and the gaze/blink
// outputs are injected (mirroring EyeAnim's RNG injection), so tests drive it
// with a deterministic sampler and counting sinks.

#pragma once

#include <functional>
#include <optional>
#include <string>

#include "Expression.h"  // GazeDirection
#include "face_animation.hpp"

namespace hexa_display {

class FaceAnimationRunner {
public:
    // uniform(lo, hi) -> a sample in [lo, hi] (the rest interval between bursts).
    using UniformFn = std::function<double(double, double)>;
    using GazeSink = std::function<void(GazeDirection)>;
    using BlinkSink = std::function<void()>;

    FaceAnimationRunner(UniformFn uniform, GazeSink on_gaze, BlinkSink on_blink);

    // Select/activate the named animation (nullopt = none). idling_start_delay_s
    // gates the start of idling only. Returns the active animation or nullptr.
    const FaceAnimation* update(const std::optional<std::string>& name, double now,
                                double idling_start_delay_s);

    // Relay the steps of the active animation newly due at `now`.
    void run(const FaceAnimation& animation, double now);

    // Start (or restart) a cycle at `now`. Exposed for testing.
    void startCycle(const FaceAnimation& animation, double now);

private:
    UniformFn uniform_;
    GazeSink on_gaze_;
    BlinkSink on_blink_;

    const FaceAnimation* active_ = nullptr;
    double start_t_ = 0.0;
    std::int64_t fired_ = 0;
    double next_start_t_ = 0.0;
    std::optional<std::string> pending_;
    double pending_since_ = 0.0;
};

}  // namespace hexa_display
