// gtest port of the hexa_display test_display_node_idling pytest suite. Drives
// the extracted FaceAnimationRunner (the former node clock) with a deterministic
// rest sampler and counting sinks — no rclcpp, no device.
//
// Each IDLING step relays exactly one output (a gaze or a blink), so the sink
// count equals the number of steps played.

#include <gtest/gtest.h>

#include "Expression.h"
#include "face_animation.hpp"
#include "face_animation_runner.hpp"

using namespace hexa_display;

namespace {

struct Harness {
    int writes = 0;
    FaceAnimationRunner runner;

    explicit Harness(double rng_value)
        : runner([rng_value](double, double) { return rng_value; },
                 [this](GazeDirection) { ++writes; },
                 [this]() { ++writes; }) {}

    int play(const FaceAnimation& animation, double span_s) {
        runner.startCycle(animation, 0.0);
        for (double t = 0.0; t < span_s; t += 0.1) {
            runner.run(animation, t);
        }
        return writes;
    }
};

}  // namespace

TEST(FaceAnimationRunner, IdlingRestsBetweenBursts) {
    // With a 7 s repeat interval over 20 s the burst fires 3 times (t=0, 7, 14),
    // not the ~6 it would if it looped every 3.04 s.
    const int steps = Harness(7.0).play(idling(), 20.0);
    EXPECT_EQ(steps, 3 * static_cast<int>(idling().steps().size()));
}

TEST(FaceAnimationRunner, IdlingFirstBurstIsASingleCycle) {
    const int steps = Harness(7.0).play(idling(), idling().period_s());
    EXPECT_EQ(steps, static_cast<int>(idling().steps().size()));
}

TEST(FaceAnimationRunner, IdlingRestartUsesRandomInterval) {
    const int short_run = Harness(5.0).play(idling(), 30.0);
    const int long_run = Harness(10.0).play(idling(), 30.0);
    EXPECT_GT(short_run, long_run);
}
