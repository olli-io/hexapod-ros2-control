// gtest port of the hexa_display face_animation pytest suite. Pure sequence /
// clock helpers — no rclcpp, no device.

#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "Expression.h"
#include "face_animation.hpp"

using namespace hexa::display;

TEST(FaceAnimation, RegistryNamesMatch) {
    std::set<std::string> names;
    for (const auto& [name, animation] : faceAnimations()) {
        names.insert(name);
        EXPECT_EQ(animation->name(), name);
    }
    EXPECT_EQ(names, (std::set<std::string>{"breathing", "idling"}));
}

TEST(FaceAnimation, BreathingIsASlowDownwardDip) {
    // Down and back only — the tail to the period rests the eyes at center.
    const std::vector<GazeDirection> expected = {GazeDirection::DOWN,
                                                 GazeDirection::CENTER};
    std::vector<GazeDirection> gazes;
    for (const auto& step : breathing().steps()) {
        ASSERT_TRUE(step.gaze.has_value());
        gazes.push_back(*step.gaze);
        EXPECT_FALSE(step.blink);
    }
    EXPECT_EQ(gazes, expected);
    EXPECT_DOUBLE_EQ(breathing().period_s(), 4.8);
}

TEST(FaceAnimation, IdlingMirrorsReferenceSequence) {
    struct Expected {
        double at_s;
        std::optional<GazeDirection> gaze;
        bool blink;
    };
    const Expected expected[] = {
        {0.0, GazeDirection::LEFT, false},
        {0.44, std::nullopt, true},
        {0.8, GazeDirection::RIGHT, false},
        {1.24, GazeDirection::UP, false},
        {1.68, GazeDirection::DOWN, false},
        {2.12, GazeDirection::CENTER, false},
        {2.48, std::nullopt, true},
    };
    const auto& steps = idling().steps();
    ASSERT_EQ(steps.size(), std::size(expected));
    for (size_t i = 0; i < steps.size(); ++i) {
        EXPECT_DOUBLE_EQ(steps[i].at_s, expected[i].at_s) << i;
        EXPECT_EQ(steps[i].gaze, expected[i].gaze) << i;
        EXPECT_EQ(steps[i].blink, expected[i].blink) << i;
    }
    EXPECT_DOUBLE_EQ(idling().period_s(), 3.04);
    ASSERT_TRUE(idling().repeat_range_s().has_value());
    EXPECT_DOUBLE_EQ(idling().repeat_range_s()->first, 5.0);
    EXPECT_DOUBLE_EQ(idling().repeat_range_s()->second, 10.0);
}

TEST(FaceAnimation, BreathingLoopsWithoutARestGap) {
    EXPECT_FALSE(breathing().repeat_range_s().has_value());
}

TEST(FaceAnimation, BreathingDriftsContinuously) {
    // The drift duration matches the step spacing, so consecutive gaze targets
    // chain into one continuous oscillation at half amplitude; idling keeps
    // the default full-travel dart.
    EXPECT_DOUBLE_EQ(breathing().gaze_ease_s(), 1.2);
    EXPECT_DOUBLE_EQ(breathing().gaze_scale(), 0.5);
    EXPECT_DOUBLE_EQ(idling().gaze_ease_s(), 0.0);
    EXPECT_DOUBLE_EQ(idling().gaze_scale(), 1.0);
}

TEST(FaceAnimation, StepCountAtBoundaries) {
    EXPECT_EQ(stepCountAt(idling(), -0.5), 0);
    EXPECT_EQ(stepCountAt(idling(), 0.0), 1);  // at_s == elapsed fires
    EXPECT_EQ(stepCountAt(idling(), 0.44), 2);
    EXPECT_EQ(stepCountAt(idling(), 3.0), 7);
    EXPECT_EQ(stepCountAt(idling(), idling().period_s()), 8);  // next cycle starts
}

TEST(FaceAnimation, DueStepsOverIncrementalTicks) {
    std::int64_t fired = 0;
    std::vector<FaceAnimationStep> seen;
    for (double t = 0.0; t < idling().period_s() * 2; t += 0.1) {
        auto [steps, new_fired] = dueSteps(idling(), t, fired);
        fired = new_fired;
        for (const auto& s : steps) seen.push_back(s);
    }
    const auto& ref = idling().steps();
    ASSERT_EQ(seen.size(), ref.size() * 2);
    for (size_t i = 0; i < ref.size(); ++i) {
        EXPECT_EQ(seen[i].gaze, ref[i].gaze);
        EXPECT_EQ(seen[i].blink, ref[i].blink);
        EXPECT_EQ(seen[i + ref.size()].gaze, ref[i].gaze);
        EXPECT_EQ(seen[i + ref.size()].blink, ref[i].blink);
    }
}

TEST(FaceAnimation, DueStepsStallReplaysAtMostOneCycle) {
    const double elapsed = idling().period_s() * 10 + 1.0;
    auto [steps, fired] = dueSteps(idling(), elapsed, 0);
    EXPECT_EQ(steps.size(), idling().steps().size());
    EXPECT_EQ(fired, stepCountAt(idling(), elapsed));
}

TEST(FaceAnimation, Validation) {
    EXPECT_THROW(FaceAnimation("empty", 1.0, {}), std::invalid_argument);
    EXPECT_THROW(FaceAnimation("past-period", 1.0,
                               {{1.5, GazeDirection::UP, false}}),
                 std::invalid_argument);
    EXPECT_THROW(FaceAnimation("unordered", 1.0,
                               {{0.5, GazeDirection::UP, false},
                                {0.2, GazeDirection::DOWN, false}}),
                 std::invalid_argument);
    EXPECT_THROW(FaceAnimation("bad-range", 1.0, {{0.0, GazeDirection::UP, false}},
                               std::make_pair(5.0, 2.0)),
                 std::invalid_argument);
    EXPECT_THROW(FaceAnimation("short-range", 4.0, {{0.0, GazeDirection::UP, false}},
                               std::make_pair(2.0, 6.0)),
                 std::invalid_argument);
    EXPECT_THROW(FaceAnimation("negative-ease", 1.0,
                               {{0.0, GazeDirection::UP, false}}, std::nullopt,
                               -0.5),
                 std::invalid_argument);
    EXPECT_THROW(FaceAnimation("zero-scale", 1.0,
                               {{0.0, GazeDirection::UP, false}}, std::nullopt,
                               0.0, 0.0),
                 std::invalid_argument);
    EXPECT_THROW(FaceAnimation("over-scale", 1.0,
                               {{0.0, GazeDirection::UP, false}}, std::nullopt,
                               0.0, 1.5),
                 std::invalid_argument);
}
