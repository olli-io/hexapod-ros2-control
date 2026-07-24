// Host unit tests for the staggered servo-rail energize sweep.
//
// Pure logic — no Pico SDK, no clock — so the inrush-staggering policy shared by
// hexa_hardware (ROS host) and the Pico firmware is exercised off-target.

#include "energize_sweep.hpp"

#include <gtest/gtest.h>

namespace {

constexpr float kDt = 0.005f;        // 200 Hz control tick
constexpr float kInterval = 0.150f;  // hardware.yaml init.sweep_leg_interval_ms

}  // namespace

TEST(EnergizeSweep, DisarmedUntilArmed) {
    hexa::EnergizeSweep s(kInterval);
    EXPECT_EQ(s.legs(), 0);
    EXPECT_FALSE(s.done());
    // Time passing while the rail is open must not energize anything.
    for (int i = 0; i < 200; ++i) EXPECT_EQ(s.step(kDt), 0);
}

TEST(EnergizeSweep, FirstLegIsLiveOnTheArmingTick) {
    // The relay closes and the first leg is driven in the same tick — the board
    // leaves every servo limp until its first SET, so there is nothing to wait
    // for.
    hexa::EnergizeSweep s(kInterval);
    s.arm();
    EXPECT_EQ(s.legs(), 1);
    EXPECT_FALSE(s.done());
}

TEST(EnergizeSweep, AddsOneLegPerInterval) {
    hexa::EnergizeSweep s(kInterval);
    s.arm();
    for (int leg = 1; leg < hexa::kNumLegs; ++leg) {
        EXPECT_EQ(s.legs(), leg);
        EXPECT_FALSE(s.done());
        s.step(kInterval);
    }
    EXPECT_EQ(s.legs(), hexa::kNumLegs);
    EXPECT_TRUE(s.done());
}

TEST(EnergizeSweep, FullSweepTakesFiveIntervals) {
    // Six legs, first one live at arm() → the sweep completes after 5 intervals.
    hexa::EnergizeSweep s(kInterval);
    s.arm();
    int ticks = 0;
    while (!s.done()) {
        s.step(kDt);
        ++ticks;
        ASSERT_LT(ticks, 10'000) << "sweep never completed";
    }
    const float elapsed = static_cast<float>(ticks) * kDt;
    EXPECT_NEAR(elapsed, kInterval * (hexa::kNumLegs - 1), kDt * 1.5f);
}

TEST(EnergizeSweep, StaysCompleteAfterTheSweep) {
    hexa::EnergizeSweep s(kInterval);
    s.arm();
    while (!s.done()) s.step(kDt);
    for (int i = 0; i < 100; ++i) EXPECT_EQ(s.step(kDt), hexa::kNumLegs);
}

TEST(EnergizeSweep, ZeroIntervalEnergizesEveryLegAtOnce) {
    // `sweep_leg_interval_ms: 0` opts out — the pre-sweep all-at-once behaviour.
    hexa::EnergizeSweep s(0.0f);
    s.arm();
    EXPECT_EQ(s.legs(), hexa::kNumLegs);
    EXPECT_TRUE(s.done());
}

TEST(EnergizeSweep, DisarmDropsEveryLeg) {
    hexa::EnergizeSweep s(kInterval);
    s.arm();
    s.step(kInterval);
    ASSERT_EQ(s.legs(), 2);
    s.disarm();
    EXPECT_EQ(s.legs(), 0);
    EXPECT_FALSE(s.done());
    EXPECT_EQ(s.step(kDt), 0);
}

TEST(EnergizeSweep, ReArmRestartsFromOneLeg) {
    // A fault drops the rail mid-sweep; the recovery edge re-staggers from
    // scratch rather than resuming where it left off.
    hexa::EnergizeSweep s(kInterval);
    s.arm();
    s.step(kInterval * 3.0f);
    ASSERT_GT(s.legs(), 1);
    s.disarm();
    s.arm();
    EXPECT_EQ(s.legs(), 1);
}

TEST(EnergizeSweep, LargeDtSkipsAheadWithoutOvershooting) {
    // A stalled tick must not energize past the leg count or wrap.
    hexa::EnergizeSweep s(kInterval);
    s.arm();
    EXPECT_EQ(s.step(kInterval * 100.0f), hexa::kNumLegs);
    EXPECT_TRUE(s.done());
}
