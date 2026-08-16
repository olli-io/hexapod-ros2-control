// The supervisor's failsafe / telemetry / status-LED policy: the battery
// hysteresis debounce, the stale-input watchdog, the relay-arming discipline, the
// LED mapping and the tick-jitter accounting.

#include "supervisor.hpp"

#include <cstdint>

#include <gtest/gtest.h>

namespace sup = hexa::supervisor;

namespace {

constexpr uint64_t kSec = 1'000'000;  // µs per second

// A config with the ladder ENABLED (the shipped default disables every rung —
// tested separately). warning 10 V, fold 9 V, cutoff 8 V, 0.3 V hysteresis, 3 s
// hold; 20 ms tick, 4 ms overrun margin.
sup::Config enabled_config() {
    return sup::Config{
        /*input_timeout_s=*/0.5f,
        /*battery_warning_v=*/10.0f,
        /*battery_fold_v=*/9.0f,
        /*battery_cutoff_v=*/8.0f,
        /*battery_hysteresis_v=*/0.3f,
        /*battery_hold_s=*/3.0f,
        /*tick_period_us=*/20'000,
        /*tick_margin_us=*/4'000,
    };
}

// Hold `volts` across the 3 s debounce with a fresh link, standing. Returns the
// decision on the tick the rung arms.
sup::Decision latch_at(sup::Supervisor& s, float volts, uint64_t start_us) {
    sup::Observation o;
    o.now_us = start_us;
    o.bt_connected = true;
    o.last_input_us = start_us;
    o.armable = true;
    o.folded = false;
    o.walking = false;
    o.battery_valid = true;
    o.battery_v = volts;
    o.fault = false;
    s.step(o);
    o.now_us = start_us + 3 * kSec + 1;  // past the hold
    o.last_input_us = o.now_us;
    return s.step(o);
}

// A baseline healthy observation: linked, fresh input, standing, not walking, no
// battery sample. Tests tweak individual fields.
sup::Observation healthy(uint64_t now_us) {
    sup::Observation o;
    o.now_us = now_us;
    o.bt_connected = true;
    o.last_input_us = now_us;  // just-arrived frame
    o.armable = true;
    o.folded = false;
    o.walking = false;
    o.battery_valid = false;
    o.battery_v = 0.0f;
    o.fault = false;
    return o;
}

}  // namespace

// ── BatteryMonitor ──────────────────────────────────────────────────────────

TEST(BatteryMonitor, DisabledThresholdNeverFires) {
    // Shipped default: 0.0 disables. Even a dead-flat pack stays clear.
    sup::BatteryMonitor bm(0.0f, 0.0f, 0.0f, 0.3f, 3.0f);
    for (int i = 0; i < 100; ++i) {
        bm.update(0.0f, static_cast<float>(i));
        EXPECT_FALSE(bm.warn());
        EXPECT_FALSE(bm.fold());
        EXPECT_FALSE(bm.cutoff());
    }
}

TEST(BatteryMonitor, RaisesOnlyAfterHold) {
    sup::BatteryMonitor bm(10.0f, 9.0f, 8.0f, 0.3f, 3.0f);
    // Below warning but not yet for hold_s → not latched.
    bm.update(9.5f, 0.0f);
    EXPECT_FALSE(bm.warn());
    bm.update(9.5f, 2.9f);
    EXPECT_FALSE(bm.warn());
    // Crossed the 3 s hold → warning arms; still above the deeper rungs.
    bm.update(9.5f, 3.0f);
    EXPECT_TRUE(bm.warn());
    EXPECT_FALSE(bm.fold());
    EXPECT_FALSE(bm.cutoff());
}

TEST(BatteryMonitor, HoldResetsOnBriefRecovery) {
    sup::BatteryMonitor bm(10.0f, 9.0f, 8.0f, 0.3f, 3.0f);
    bm.update(9.5f, 0.0f);   // below
    bm.update(9.5f, 2.0f);   // still counting
    bm.update(10.5f, 2.5f);  // popped back above → timer resets
    bm.update(9.5f, 4.0f);   // below again, but only 0 s into a fresh hold
    EXPECT_FALSE(bm.warn());
    bm.update(9.5f, 7.0f);   // now 3 s in → arms
    EXPECT_TRUE(bm.warn());
}

TEST(BatteryMonitor, ClearsAboveHysteresisImmediately) {
    // The monitor is bidirectional — the Supervisor owns the latching. A sag
    // that recovers must un-arm the flag, or the hold could never re-run.
    sup::BatteryMonitor bm(10.0f, 9.0f, 8.0f, 0.3f, 3.0f);
    bm.update(9.5f, 0.0f);
    bm.update(9.5f, 3.0f);
    ASSERT_TRUE(bm.warn());
    // Above threshold but within hysteresis band → stays armed.
    bm.update(10.2f, 3.5f);
    EXPECT_TRUE(bm.warn());
    // Above threshold + hysteresis (10.3) → clears at once, no hold.
    bm.update(10.4f, 3.6f);
    EXPECT_FALSE(bm.warn());
}

TEST(BatteryMonitor, DeeperDropArmsEveryRungItPasses) {
    sup::BatteryMonitor bm(10.0f, 9.0f, 8.0f, 0.3f, 3.0f);
    bm.update(8.5f, 0.0f);
    bm.update(8.5f, 3.0f);
    EXPECT_TRUE(bm.warn());     // below warning too
    EXPECT_TRUE(bm.fold());     // and below fold
    EXPECT_FALSE(bm.cutoff());  // but not yet the cutoff
}

TEST(BatteryMonitor, CutoffArmsUnderTheDeepestDrop) {
    sup::BatteryMonitor bm(10.0f, 9.0f, 8.0f, 0.3f, 3.0f);
    bm.update(7.0f, 0.0f);
    bm.update(7.0f, 3.0f);
    EXPECT_TRUE(bm.warn());
    EXPECT_TRUE(bm.fold());
    EXPECT_TRUE(bm.cutoff());
}

TEST(BatteryMonitor, DisabledShallowRungsDoNotBlockTheCutoff) {
    // Only the cutoff configured: a pack that falls straight past it must still
    // arm that rung, even though warning/fold are 0.0 (disabled).
    sup::BatteryMonitor bm(0.0f, 0.0f, 8.0f, 0.3f, 3.0f);
    bm.update(7.0f, 0.0f);
    bm.update(7.0f, 3.0f);
    EXPECT_FALSE(bm.warn());
    EXPECT_FALSE(bm.fold());
    EXPECT_TRUE(bm.cutoff());
}

// ── Input watchdog ──────────────────────────────────────────────────────────

TEST(Watchdog, FreshLinkedInputNotStale) {
    sup::Supervisor s(enabled_config());
    const auto d = s.step(healthy(5 * kSec));
    EXPECT_FALSE(d.input_stale);
}

TEST(Watchdog, StaleAfterTimeout) {
    sup::Supervisor s(enabled_config());
    auto o = healthy(5 * kSec);
    o.last_input_us = 5 * kSec - 600'000;  // 0.6 s old > 0.5 s timeout
    EXPECT_TRUE(s.step(o).input_stale);
}

TEST(Watchdog, JustUnderTimeoutStillFresh) {
    sup::Supervisor s(enabled_config());
    auto o = healthy(5 * kSec);
    o.last_input_us = 5 * kSec - 400'000;  // 0.4 s old < 0.5 s
    EXPECT_FALSE(s.step(o).input_stale);
}

TEST(Watchdog, DisconnectedIsStale) {
    sup::Supervisor s(enabled_config());
    auto o = healthy(5 * kSec);
    o.bt_connected = false;
    EXPECT_TRUE(s.step(o).input_stale);
}

TEST(Watchdog, NoFrameYetIsStale) {
    sup::Supervisor s(enabled_config());
    auto o = healthy(5 * kSec);
    o.last_input_us = 0;  // connected but never delivered a frame
    EXPECT_TRUE(s.step(o).input_stale);
}

// ── Safe-stop aggregate (force_zero) ────────────────────────────────────────

TEST(SafeStop, HealthyDoesNotForceZero) {
    sup::Supervisor s(enabled_config());
    EXPECT_FALSE(s.step(healthy(5 * kSec)).force_zero);
}

TEST(SafeStop, StaleInputForcesZero) {
    sup::Supervisor s(enabled_config());
    auto o = healthy(5 * kSec);
    o.last_input_us = 5 * kSec - 600'000;  // stale
    EXPECT_TRUE(s.step(o).force_zero);
}

TEST(SafeStop, WarningRungLeavesTheRobotDrivable) {
    sup::Supervisor s(enabled_config());
    // Rung 1 is a beep and nothing else — a force_zero here would strand the
    // robot wherever it ran flat.
    const auto d = latch_at(s, 9.5f, 2 * kSec);
    ASSERT_EQ(d.undervolt_stage, sup::UndervoltStage::kWarn);
    EXPECT_FALSE(d.input_stale);
    EXPECT_FALSE(d.force_zero);
    EXPECT_TRUE(d.relay_energized);  // and the rail stays up
}

TEST(SafeStop, FoldRungForcesZeroEvenWithFreshInput) {
    sup::Supervisor s(enabled_config());
    const auto d = latch_at(s, 8.5f, 2 * kSec);
    ASSERT_EQ(d.undervolt_stage, sup::UndervoltStage::kFold);
    EXPECT_FALSE(d.input_stale);  // link is fine…
    EXPECT_TRUE(d.force_zero);    // …but a spent pack stops the walk
}

// ── Undervoltage ladder ─────────────────────────────────────────────────────

TEST(Undervolt, DisabledThresholdsStayAtNone) {
    // Shipped default: every rung 0.0. A flat pack must not move the ladder.
    auto cfg = enabled_config();
    cfg.battery_warning_v = 0.0f;
    cfg.battery_fold_v = 0.0f;
    cfg.battery_cutoff_v = 0.0f;
    sup::Supervisor s(cfg);
    const auto d = latch_at(s, 0.0f, 2 * kSec);
    EXPECT_EQ(d.undervolt_stage, sup::UndervoltStage::kNone);
    EXPECT_FALSE(d.force_zero);
    EXPECT_TRUE(d.relay_energized);
}

TEST(Undervolt, FoldRungRequestsExactlyOneFold) {
    sup::Supervisor s(enabled_config());
    const auto armed = latch_at(s, 8.5f, 2 * kSec);
    ASSERT_EQ(armed.undervolt_stage, sup::UndervoltStage::kFold);
    EXPECT_TRUE(armed.request_fold);  // the edge
    // Later ticks must not re-request: the fold is already queued, and re-asking
    // each tick would stomp the operator's recovery.
    auto o = healthy(10 * kSec);
    o.battery_valid = true;
    o.battery_v = 8.5f;
    for (int i = 0; i < 5; ++i) {
        o.now_us = static_cast<uint64_t>(10 + i) * kSec;
        o.last_input_us = o.now_us;
        EXPECT_FALSE(s.step(o).request_fold) << "tick " << i;
    }
}

TEST(Undervolt, EscalatingToCutoffDoesNotReRequestTheFold) {
    sup::Supervisor s(enabled_config());
    ASSERT_TRUE(latch_at(s, 8.5f, 2 * kSec).request_fold);
    // Pack collapses past the cutoff: a second fold request is pointless, the
    // rail is about to be cut regardless of posture.
    const auto d = latch_at(s, 7.0f, 10 * kSec);
    EXPECT_EQ(d.undervolt_stage, sup::UndervoltStage::kCutoff);
    EXPECT_FALSE(d.request_fold);
}

TEST(Undervolt, LadderNeverDeEscalates) {
    sup::Supervisor s(enabled_config());
    ASSERT_EQ(latch_at(s, 7.0f, 2 * kSec).undervolt_stage,
              sup::UndervoltStage::kCutoff);
    // Cutting the rail unloads the pack and the voltage rebounds, here all the
    // way to healthy. The ladder must not follow it down, or the rail re-arms
    // into the same sag and drops the robot again.
    auto o = healthy(20 * kSec);
    o.battery_valid = true;
    o.battery_v = 12.0f;
    const auto d = s.step(o);
    EXPECT_EQ(d.undervolt_stage, sup::UndervoltStage::kCutoff);
    EXPECT_TRUE(d.undervolt_cutoff);
    EXPECT_FALSE(d.relay_energized);
}

TEST(Undervolt, CutoffCutsTheRailWithoutWaitingToFold) {
    sup::Supervisor s(enabled_config());
    ASSERT_TRUE(s.step(healthy(kSec)).relay_energized);
    // Straight past every rung while standing (never folded): the rail goes now,
    // whatever the posture.
    const auto d = latch_at(s, 7.0f, 2 * kSec);
    EXPECT_EQ(d.undervolt_stage, sup::UndervoltStage::kCutoff);
    EXPECT_FALSE(d.relay_energized);
}

// ── Relay-arming discipline ─────────────────────────────────────────────────

TEST(Relay, ArmsAtBootWhileFolded) {
    sup::Supervisor s(enabled_config());
    // Boot posture: linked pad, engine still FOLDED. Booting folded is not a
    // park, so the rail closes and the robot holds the folded pose under power
    // (the consumer staggers the servo energize leg by leg).
    auto o = healthy(kSec);
    o.folded = true;
    const auto d = s.step(o);
    EXPECT_TRUE(d.relay_energized);
    EXPECT_TRUE(s.relay_armed());
}

TEST(Relay, StaysArmedWhileHeldFolded) {
    sup::Supervisor s(enabled_config());
    auto o = healthy(kSec);
    o.folded = true;
    ASSERT_TRUE(s.step(o).relay_energized);
    // Level, not edge: sitting folded indefinitely must not drop the rail.
    for (int i = 2; i < 20; ++i) {
        o.now_us = static_cast<uint64_t>(i) * kSec;
        o.last_input_us = o.now_us;
        EXPECT_TRUE(s.step(o).relay_energized) << "tick " << i;
    }
}

TEST(Relay, DoesNotArmWithoutLink) {
    sup::Supervisor s(enabled_config());
    auto o = healthy(kSec);
    o.bt_connected = false;
    EXPECT_FALSE(s.step(o).relay_energized);
}

TEST(Relay, StaysArmedThroughStaleLink) {
    sup::Supervisor s(enabled_config());
    ASSERT_TRUE(s.step(healthy(kSec)).relay_energized);
    // Link goes stale but still connected — rail holds so the robot can settle.
    auto o = healthy(2 * kSec);
    o.last_input_us = 2 * kSec - kSec;  // 1 s stale
    EXPECT_TRUE(s.step(o).relay_energized);
}

TEST(Relay, DropsOnCleanFold) {
    sup::Supervisor s(enabled_config());
    ASSERT_TRUE(s.step(healthy(kSec)).relay_energized);  // standing, armed
    auto o = healthy(2 * kSec);  // FOLDING -> FOLDED: the park edge
    o.folded = true;
    EXPECT_FALSE(s.step(o).relay_energized);
}

TEST(Relay, ReArmsOnLeavingTheParkedFold) {
    sup::Supervisor s(enabled_config());
    ASSERT_TRUE(s.step(healthy(kSec)).relay_energized);
    auto folded = healthy(2 * kSec);
    folded.folded = true;
    ASSERT_FALSE(s.step(folded).relay_energized);  // parked, rail dropped
    // Start: the engine leaves FOLDED for INITIALIZE and the rail closes again.
    EXPECT_TRUE(s.step(healthy(3 * kSec)).relay_energized);
}

TEST(Relay, StaysDroppedWhileFaulted) {
    sup::Supervisor s(enabled_config());
    ASSERT_TRUE(s.step(healthy(kSec)).relay_energized);
    // Board over-current: the engine latches FAULT (not armable) and the rail
    // stays open until the operator's Start leaves FAULT.
    auto o = healthy(2 * kSec);
    o.fault = true;
    o.armable = false;
    EXPECT_FALSE(s.step(o).relay_energized);
    // Latch cleared on the board, but the engine is still in FAULT.
    auto cleared = healthy(3 * kSec);
    cleared.armable = false;
    EXPECT_FALSE(s.step(cleared).relay_energized);
    // Start recovers: engine leaves FAULT -> armable -> rail re-arms.
    EXPECT_TRUE(s.step(healthy(4 * kSec)).relay_energized);
}

TEST(Relay, FoldRungHoldsTheRailUntilTheRobotParks) {
    sup::Supervisor s(enabled_config());
    ASSERT_TRUE(s.step(healthy(kSec)).relay_energized);
    // Rung 2 folds; it must not cut the rail on the spot, or the legs go limp
    // mid-stance and the robot falls instead of folding.
    const auto armed = latch_at(s, 8.5f, 2 * kSec);
    ASSERT_EQ(armed.undervolt_stage, sup::UndervoltStage::kFold);
    ASSERT_TRUE(armed.request_fold);
    EXPECT_TRUE(armed.relay_energized) << "rail must survive the fold itself";
    // …and drops on the FOLDING -> FOLDED park edge, like any clean fold.
    auto folded = healthy(10 * kSec);
    folded.folded = true;
    folded.battery_valid = true;
    folded.battery_v = 8.5f;
    EXPECT_FALSE(s.step(folded).relay_energized);
}

TEST(Relay, WillNotReArmAfterTheFoldRung) {
    sup::Supervisor s(enabled_config());
    ASSERT_TRUE(s.step(healthy(kSec)).relay_energized);
    ASSERT_EQ(latch_at(s, 8.5f, 2 * kSec).undervolt_stage,
              sup::UndervoltStage::kFold);
    auto folded = healthy(10 * kSec);
    folded.folded = true;
    ASSERT_FALSE(s.step(folded).relay_energized);  // parked, rail dropped
    // Leaving the parked fold normally re-arms (ReArmsOnLeavingTheParkedFold) —
    // but not once the ladder has condemned the pack, even with the voltage
    // rebounded to healthy.
    auto stand = healthy(11 * kSec);
    stand.battery_valid = true;
    stand.battery_v = 12.0f;
    EXPECT_FALSE(s.step(stand).relay_energized);
}

TEST(Relay, WillNotArmWhileLadderIsLatched) {
    sup::Supervisor s(enabled_config());
    // Latch the fold rung while the engine is in FAULT (never armed).
    sup::Observation o;
    o.now_us = 2 * kSec;
    o.bt_connected = true;
    o.last_input_us = 2 * kSec;
    o.armable = false;
    o.folded = false;
    o.walking = false;
    o.battery_valid = true;
    o.battery_v = 8.0f;
    o.fault = false;
    s.step(o);
    o.now_us = 5 * kSec;
    o.last_input_us = o.now_us;
    s.step(o);
    // Now stand with a good link — but the ladder is latched → stays disarmed.
    auto stand = healthy(6 * kSec);
    stand.battery_valid = true;
    stand.battery_v = 8.0f;
    EXPECT_FALSE(s.step(stand).relay_energized);
}

// ── Status LED ──────────────────────────────────────────────────────────────

TEST(Led, SolidWhenWalking) {
    sup::Supervisor s(enabled_config());
    auto o = healthy(kSec);
    o.walking = true;
    EXPECT_EQ(s.step(o).led, sup::LedPattern::kSolid);
}

TEST(Led, SlowWhenIdleStanding) {
    sup::Supervisor s(enabled_config());
    EXPECT_EQ(s.step(healthy(kSec)).led, sup::LedPattern::kSlowBlink);
}

TEST(Led, FastOnBatteryFault) {
    sup::Supervisor s(enabled_config());
    const auto d = latch_at(s, 8.0f, 2 * kSec);
    ASSERT_TRUE(d.fault);
    EXPECT_EQ(d.led, sup::LedPattern::kFastBlink);
}

TEST(Led, FastOnTheWarningRungToo) {
    sup::Supervisor s(enabled_config());
    // Rung 1 leaves the robot drivable, so the LED is the only always-on signal
    // that something is wrong (the buzzer beeps once and stops).
    const auto d = latch_at(s, 9.5f, 2 * kSec);
    ASSERT_EQ(d.undervolt_stage, sup::UndervoltStage::kWarn);
    EXPECT_TRUE(d.fault);
    EXPECT_EQ(d.led, sup::LedPattern::kFastBlink);
}

TEST(Led, FastOnLostLinkAfterArming) {
    sup::Supervisor s(enabled_config());
    ASSERT_TRUE(s.step(healthy(kSec)).relay_energized);  // arm first
    auto o = healthy(2 * kSec);
    o.bt_connected = false;  // lost the pilot mid-operation
    const auto d = s.step(o);
    EXPECT_TRUE(d.fault);
    EXPECT_EQ(d.led, sup::LedPattern::kFastBlink);
}

TEST(Led, StaleWalkingIsNotSolid) {
    sup::Supervisor s(enabled_config());
    auto o = healthy(2 * kSec);
    o.walking = true;
    o.last_input_us = 2 * kSec - kSec;  // stale → command will be zeroed
    const auto d = s.step(o);
    EXPECT_TRUE(d.input_stale);
    EXPECT_NE(d.led, sup::LedPattern::kSolid);  // slow blink, not walking-solid
}

TEST(Led, ScanningAtBootIsSlowNotFault) {
    sup::Supervisor s(enabled_config());
    // Never linked, never armed: a calm slow blink, not the fault cadence.
    auto o = healthy(kSec);
    o.bt_connected = false;
    o.folded = true;
    const auto d = s.step(o);
    EXPECT_FALSE(d.fault);
    EXPECT_EQ(d.led, sup::LedPattern::kSlowBlink);
}

// ── Tick jitter ─────────────────────────────────────────────────────────────

TEST(Jitter, TracksIntervalsAndOverruns) {
    sup::Supervisor s(enabled_config());
    uint64_t t = 0;
    s.record_tick(t);  // first tick: no interval yet
    EXPECT_EQ(s.tick_stats().count, 0u);

    // Three nominal 20 ms ticks.
    for (int i = 0; i < 3; ++i) {
        t += 20'000;
        s.record_tick(t);
    }
    const auto& a = s.tick_stats();
    EXPECT_EQ(a.count, 3u);
    EXPECT_EQ(a.last_dt_us, 20'000u);
    EXPECT_EQ(a.min_dt_us, 20'000u);
    EXPECT_EQ(a.max_dt_us, 20'000u);
    EXPECT_EQ(a.overruns, 0u);

    // A stalled tick: 30 ms > 20 + 4 margin → one overrun, new max.
    t += 30'000;
    s.record_tick(t);
    const auto& b = s.tick_stats();
    EXPECT_EQ(b.overruns, 1u);
    EXPECT_EQ(b.max_dt_us, 30'000u);
    EXPECT_EQ(b.min_dt_us, 20'000u);
    EXPECT_EQ(b.count, 4u);
}

TEST(Jitter, WithinMarginIsNotOverrun) {
    sup::Supervisor s(enabled_config());
    uint64_t t = 0;
    s.record_tick(t);
    t += 23'000;  // 3 ms late, inside the 4 ms margin
    s.record_tick(t);
    EXPECT_EQ(s.tick_stats().overruns, 0u);
}
