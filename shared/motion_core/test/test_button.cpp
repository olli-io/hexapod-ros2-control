// The Pico front-panel button's pure halves: press/hold discrimination
// (button_fsm.hpp) and the screen sequencer + text (button_screens.hpp).
//
// Both live in the firmware tree but carry no Pico SDK, so they run here under
// the same host harness as the rest of the shared core.

#include <string>

#include <gtest/gtest.h>

#include "button_fsm.hpp"
#include "button_screens.hpp"

namespace {

constexpr float kHoldS = 3.0f;
constexpr float kDebounceS = 0.03f;

button::PressDetector detector() { return {kHoldS, kDebounceS}; }

// Poll at 200 Hz (the control tick) from `from_s` to `to_s`, returning the
// first non-kNone event seen. Mirrors how button.cpp actually drives it.
button::Event pollUntil(button::PressDetector& d, bool down, double from_s,
                        double to_s) {
    button::Event seen = button::Event::kNone;
    for (double t = from_s; t <= to_s; t += 0.005) {
        const button::Event e = d.update(down, t);
        if (e != button::Event::kNone && seen == button::Event::kNone) seen = e;
    }
    return seen;
}

TEST(ButtonFsm, IdleProducesNothing) {
    auto d = detector();
    EXPECT_EQ(pollUntil(d, false, 0.0, 10.0), button::Event::kNone);
    EXPECT_FALSE(d.down());
}

TEST(ButtonFsm, ShortPressFiresOnRelease) {
    auto d = detector();
    // Press and hold well short of hold_s — nothing yet.
    EXPECT_EQ(pollUntil(d, true, 0.0, 0.5), button::Event::kNone);
    EXPECT_TRUE(d.down());
    // Release fires kPress.
    EXPECT_EQ(pollUntil(d, false, 0.6, 1.0), button::Event::kPress);
}

TEST(ButtonFsm, HoldFiresMidPressAtHoldSeconds) {
    auto d = detector();
    // Nothing before hold_s...
    EXPECT_EQ(pollUntil(d, true, 0.0, kHoldS - 0.1), button::Event::kNone);
    // ...then kHold, without waiting for the release.
    EXPECT_EQ(pollUntil(d, true, kHoldS - 0.1, kHoldS + 0.1),
              button::Event::kHold);
}

// The property that keeps "press = battery" and "hold = pair" from both firing
// on one long press.
TEST(ButtonFsm, HoldSuppressesThePressOnRelease) {
    auto d = detector();
    ASSERT_EQ(pollUntil(d, true, 0.0, kHoldS + 0.1), button::Event::kHold);
    EXPECT_EQ(pollUntil(d, false, kHoldS + 0.2, kHoldS + 1.0),
              button::Event::kNone);
}

TEST(ButtonFsm, HoldFiresOnlyOncePerPress) {
    auto d = detector();
    ASSERT_EQ(pollUntil(d, true, 0.0, kHoldS + 0.1), button::Event::kHold);
    // Keep holding for a long time — no repeat.
    EXPECT_EQ(pollUntil(d, true, kHoldS + 0.2, kHoldS + 20.0),
              button::Event::kNone);
}

TEST(ButtonFsm, ContactBounceOnPressDoesNotDoubleFire) {
    auto d = detector();
    // Press at t=0, then chatter inside the debounce window.
    ASSERT_EQ(d.update(true, 0.0), button::Event::kNone);
    for (double t = 0.001; t < kDebounceS; t += 0.002) {
        EXPECT_EQ(d.update(false, t), button::Event::kNone) << "bounce at " << t;
        EXPECT_EQ(d.update(true, t + 0.001), button::Event::kNone);
    }
    // The line settles high; a later clean release is still one kPress.
    EXPECT_EQ(pollUntil(d, true, kDebounceS, 0.5), button::Event::kNone);
    EXPECT_EQ(pollUntil(d, false, 0.6, 1.0), button::Event::kPress);
}

// A press shorter than the debounce lockout must still register — it is simply
// recognized when the lockout expires, not dropped.
TEST(ButtonFsm, PressShorterThanDebounceStillCounts) {
    auto d = detector();
    ASSERT_EQ(d.update(true, 0.0), button::Event::kNone);
    EXPECT_EQ(pollUntil(d, false, 0.005, 0.2), button::Event::kPress);
}

TEST(ButtonFsm, TwoSeparatePressesFireTwice) {
    auto d = detector();
    ASSERT_EQ(pollUntil(d, true, 0.0, 0.3), button::Event::kNone);
    ASSERT_EQ(pollUntil(d, false, 0.4, 0.7), button::Event::kPress);
    ASSERT_EQ(pollUntil(d, true, 0.8, 1.1), button::Event::kNone);
    EXPECT_EQ(pollUntil(d, false, 1.2, 1.5), button::Event::kPress);
}

// A switch jammed down at boot must not fire a hold; it needs a release first.
TEST(ButtonFsm, HeldAtBootIsIgnoredUntilReleased) {
    button::PressDetector d(kHoldS, kDebounceS, /*initially_down=*/true);
    EXPECT_EQ(pollUntil(d, true, 0.0, kHoldS * 3), button::Event::kNone);
    EXPECT_EQ(pollUntil(d, false, kHoldS * 3, kHoldS * 3 + 0.5),
              button::Event::kNone);
    // After the release it works as normal.
    ASSERT_EQ(pollUntil(d, true, 10.0, 10.3), button::Event::kNone);
    EXPECT_EQ(pollUntil(d, false, 10.4, 10.7), button::Event::kPress);
}

// ── Sequencer ───────────────────────────────────────────────────────────────

using button::Button;
using button::Screen;

constexpr float kScreenS = 5.0f;
constexpr float kScanS = 60.0f;

button::ScreenSequencer sequencer() { return {kScreenS, kScanS}; }

// One short press: down edge then release.
void tap(button::ScreenSequencer& s, Button b, double t) {
    s.press(b, t);
    s.release(b, t + 0.1);
}

void startScan(button::ScreenSequencer& s, double t) {
    s.press(Button::kBluetooth, t);
    s.hold(Button::kBluetooth, t + kHoldS);
}

TEST(ButtonSequencer, EachPressTogglesItsOwnScreen) {
    auto s = sequencer();
    tap(s, Button::kBluetooth, 0.0);
    EXPECT_EQ(s.screen(), Screen::kBluetooth);
    tap(s, Button::kBluetooth, 1.0);
    EXPECT_EQ(s.screen(), Screen::kNone);
    tap(s, Button::kInfo, 2.0);
    EXPECT_EQ(s.screen(), Screen::kBattery);
    tap(s, Button::kInfo, 3.0);
    EXPECT_EQ(s.screen(), Screen::kNone);
}

TEST(ButtonSequencer, TheOtherButtonSwapsTheScreen) {
    auto s = sequencer();
    tap(s, Button::kInfo, 0.0);
    tap(s, Button::kBluetooth, 1.0);
    EXPECT_EQ(s.screen(), Screen::kBluetooth);
    tap(s, Button::kInfo, 2.0);
    EXPECT_EQ(s.screen(), Screen::kBattery);
}

TEST(ButtonSequencer, ScreenTimesOut) {
    auto s = sequencer();
    tap(s, Button::kInfo, 0.0);
    s.tick(0.1 + kScreenS - 0.01);
    EXPECT_EQ(s.screen(), Screen::kBattery);
    s.tick(0.1 + kScreenS);
    EXPECT_EQ(s.screen(), Screen::kNone);
}

TEST(ButtonSequencer, BluetoothHoldStartsAScanThatIgnoresTheScreenTimeout) {
    auto s = sequencer();
    startScan(s, 0.0);
    EXPECT_EQ(s.screen(), Screen::kScanning);
    s.tick(kHoldS + kScreenS + 1.0);
    EXPECT_EQ(s.screen(), Screen::kScanning);
    s.tick(kHoldS + kScanS);
    EXPECT_EQ(s.screen(), Screen::kNone);
}

// No network on the Pico, so the info hold has nothing to start.
TEST(ButtonSequencer, InfoHoldDoesNothing) {
    auto s = sequencer();
    s.press(Button::kInfo, 0.0);
    s.hold(Button::kInfo, kHoldS);
    EXPECT_EQ(s.screen(), Screen::kNone);
}

TEST(ButtonSequencer, HoldFromAScreenScans) {
    auto s = sequencer();
    tap(s, Button::kInfo, 0.0);
    startScan(s, 1.0);
    EXPECT_EQ(s.screen(), Screen::kScanning);
}

// The cancelling press must not go on to open its button's screen.
TEST(ButtonSequencer, EitherPressCancelsAScanAndItsReleaseIsSwallowed) {
    for (Button b : {Button::kInfo, Button::kBluetooth}) {
        auto s = sequencer();
        startScan(s, 0.0);
        s.press(b, 5.0);
        EXPECT_EQ(s.screen(), Screen::kNone);
        s.release(b, 5.1);
        EXPECT_EQ(s.screen(), Screen::kNone);
        // The next press works as normal.
        tap(s, b, 6.0);
        EXPECT_NE(s.screen(), Screen::kNone);
    }
}

TEST(ButtonSequencer, ControllerConnectingEndsTheScan) {
    auto s = sequencer();
    startScan(s, 0.0);
    s.controllerSeen(5.0);
    EXPECT_EQ(s.screen(), Screen::kNone);
}

TEST(ButtonSequencer, ControllerConnectingLeavesTheScreenUp) {
    auto s = sequencer();
    tap(s, Button::kBluetooth, 0.0);
    s.controllerSeen(1.0);
    EXPECT_EQ(s.screen(), Screen::kBluetooth);
}

// ── Screen text ─────────────────────────────────────────────────────────────

constexpr float kEmpty = 6.6f;  // hardware.yaml battery.empty_v
constexpr float kFull = 8.4f;   // hardware.yaml battery.full_v

std::string text(Screen screen, const char* controller = "",
                 bool battery_valid = false, float battery_v = 0.0f) {
    button::ScreenInputs in;
    in.controller = controller;
    in.battery_valid = battery_valid;
    in.battery_v = battery_v;
    in.battery_empty_v = kEmpty;
    in.battery_full_v = kFull;
    in.hold_s = kHoldS;
    char buf[96];
    button::screenText(buf, sizeof(buf), screen, in);
    return buf;
}

TEST(ButtonScreens, PercentageSpansAndClamps) {
    EXPECT_EQ(button::batteryPercent(kEmpty, kEmpty, kFull), 0);
    EXPECT_EQ(button::batteryPercent(kFull, kEmpty, kFull), 100);
    EXPECT_EQ(button::batteryPercent(7.5f, kEmpty, kFull), 50);
    EXPECT_EQ(button::batteryPercent(5.0f, kEmpty, kFull), 0);
    EXPECT_EQ(button::batteryPercent(9.9f, kEmpty, kFull), 100);
}

// Matches info_text.py's _round_half_away, not banker's rounding.
TEST(ButtonScreens, PercentageRoundsHalfAwayFromZero) {
    const float v = kEmpty + 0.505f * (kFull - kEmpty);
    EXPECT_EQ(button::batteryPercent(v, kEmpty, kFull), 51);
}

TEST(ButtonScreens, DegenerateSpanReadsZeroInsteadOfDividingByZero) {
    EXPECT_EQ(button::batteryPercent(7.5f, 8.4f, 8.4f), 0);
    EXPECT_EQ(button::batteryPercent(7.5f, 9.0f, 6.6f), 0);
}

// Strings match hexa_buttons info_text.py.
TEST(ButtonScreens, BatteryScreenText) {
    EXPECT_EQ(text(Screen::kBattery, "", true, 7.5f), "Battery -> 50 %  ( 7.5 V )");
    EXPECT_EQ(text(Screen::kBattery, "", false), "Battery -> -- %  ( --.- V )");
}

TEST(ButtonScreens, ControllerScreenText) {
    EXPECT_EQ(text(Screen::kBluetooth, ""),
              "No connected controllers\nHold 3 seconds to pair");
    EXPECT_EQ(text(Screen::kBluetooth, "Wireless Controller"),
              "Connected to:\nWireless Controller\nHold 3 seconds to pair");
}

// The spinners are the face, so scanning (and none) hand the panel back.
TEST(ButtonScreens, NonTextScreensAreEmpty) {
    EXPECT_EQ(text(Screen::kNone, "pad", true, 7.5f), "");
    EXPECT_EQ(text(Screen::kScanning, "pad", true, 7.5f), "");
}

// Every screen has to fit the panel: 4 lines of 30 characters (the budget
// text_screen.cpp wraps at; a wrap can push the last line off the panel).
TEST(ButtonScreens, ScreensFitThePanel) {
    auto checkFits = [](const std::string& s) {
        int lines = 1;
        int col = 0;
        for (char c : s) {
            if (c == '\n') {
                ++lines;
                col = 0;
                continue;
            }
            EXPECT_LE(++col, button::kLineBudget) << "line too long in: " << s;
        }
        EXPECT_LE(lines, 4) << "too many lines in: " << s;
    };
    checkFits(text(Screen::kBattery, "", true, 12.6f));
    checkFits(text(Screen::kBattery, "", false));
    checkFits(text(Screen::kBluetooth, ""));
    checkFits(text(Screen::kBluetooth,
                   "A controller name far longer than one panel line"));
}

}  // namespace
