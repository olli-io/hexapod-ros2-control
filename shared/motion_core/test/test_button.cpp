// The Pico front-panel button's pure halves: press/hold discrimination
// (button_fsm.hpp) and the screen strings (button_screens.hpp).
//
// Both live in the firmware tree but carry no Pico SDK, so they run here under
// the same host harness as the rest of the shared core.

#include <cstring>
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

// ── Screens ─────────────────────────────────────────────────────────────────

constexpr float kEmpty = 6.6f;  // hardware.yaml battery.empty_v
constexpr float kFull = 8.4f;   // hardware.yaml battery.full_v

TEST(ButtonScreens, PercentageSpansAndClamps) {
    EXPECT_EQ(button::batteryPercent(kEmpty, kEmpty, kFull), 0);
    EXPECT_EQ(button::batteryPercent(kFull, kEmpty, kFull), 100);
    EXPECT_EQ(button::batteryPercent(7.5f, kEmpty, kFull), 50);
    // Outside the endpoints clamps rather than going negative / over 100.
    EXPECT_EQ(button::batteryPercent(5.0f, kEmpty, kFull), 0);
    EXPECT_EQ(button::batteryPercent(9.9f, kEmpty, kFull), 100);
}

// Matches info_text.py's _round_half_away, not banker's rounding.
TEST(ButtonScreens, PercentageRoundsHalfAwayFromZero) {
    // 0.505 of the span -> 50.5 % -> 51, not 50.
    const float v = kEmpty + 0.505f * (kFull - kEmpty);
    EXPECT_EQ(button::batteryPercent(v, kEmpty, kFull), 51);
}

TEST(ButtonScreens, DegenerateSpanReadsZeroInsteadOfDividingByZero) {
    EXPECT_EQ(button::batteryPercent(7.5f, 8.4f, 8.4f), 0);
    EXPECT_EQ(button::batteryPercent(7.5f, 9.0f, 6.6f), 0);
}

TEST(ButtonScreens, BatteryTextCarriesPercentageAndVolts) {
    char buf[64];
    button::batteryScreenText(buf, sizeof(buf), true, 7.5f, kEmpty, kFull);
    const std::string s(buf);
    EXPECT_NE(s.find("50 %"), std::string::npos) << s;
    EXPECT_NE(s.find("7.5 V"), std::string::npos) << s;
    EXPECT_NE(s.find('\n'), std::string::npos) << "expected two lines: " << s;
}

TEST(ButtonScreens, BatteryTextSaysSoWhenTheReadingIsStale) {
    char buf[64];
    button::batteryScreenText(buf, sizeof(buf), false, 0.0f, kEmpty, kFull);
    const std::string s(buf);
    EXPECT_EQ(s.find('%'), std::string::npos)
        << "must not show a percentage without a reading: " << s;
}

// Every screen has to fit the panel: 4 lines of 30 characters (the budget
// text_screen.cpp wraps at; a wrap can push the last line off the panel).
TEST(ButtonScreens, ScreensFitThePanel) {
    auto checkFits = [](const char* s) {
        int lines = 1;
        int col = 0;
        for (const char* p = s; *p != '\0'; ++p) {
            if (*p == '\n') {
                ++lines;
                col = 0;
                continue;
            }
            EXPECT_LE(++col, 30) << "line too long in: " << s;
        }
        EXPECT_LE(lines, 4) << "too many lines in: " << s;
    };

    char buf[64];
    button::batteryScreenText(buf, sizeof(buf), true, 8.4f, kEmpty, kFull);
    checkFits(buf);
    button::batteryScreenText(buf, sizeof(buf), false, 0.0f, kEmpty, kFull);
    checkFits(buf);
    button::pairingScreenText(buf, sizeof(buf));
    checkFits(buf);
}

}  // namespace
