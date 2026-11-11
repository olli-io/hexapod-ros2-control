// Unit tests for the greyscale face. No ROS graph and no hardware: EyesFace is
// pure, and the text buffer is a headless u8g2 panel.

#include <array>
#include <cmath>
#include <string>
#include <stdexcept>

#include <gtest/gtest.h>

#include "EyeAnim.h"
#include "Expression.h"
#include "Sh1122Panel.h"
#include "eyes.hpp"
#include "eyes_face.hpp"
#include "text_screen.hpp"

namespace {

using hexa::eyes_cpp::EyesFace;
using hexa::eyes_cpp::kExpressionCount;

std::array<std::string, kExpressionCount> allMoods(const std::string& mood) {
    std::array<std::string, kExpressionCount> a;
    a.fill(mood);
    return a;
}

eyes::AnimFrame frame(Expression e, float lid = 1.0f, int gx = 0, int gy = 0) {
    eyes::AnimFrame f{};
    f.expr = e;
    f.lid = lid;
    f.gx = gx;
    f.gy = gy;
    return f;
}

int lit(const Image& im) {
    int n = 0;
    for (uint8_t p : im.px) n += p != 0;
    return n;
}

}  // namespace

TEST(EyesFace, RejectsAnUnknownMoodAndNamesIt) {
    auto moods = allMoods("normal");
    moods[static_cast<size_t>(Expression::GREEDY)] = "cool";
    try {
        EyesFace face(moods);
        FAIL() << "expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        const std::string what = e.what();
        EXPECT_NE(what.find("cool"), std::string::npos);
        EXPECT_NE(what.find("GREEDY"), std::string::npos);
        EXPECT_NE(what.find("thug-life"), std::string::npos);   // lists the valid ones
    }
}

TEST(EyesFace, AcceptsEveryMoodTheRendererHas) {
    for (const auto& m : kMoods) EXPECT_TRUE(hexa::eyes_cpp::isMood(m)) << m;
    for (const auto& m : kScanning) EXPECT_TRUE(hexa::eyes_cpp::isMood(m)) << m;
    EXPECT_FALSE(hexa::eyes_cpp::isMood("neutral"));
}

TEST(EyesFace, DrawsTheMoodMappedToTheExpression) {
    auto moods = allMoods("normal");
    moods[static_cast<size_t>(Expression::HAPPY)] = "happy";
    EyesFace face(moods);
    const Image happy = face.render(frame(Expression::HAPPY, 1.0f, 3, -2), 10.0);
    const Image want = draw_eyes(3, -2, 1.0, "happy", 0.0, 0.0);
    EXPECT_EQ(happy.px, want.px);
    const Image neutral = face.render(frame(Expression::NEUTRAL, 1.0f, 3, -2), 10.1);
    EXPECT_NE(neutral.px, happy.px);
}

TEST(EyesFace, LidAndGazeReachTheRenderer) {
    EyesFace face(allMoods("normal"));
    const Image open = face.render(frame(Expression::NEUTRAL, 1.0f), 1.0);
    const Image shut = face.render(frame(Expression::NEUTRAL, 0.08f), 1.1);
    EXPECT_GT(lit(open), 2 * lit(shut));
    const Image left = face.render(frame(Expression::NEUTRAL, 1.0f, -10, 0), 1.2);
    EXPECT_EQ(left.px, draw_eyes(-10, 0, 1.0, "normal", 0.0, 0.0).px);
}

TEST(EyesFace, AnimatedMoodRestartsItsClockWhenTheMoodChanges) {
    auto moods = allMoods("normal");
    moods[static_cast<size_t>(Expression::GREEDY)] = "thug-life";
    EyesFace face(moods);
    face.render(frame(Expression::NEUTRAL), 100.0);
    // GREEDY appears at 150 s: the glasses must start from the bare-eyes step,
    // not from wherever the 9 s loop happens to be.
    const Image first = face.render(frame(Expression::GREEDY), 150.0);
    EXPECT_EQ(first.px, draw_eyes(0, 0, 1.0, "thug-life", 0.0, 0.0).px);
    const Image later = face.render(frame(Expression::GREEDY), 154.0);   // glasses on
    EXPECT_EQ(later.px, draw_eyes(0, 0, 1.0, "thug-life", std::fmod(4.0 / 1.2, 1.0), 4.0).px);
    EXPECT_NE(later.px, first.px);
}

TEST(EyesFace, TextBufferComesAcrossPixelForPixel) {
    face::PanelConfig cfg;
    cfg.headless = true;
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(cfg));
    panel.clearBuffer();
    hexa::display::drawTextScreen(panel.u8g2(), "Hi", hexa::display::TextScreenConfig{});
    const Image im = EyesFace::fromU8g2(panel.u8g2());
    ASSERT_EQ(im.w, 256);
    ASSERT_EQ(im.h, 64);
    int want = 0;
    for (int y = 0; y < 64; ++y)
        for (int x = 0; x < 256; ++x) {
            const bool on = hexa::display::textScreenPixel(panel.u8g2(), x, y);
            want += on;
            EXPECT_EQ(im.at(x, y), on ? 255 : 0) << x << "," << y;
        }
    EXPECT_GT(want, 20);   // the text actually drew something
}
