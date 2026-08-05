// Unit tests for the on-Pi face renderer. No ROS graph and no hardware: the
// panel runs headless (full buffer/dirty pipeline, no SPI/GPIO) and the eye
// core is driven directly, mirroring face_node's renderTick.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iterator>
#include <set>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "EyeAnim.h"
#include "EyeRaster.h"
#include "Expression.h"
#include "FaceNames.h"
#include "Sh1122Panel.h"
#include "u8g2.h"

namespace {

face::PanelConfig headlessCfg() {
    face::PanelConfig cfg;
    cfg.headless = true;
    return cfg;
}

void pixelSink(void* ctx, int x, int y) {
    u8g2_DrawPixel(static_cast<u8g2_t*>(ctx),
                   static_cast<u8g2_uint_t>(x), static_cast<u8g2_uint_t>(y));
}

// Collects lit pixels instead of drawing them — for asserting on shape rather
// than on the framebuffer.
void setSink(void* ctx, int x, int y) {
    static_cast<std::set<std::pair<int, int>>*>(ctx)->insert({x, y});
}

uint32_t zeroRand() { return 0; }

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

}  // namespace

// Name-parity contract: the topic vocabulary the policy node publishes must
// map onto the firmware enum. Every value round-trips through its canonical
// name, case-insensitively; garbage is rejected.
TEST(FaceNames, ExpressionRoundTrip) {
    for (int i = 0; i < static_cast<int>(Expression::_COUNT); ++i) {
        const auto e = static_cast<Expression>(i);
        const std::string name = expressionName(e);
        EXPECT_EQ(face::parseExpression(name), e);
        EXPECT_EQ(face::parseExpression(lower(name)), e);
    }
    EXPECT_FALSE(face::parseExpression("nope").has_value());
    EXPECT_FALSE(face::parseExpression("").has_value());
}

TEST(FaceNames, GazeRoundTrip) {
    for (int i = 0; i < static_cast<int>(GazeDirection::_COUNT); ++i) {
        const auto g = static_cast<GazeDirection>(i);
        const std::string name = gazeName(g);
        EXPECT_EQ(face::parseGaze(name), g);
        EXPECT_EQ(face::parseGaze(lower(name)), g);
    }
    EXPECT_FALSE(face::parseGaze("sideways").has_value());
}

// The panel only pushes bytes when the framebuffer changed, so a static face
// costs no SPI traffic. Verified headless (no device touched).
TEST(Sh1122Panel, DirtyFlushSkipsUnchangedFrames) {
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(headlessCfg()));
    u8g2_t* g = panel.u8g2();

    panel.clearBuffer();
    eyes::drawEye(Expression::NEUTRAL, false, eyes::kEyeLX, 1.0f, 0, 0, 0.0f,
                  pixelSink, g);
    EXPECT_TRUE(panel.present());  // first draw flushes
    EXPECT_EQ(panel.flushCount(), 1u);

    panel.clearBuffer();
    eyes::drawEye(Expression::NEUTRAL, false, eyes::kEyeLX, 1.0f, 0, 0, 0.0f,
                  pixelSink, g);
    EXPECT_FALSE(panel.present());  // identical frame — no flush
    EXPECT_EQ(panel.flushCount(), 1u);

    panel.clearBuffer();
    eyes::drawEye(Expression::HAPPY, false, eyes::kEyeLX, 1.0f, 0, 0, 0.0f,
                  pixelSink, g);
    EXPECT_TRUE(panel.present());  // changed frame flushes
    EXPECT_EQ(panel.flushCount(), 2u);
}

// End-to-end of the render half (face_node's renderTick): EyeAnim → drawEye →
// present. A target change draws frames while the blink-through plays; once the
// face settles, further ticks with the same target stop flushing.
TEST(FaceRender, FlushesOnChangeThenSettles) {
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(headlessCfg()));
    u8g2_t* g = panel.u8g2();
    eyes::EyeAnim anim(&zeroRand);

    auto tick = [&](const RenderState& target, uint32_t nowMs) {
        const eyes::AnimFrame f = anim.update(target, nowMs);
        panel.clearBuffer();
        eyes::drawEye(f.expr, false, eyes::kEyeLX, f.lid, f.gx, f.gy, f.phase,
                      pixelSink, g);
        eyes::drawEye(f.expr, true, eyes::kEyeRX, f.lid, f.gx, f.gy, f.phase,
                      pixelSink, g);
        panel.present();
    };

    const RenderState happy{Expression::HAPPY, GazeDirection::CENTER};
    // Blink-through + settle over ~1 s at 60 Hz.
    for (uint32_t t = 0; t <= 1000; t += 16) tick(happy, t);
    const uint64_t settled = panel.flushCount();
    EXPECT_GT(settled, 0u);

    // Idle another ~0.5 s, well before the first autonomous idle blink
    // (~2.2 s): the settled face must not flush again.
    for (uint32_t t = 1016; t <= 1500; t += 16) tick(happy, t);
    EXPECT_EQ(panel.flushCount(), settled);
}

// Boot: nothing has been on screen yet, so there is no expression to blink away
// from. The first frame must show the target outright — the face boots SLEEPY
// (no /gait/state heard yet) and must not flash NEUTRAL on the way there.
TEST(FaceRender, FirstFrameAdoptsTargetWithoutBlink) {
    eyes::EyeAnim anim(&zeroRand);
    const RenderState sleepy{Expression::SLEEPY, GazeDirection::CENTER};
    for (uint32_t t = 0; t <= 500; t += 16) {
        const eyes::AnimFrame f = anim.update(sleepy, t);
        EXPECT_EQ(f.expr, Expression::SLEEPY) << "at t=" << t;
        EXPECT_FLOAT_EQ(f.lid, 1.0f) << "at t=" << t;  // lids never moved
    }
}

// ...but a change after that first frame still passes through the 260 ms blink,
// swapping expression at the midpoint.
TEST(FaceRender, LaterExpressionChangeStillBlinksThrough) {
    eyes::EyeAnim anim(&zeroRand);
    const RenderState sleepy{Expression::SLEEPY, GazeDirection::CENTER};
    const RenderState happy{Expression::HAPPY, GazeDirection::CENTER};

    anim.update(sleepy, 0);
    EXPECT_EQ(anim.update(happy, 16).expr, Expression::SLEEPY);  // blink starts
    const eyes::AnimFrame closing = anim.update(happy, 100);
    EXPECT_EQ(closing.expr, Expression::SLEEPY);
    EXPECT_LT(closing.lid, 1.0f);
    EXPECT_EQ(anim.update(happy, 300).expr, Expression::HAPPY);  // swapped
}

// SCANNING is the one expression that never settles: the spinner phase keeps
// stepping, so the panel keeps flushing for as long as it is on screen. This is
// the counterpart of FlushesOnChangeThenSettles above.
TEST(FaceRender, ScanningKeepsFlushing) {
    face::Sh1122Panel panel;
    ASSERT_TRUE(panel.begin(headlessCfg()));
    u8g2_t* g = panel.u8g2();
    eyes::EyeAnim anim(&zeroRand);

    const RenderState scanning{Expression::SCANNING, GazeDirection::CENTER};
    auto tick = [&](uint32_t nowMs) {
        const eyes::AnimFrame f = anim.update(scanning, nowMs);
        panel.clearBuffer();
        eyes::drawEye(f.expr, false, eyes::kEyeLX, f.lid, f.gx, f.gy, f.phase,
                      pixelSink, g);
        eyes::drawEye(f.expr, true, eyes::kEyeRX, f.lid, f.gx, f.gy, f.phase,
                      pixelSink, g);
        panel.present();
    };

    for (uint32_t t = 0; t <= 500; t += 16) tick(t);
    const uint64_t spun = panel.flushCount();
    for (uint32_t t = 516; t <= 1000; t += 16) tick(t);
    EXPECT_GT(panel.flushCount(), spun);
}

// The spinner is the NEUTRAL ring minus a gap: strictly fewer pixels, all of
// them on the same circle, and the lit set sweeps round as the phase advances.
TEST(EyeRaster, ScanningIsAnArcOfTheNeutralRing) {
    auto pixels = [](Expression e, float phase) {
        std::set<std::pair<int, int>> out;
        eyes::drawEye(e, false, eyes::kEyeLX, 1.0f, 0, 0, phase, setSink, &out);
        return out;
    };
    // Distance from the eye center, for "is this pixel on the ring's circle".
    auto radius = [](const std::pair<int, int>& p) {
        return std::hypot(p.first - eyes::kEyeLX, p.second - eyes::kCY);
    };

    const auto ring = pixels(Expression::NEUTRAL, 0.0f);
    const auto arc0 = pixels(Expression::SCANNING, 0.0f);
    const auto arc_half = pixels(Expression::SCANNING, 0.5f);

    EXPECT_FALSE(arc0.empty());
    EXPECT_LT(arc0.size(), ring.size());
    // Same circle, same 3 px brush — the arc samples its own angle grid, so the
    // tolerance is the brush width rather than pixel-exact set membership.
    for (const auto& p : ring) EXPECT_NEAR(radius(p), 18.0, 2.0);
    for (const auto& p : arc0) EXPECT_NEAR(radius(p), 18.0, 2.0);
    for (const auto& p : arc_half) EXPECT_NEAR(radius(p), 18.0, 2.0);

    // kSpinSweepRad is well under half a turn, so arcs half a turn apart share
    // no pixel at all: the spinner really moves rather than pulsing in place.
    ASSERT_LT(eyes::kSpinSweepRad, 3.14f);
    std::set<std::pair<int, int>> both;
    std::set_intersection(arc0.begin(), arc0.end(), arc_half.begin(), arc_half.end(),
                          std::inserter(both, both.begin()));
    EXPECT_TRUE(both.empty());
}
