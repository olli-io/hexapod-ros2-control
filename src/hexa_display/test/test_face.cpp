// Unit tests for the on-Pi face renderer. No ROS graph and no hardware: the
// panel runs headless (full buffer/dirty pipeline, no SPI/GPIO) and the eye
// core is driven directly, mirroring face_node's renderTick.

#include <cctype>
#include <string>

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
    eyes::drawEye(Expression::NEUTRAL, false, eyes::kEyeLX, 1.0f, 0, 0, pixelSink, g);
    EXPECT_TRUE(panel.present());  // first draw flushes
    EXPECT_EQ(panel.flushCount(), 1u);

    panel.clearBuffer();
    eyes::drawEye(Expression::NEUTRAL, false, eyes::kEyeLX, 1.0f, 0, 0, pixelSink, g);
    EXPECT_FALSE(panel.present());  // identical frame — no flush
    EXPECT_EQ(panel.flushCount(), 1u);

    panel.clearBuffer();
    eyes::drawEye(Expression::HAPPY, false, eyes::kEyeLX, 1.0f, 0, 0, pixelSink, g);
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
        eyes::drawEye(f.expr, false, eyes::kEyeLX, f.lid, f.gx, f.gy, pixelSink, g);
        eyes::drawEye(f.expr, true, eyes::kEyeRX, f.lid, f.gx, f.gy, pixelSink, g);
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
