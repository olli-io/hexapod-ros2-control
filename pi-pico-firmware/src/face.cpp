// Hexapod face — core0 policy + core1 render (part 11). See face.hpp.

#include "face.hpp"

#include <cstring>
#include <optional>
#include <string>

#include "pico/mutex.h"
#include "pico/time.h"

#include "config_generated.hpp"
#include "gait/engine.hpp"

// Vendored eye core + pure policy library — the exact source the sim/ROS face
// runs, so the eyes rasterize bit-identically.
#include "EyeAnim.h"
#include "EyeRaster.h"
#include "Expression.h"
#include "IRenderer.h"
#include "u8g2.h"

#include "expression_policy.hpp"
#include "face_animation.hpp"
#include "face_animation_runner.hpp"
#include "frame_compare.hpp"
#include "text_screen.hpp"

#include "Sh1122PanelPico.h"
#include "face_policy.hpp"

namespace face {
namespace {

// Everything core0 hands core1. POD — no std::string crosses cores. Published as
// a whole struct once per policy tick: gaze_ease_ms/gaze_scale are set together
// with gaze from the same animation, so a torn read would drive a correct gaze
// at the wrong drift speed.
// Text is a fixed array, not a std::string: core1 copies the payload out under
// the mutex and must not allocate doing it.
constexpr std::size_t kTextMax = 64;

struct FaceRenderTarget {
    Expression    expr = Expression::NEUTRAL;
    GazeDirection gaze = GazeDirection::CENTER;
    std::uint32_t gaze_ease_ms = 0;
    float         gaze_scale = 1.0f;
    bool          blink_pending = false;
    char          text[kTextMax] = {};  // empty = eyes
};

// Leaf lock: never held across a flush, a cyw43 call, or bt_teleop's critical
// section, and never taken from an IRQ handler (mutex_enter_blocking asserts
// there). Both sides are thread context, hence a mutex — bt_teleop's own
// snapshot uses a critical section because that one IS written from BT callbacks.
mutex_t          g_mutex;
FaceRenderTarget g_target;  // written by core0, read by core1

// The eye half of the target. Text is owned by publish_text and survives this,
// so a policy tick during a text screen cannot wipe the screen out from under it.
void publish(const FaceRenderTarget& t) {
    mutex_enter_blocking(&g_mutex);
    const bool blink = g_target.blink_pending || t.blink_pending;
    char text[kTextMax];
    std::memcpy(text, g_target.text, kTextMax);
    g_target = t;
    g_target.blink_pending = blink;  // never drop a blink core1 has not consumed
    std::memcpy(g_target.text, text, kTextMax);
    mutex_exit(&g_mutex);
}

void publish_text(const char* text) {
    mutex_enter_blocking(&g_mutex);
    if (text == nullptr) {
        g_target.text[0] = '\0';
    } else {
        std::strncpy(g_target.text, text, kTextMax - 1);
        g_target.text[kTextMax - 1] = '\0';
    }
    mutex_exit(&g_mutex);
}

FaceRenderTarget snapshot(bool clear_blink) {
    mutex_enter_blocking(&g_mutex);
    FaceRenderTarget t = g_target;
    if (clear_blink) g_target.blink_pending = false;
    mutex_exit(&g_mutex);
    return t;
}

// ── Config (built once from the baked hexa::config) ─────────────────────────
bool                        g_enabled = false;
PanelConfigPico             g_panel_cfg;
hexa::display::PolicyConfig g_cfg;
std::optional<hexa::display::FaceAnimationRunner> g_runner;

// core0 policy state
hexa::display::DisplayTarget g_last_target;
std::string g_animation_mode;  // current, tracked from selects
double      g_next_policy_s = 0.0;
double      g_text_until_s = 0.0;  // 0 = no text screen pending
bool        g_text_active = false;

// Staging copy the runner's gaze sink writes into, so one publish() per tick
// carries the animation's gaze together with its drift parameters.
FaceRenderTarget g_staging;

void build_config() {
    g_cfg = buildPolicyConfig();  // shared with the host test (face_policy.hpp)

    const auto& p = hexa::config::kFacePanel;
    g_panel_cfg.spi = (p.spi_index == 0) ? spi0 : spi1;
    g_panel_cfg.spi_hz = p.spi_hz;
    g_panel_cfg.sck = p.sck;
    g_panel_cfg.mosi = p.mosi;
    g_panel_cfg.cs = p.cs;
    g_panel_cfg.dc = p.dc;
    g_panel_cfg.rst = p.rst;
}

// ── core1 ───────────────────────────────────────────────────────────────────
Sh1122PanelPico g_panel;

// EyeAnim's injected RNG (idle-blink jitter). Touched only on core1.
uint32_t eye_rand() {
    static uint32_t s = 0;
    if (s == 0) s = static_cast<uint32_t>(time_us_64()) | 1u;
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}

void pixel_sink(void* ctx, int x, int y) {
    u8g2_DrawPixel(static_cast<u8g2_t*>(ctx),
                   static_cast<u8g2_uint_t>(x), static_cast<u8g2_uint_t>(y));
}

std::optional<eyes::EyeAnim> g_anim;
eyes::AnimFrame g_last_frame{};
bool g_have_frame = false;
char g_drawn_text[kTextMax] = {};  // what core1 last rendered; "" = eyes
hexa::display::TextScreenConfig g_text_cfg;
// core0: the runner's rest-gap sampler.
double uniform_sample(double lo, double hi) {
    static uint32_t s = 0x9e3779b9u;
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return lo + (hi - lo) * (static_cast<double>(s) / 4294967296.0);
}

}  // namespace

void init() {
    g_enabled = hexa::config::kFaceEnabled;
    if (!g_enabled) return;

    build_config();
    mutex_init(&g_mutex);
    g_runner.emplace(&uniform_sample,
                     [](GazeDirection g) { g_staging.gaze = g; },
                     []() { g_staging.blink_pending = true; });
}

void show_text(const char* text, double now_s, double seconds) {
    if (!g_enabled) return;
    g_text_active = true;
    g_text_until_s = now_s + seconds;
    publish_text(text);
}

void tick(const FaceState& state, double now_s) {
    if (!g_enabled) return;

    // Text expiry is checked every control tick, not on the policy cadence —
    // a 10 Hz revert would leave the screen up to 100 ms past its dwell.
    if (g_text_active && now_s >= g_text_until_s) {
        g_text_active = false;
        publish_text(nullptr);
    }

    if (now_s < g_next_policy_s) return;
    g_next_policy_s =
        now_s + 1.0 / static_cast<double>(hexa::config::kFaceUpdateRateHz);

    if (state.animation_event) g_animation_mode = std::string(state.animation_name);

    hexa::display::PolicyInputs in;
    in.gait_state = hexa::gait::state_value(state.engine_state);
    in.vx = static_cast<double>(state.vx);
    in.vy = static_cast<double>(state.vy);
    in.wz = static_cast<double>(state.wz);
    in.animation_mode = g_animation_mode;
    in.x = static_cast<double>(state.x);
    in.y = static_cast<double>(state.y);
    in.roll = static_cast<double>(state.roll);
    in.pitch = static_cast<double>(state.pitch);
    in.yaw = static_cast<double>(state.yaw);
    in.battery_low = state.battery_low;
    in.battery_critical = state.battery_critical;
    // busy outranks the gait state, so the boot face is the spinner until the
    // first pair — no link-ever gate needed.
    in.busy = state.busy;

    g_last_target = hexa::display::decide(in, g_cfg, g_last_target);
    const hexa::display::FaceAnimation* animation = g_runner->update(
        hexa::display::selectFaceAnimation(in, g_cfg), now_s,
        g_cfg.idling_start_delay_s);

    // Expression always comes from the policy; gaze too, unless an animation is
    // active — then its steps own the gaze via the runner's sink and its easing
    // parameters ride along (mirrors display_node's policyTick).
    g_staging.expr = g_last_target.expression;
    if (animation == nullptr) {
        g_staging.gaze = g_last_target.gaze;
        g_staging.gaze_ease_ms = 0;
        g_staging.gaze_scale = 1.0f;
    } else {
        g_runner->run(*animation, now_s);
        g_staging.gaze_ease_ms =
            static_cast<std::uint32_t>(animation->gaze_ease_s() * 1000.0);
        g_staging.gaze_scale = static_cast<float>(animation->gaze_scale());
    }

    publish(g_staging);
    g_staging.blink_pending = false;
}

void core1_init() {
    if (!g_enabled) return;
    g_panel.begin(g_panel_cfg);
    g_anim.emplace(&eye_rand);
}

void render_tick() {
    if (!g_enabled || !g_anim) return;

    const FaceRenderTarget t = snapshot(/*clear_blink=*/true);

    if (t.text[0] != '\0') {
        // Text is static, so redraw only on change — the string compare stands
        // in for the eye path's sameFrame skip.
        if (std::strcmp(t.text, g_drawn_text) != 0) {
            std::memcpy(g_drawn_text, t.text, kTextMax);
            g_panel.clearBuffer();
            hexa::display::drawTextScreen(g_panel.u8g2(), t.text, g_text_cfg);
            g_panel.present();
        }
        g_have_frame = false;  // repaint the face when the screen ends
        return;
    }
    g_drawn_text[0] = '\0';

    if (t.blink_pending) g_anim->requestBlink();

    RenderState screen;
    screen.expr = t.expr;
    // Robot frame -> panel coordinates. A render-tick transform: the policy
    // speaks in the robot's left/right, the panel in its own.
    screen.gaze = hexa::display::toScreenGaze(t.gaze);
    screen.gazeEaseMs = t.gaze_ease_ms;
    screen.gazeScale = t.gaze_scale;

    const eyes::AnimFrame f =
        g_anim->update(screen, static_cast<uint32_t>(time_us_64() / 1000));
    if (g_have_frame && eyes::sameFrame(f, g_last_frame)) return;

    g_last_frame = f;
    g_have_frame = true;
    g_panel.clearBuffer();
    u8g2_t* g = g_panel.u8g2();
    eyes::drawEye(f.expr, false, eyes::kEyeLX, f.lid, f.gx, f.gy, f.phase,
                  &pixel_sink, g);
    eyes::drawEye(f.expr, true, eyes::kEyeRX, f.lid, f.gx, f.gy, f.phase,
                  &pixel_sink, g);
    g_panel.present();
}

std::uint64_t flush_count() { return g_panel.flushCount(); }
std::uint64_t last_flush_us() { return g_panel.lastFlushUs(); }
std::uint32_t actual_spi_hz() { return g_panel.actualSpiHz(); }

}  // namespace face
