// Hexapod face — core0 policy + core1 render (part 11). See face.hpp.

#include "face.hpp"

#include <optional>
#include <string>

#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/time.h"

#include "config_generated.hpp"
#include "gait/engine.hpp"

// Vendored eye core + pure policy library — the exact source the sim/ROS face
// runs, so the eyes rasterize bit-identically.
#include "EyeAnim.h"
#include "EyeRaster.h"
#include "Expression.h"
#include "FaceNames.h"
#include "IRenderer.h"
#include "u8g2.h"

#include "expression_policy.hpp"
#include "face_animation.hpp"
#include "face_animation_runner.hpp"

#include "Sh1122PanelPico.h"
#include "face_policy.hpp"

namespace face {
namespace {

// ── Cross-core render target ────────────────────────────────────────────────
// The whole payload core0 hands core1: the two policy targets EyeAnim eases
// toward, plus a one-shot blink request. POD — no std::string crosses cores.
struct FaceRenderTarget {
    Expression    expr = Expression::NEUTRAL;
    GazeDirection gaze = GazeDirection::CENTER;
    bool          blink_pending = false;
};

mutex_t          g_mutex;
FaceRenderTarget g_target;  // written by core0, read by core1

void publish_expr(Expression e) {
    mutex_enter_blocking(&g_mutex);
    g_target.expr = e;
    mutex_exit(&g_mutex);
}
void publish_gaze(GazeDirection g) {
    mutex_enter_blocking(&g_mutex);
    g_target.gaze = g;
    mutex_exit(&g_mutex);
}
void request_blink() {
    mutex_enter_blocking(&g_mutex);
    g_target.blink_pending = true;
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
hexa::display::PolicyConfig  g_cfg;
std::optional<hexa::display::FaceAnimationRunner> g_runner;

// core0 policy state
hexa::display::DisplayTarget g_last_target;
std::string                 g_animation_mode;  // current, tracked from selects
bool                        g_link_ever = false;
double                      g_next_policy_s = 0.0;

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

// ── core1: RNG, pixel sink, render loop ─────────────────────────────────────
// EyeAnim's injected RNG (idle-blink jitter). A plain xorshift with static
// state; touched only on core1, so no cross-core sharing.
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

// drawEye is pure in these fields, so bit-identical frames rasterize identically
// — skip the float-heavy raster + SPI flush when nothing changed.
bool same_frame(const eyes::AnimFrame& a, const eyes::AnimFrame& b) {
    return a.expr == b.expr && a.lid == b.lid && a.gx == b.gx && a.gy == b.gy;
}

void core1_entry() {
    static Sh1122PanelPico panel;
    panel.begin(g_panel_cfg);

    eyes::EyeAnim anim(&eye_rand);
    RenderState target{Expression::NEUTRAL, GazeDirection::CENTER};
    eyes::AnimFrame last_frame{};
    bool have_frame = false;

    const uint64_t period_us =
        static_cast<uint64_t>(1.0e6f / hexa::config::kFaceRenderHz);
    absolute_time_t next = get_absolute_time();

    while (true) {
        const uint64_t now_us = time_us_64();
        const FaceRenderTarget t = snapshot(/*clear_blink=*/true);
        target.expr = t.expr;
        target.gaze = t.gaze;
        if (t.blink_pending) anim.requestBlink();

        const eyes::AnimFrame f =
            anim.update(target, static_cast<uint32_t>(now_us / 1000));
        if (!have_frame || !same_frame(f, last_frame)) {
            last_frame = f;
            have_frame = true;
            panel.clearBuffer();
            u8g2_t* g = panel.u8g2();
            eyes::drawEye(f.expr, false, eyes::kEyeLX, f.lid, f.gx, f.gy,
                          &pixel_sink, g);
            eyes::drawEye(f.expr, true, eyes::kEyeRX, f.lid, f.gx, f.gy,
                          &pixel_sink, g);
            panel.present();
        }

        next = delayed_by_us(next, period_us);
        sleep_until(next);
    }
}

// ── core0: runner uniform sampler (idling rest-gap jitter) ───────────────────
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
                     [](GazeDirection g) { publish_gaze(g); },
                     []() { request_blink(); });

    multicore_launch_core1(&core1_entry);
}

void tick(const FaceState& state, double now_s) {
    if (!g_enabled) return;
    if (now_s < g_next_policy_s) return;
    g_next_policy_s =
        now_s + 1.0 / static_cast<double>(hexa::config::kFaceUpdateRateHz);

    // Breathing plays until the gamepad first pairs (the Pico's analog of the
    // ROS face's "no /gait/state yet — stack still initializing"). Once linked,
    // the engine state always drives the expression, even across a later drop.
    if (state.link_up) g_link_ever = true;
    if (state.animation_event) g_animation_mode = std::string(state.animation_name);

    hexa::display::PolicyInputs in;
    if (g_link_ever) in.gait_state = hexa::gait::state_value(state.engine_state);
    in.vx = state.vx;
    in.vy = state.vy;
    in.wz = state.wz;
    in.animation_mode = g_animation_mode;
    in.roll = state.roll;
    in.pitch = state.pitch;
    in.yaw = state.yaw;
    in.battery_low = state.battery_low;
    in.battery_critical = state.battery_critical;

    g_last_target = hexa::display::decide(in, g_cfg, g_last_target);
    const hexa::display::FaceAnimation* animation = g_runner->update(
        hexa::display::selectFaceAnimation(in, g_cfg), now_s,
        g_cfg.idling_start_delay_s);

    // Expression always comes from the policy; gaze too, unless a face animation
    // is active — then its steps own the gaze via the runner's sinks (mirrors
    // display_node's policyTick).
    publish_expr(g_last_target.expression);
    if (animation == nullptr) {
        publish_gaze(g_last_target.gaze);
    } else {
        g_runner->run(*animation, now_s);
    }
}

}  // namespace face
