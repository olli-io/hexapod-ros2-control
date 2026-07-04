// Bluetooth gamepad teleop — Bluepad32 (C platform API) → teleop_joy.yaml layout.
//
// The platform callbacks fire from the cyw43 background context; read() runs in
// the main loop. The shared snapshot is guarded by a critical section (disables
// IRQs + takes a spinlock), which is the correct cross-context primitive on the
// RP2350.
//
// NOTE: the callback wiring mirrors bluepad32/examples/pico_w/src/my_platform.c
// for the vendored Bluepad32 version. If your checkout renames a symbol
// (uni_bt_* helpers, MISC_BUTTON_* aliases) adjust the marked spots to match.

#include "bt_teleop.hpp"

#include <cstdio>
#include <cstring>

#include "pico/critical_section.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"  // time_us_64

#include "dbg.hpp"  // HEXA_DBG — BT event logs (gated by enable_usb_debugging)

extern "C" {
#include <uni.h>
}

namespace bt_teleop {
namespace {

// ── Shared snapshot (written by callbacks, read by the main loop) ───────────
critical_section_t g_lock;
bool g_locked = false;  // guards against read() before init()
int16_t g_axes[kNumAxes];
uint32_t g_buttons = 0;
bool g_connected = false;
uint64_t g_last_data_us = 0;

// The one controller we drive from. Bluepad32 can pair several; we bind to the
// first ready gamepad and ignore the rest (a hexapod has one pilot).
uni_hid_device_t* g_device = nullptr;

// ── Adapter helpers ─────────────────────────────────────────────────────────

int16_t clamp16(int32_t v) {
    if (v > kAxisMax) return kAxisMax;
    if (v < kAxisMin) return kAxisMin;
    return static_cast<int16_t>(v);
}

// Bluepad32 sticks are ~[-512, 512], left/up negative — the same sign
// convention the Linux joystick driver uses, so a straight scale into int16
// lands in the raw layout teleop_joy.yaml's `axis_signs` were tuned for. 512*64
// = 32768, clamped to 32767.
int16_t scale_stick(int32_t v) { return clamp16(v * 64); }

// Triggers: Bluepad32 brake/throttle are 0..1023. joy_node convention has the
// trigger axis rest at +1.0 and travel to -1.0 when pressed (map_joy reads
// value < 0.5 as pressed), so released -> +32767, fully pressed -> -32767.
int16_t scale_trigger(int32_t v) {
    if (v < 0) v = 0;
    if (v > 1023) v = 1023;
    return clamp16(kAxisMax - static_cast<int32_t>((static_cast<int64_t>(v) * 65534) / 1023));
}

// Safe idle: sticks/dpad centered, triggers released at +max, no buttons.
void fill_neutral(int16_t axes[kNumAxes], uint32_t& buttons) {
    for (int i = 0; i < kNumAxes; ++i) axes[i] = 0;
    axes[kL2] = kAxisMax;
    axes[kR2] = kAxisMax;
    buttons = 0;
}

// Map one gamepad frame into the teleop_joy.yaml raw layout.
void adapt_gamepad(const uni_gamepad_t& gp, int16_t axes[kNumAxes], uint32_t& buttons) {
    axes[kLeftStickX] = scale_stick(gp.axis_x);
    axes[kLeftStickY] = scale_stick(gp.axis_y);
    axes[kRightStickX] = scale_stick(gp.axis_rx);
    axes[kRightStickY] = scale_stick(gp.axis_ry);

    axes[kL2] = scale_trigger(gp.brake);
    axes[kR2] = scale_trigger(gp.throttle);

    // D-pad -> hat axes, encoded so (raw) * (axis_sign = -1) crosses the ±0.5
    // thresholds in joy_mapping.DPAD_DIRECTIONS the right way:
    //   up   -> dpad_y = -max    down  -> dpad_y = +max
    //   left -> dpad_x = +max    right -> dpad_x = -max
    int16_t dx = 0, dy = 0;
    if (gp.dpad & DPAD_UP) dy = kAxisMin;
    if (gp.dpad & DPAD_DOWN) dy = kAxisMax;
    if (gp.dpad & DPAD_LEFT) dx = kAxisMax;
    if (gp.dpad & DPAD_RIGHT) dx = kAxisMin;
    axes[kDpadX] = dx;
    axes[kDpadY] = dy;

    uint32_t b = 0;
    if (gp.buttons & BUTTON_A) b |= 1u << kA;
    if (gp.buttons & BUTTON_B) b |= 1u << kB;
    if (gp.buttons & BUTTON_X) b |= 1u << kX;
    if (gp.buttons & BUTTON_Y) b |= 1u << kY;
    if (gp.buttons & BUTTON_SHOULDER_L) b |= 1u << kL1;
    if (gp.buttons & BUTTON_SHOULDER_R) b |= 1u << kR1;
    // Select/Start live in the misc bitmask. (Older Bluepad32 spells these
    // MISC_BUTTON_BACK / MISC_BUTTON_HOME — rename here if your version does.)
    if (gp.misc_buttons & MISC_BUTTON_SELECT) b |= 1u << kSelect;
    if (gp.misc_buttons & MISC_BUTTON_START) b |= 1u << kStart;
    buttons = b;
}

void store_snapshot(const int16_t axes[kNumAxes], uint32_t buttons, bool connected_now) {
    critical_section_enter_blocking(&g_lock);
    memcpy(g_axes, axes, sizeof(g_axes));
    g_buttons = buttons;
    g_connected = connected_now;
    g_last_data_us = time_us_64();
    critical_section_exit(&g_lock);
}

// ── Bluepad32 platform callbacks (run in the background context) ────────────

void platform_init(int /*argc*/, const char** /*argv*/) {
    // Neutral snapshot so a read() before the first frame is safe.
    int16_t axes[kNumAxes];
    uint32_t buttons;
    fill_neutral(axes, buttons);
    store_snapshot(axes, buttons, /*connected_now=*/false);
}

void platform_on_init_complete() {
    // Accept new controller connections and start scanning for pads in pairing
    // mode. (_unsafe: must be called from the BT thread — this callback is.)
    uni_bt_enable_new_connections_unsafe(true);
    uni_bt_start_scanning_and_autoconnect_unsafe();
    HEXA_DBG("[bt] init complete — scanning for controllers\n");
}

void platform_on_device_connected(uni_hid_device_t* d) {
    HEXA_DBG("[bt] device connected: %p\n", static_cast<void*>(d));
}

void platform_on_device_disconnected(uni_hid_device_t* d) {
    HEXA_DBG("[bt] device disconnected: %p\n", static_cast<void*>(d));
    if (d == g_device) {
        g_device = nullptr;
        int16_t axes[kNumAxes];
        uint32_t buttons;
        fill_neutral(axes, buttons);
        store_snapshot(axes, buttons, /*connected_now=*/false);
    }
}

uni_error_t platform_on_device_ready(uni_hid_device_t* d) {
    // Bind to the first ready gamepad; reject additional controllers.
    if (g_device != nullptr && g_device != d) {
        HEXA_DBG("[bt] second controller rejected (already piloting %p)\n",
               static_cast<void*>(g_device));
        return UNI_ERROR_IGNORE_DEVICE;
    }
    g_device = d;
    HEXA_DBG("[bt] controller ready: %p\n", static_cast<void*>(d));
    return UNI_ERROR_SUCCESS;
}

void platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    if (d != g_device || ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD) {
        return;
    }
    int16_t axes[kNumAxes];
    uint32_t buttons;
    adapt_gamepad(ctl->gamepad, axes, buttons);
    store_snapshot(axes, buttons, /*connected_now=*/true);
}

const uni_property_t* platform_get_property(uni_property_idx_t /*idx*/) { return nullptr; }

void platform_on_oob_event(uni_platform_oob_event_t /*event*/, void* /*data*/) {}

uni_platform* get_platform() {
    static uni_platform plat;
    plat.name = const_cast<char*>("hexa-pico");
    plat.init = platform_init;
    plat.on_init_complete = platform_on_init_complete;
    plat.on_device_connected = platform_on_device_connected;
    plat.on_device_disconnected = platform_on_device_disconnected;
    plat.on_device_ready = platform_on_device_ready;
    plat.on_controller_data = platform_on_controller_data;
    plat.get_property = platform_get_property;
    plat.on_oob_event = platform_on_oob_event;
    return &plat;
}

}  // namespace

// ── Public interface ────────────────────────────────────────────────────────

bool init() {
    critical_section_init(&g_lock);
    g_locked = true;
    fill_neutral(g_axes, g_buttons);

    if (cyw43_arch_init() != 0) {
        HEXA_DBG("[bt] cyw43_arch_init failed\n");
        return false;
    }
    uni_platform_set_custom(get_platform());
    uni_init(0, nullptr);
    // No btstack_run_loop_execute() here: pico_cyw43_arch_none still runs the
    // threadsafe_background async context (it's the BT-only, CYW43_LWIP=0 arch),
    // so the BTstack run loop is driven in the background and the caller's
    // cooperative loop keeps running.
    return true;
}

void pump() {
    // No-op: BTstack is serviced in the background. Present so the main loop's
    // call site is stable if a future build switches to a polled cyw43 arch.
}

bool read(int16_t axes[kNumAxes], uint32_t& buttons) {
    if (!g_locked) {
        fill_neutral(axes, buttons);
        return false;
    }
    critical_section_enter_blocking(&g_lock);
    memcpy(axes, g_axes, sizeof(g_axes));
    buttons = g_buttons;
    const bool connected_now = g_connected;
    critical_section_exit(&g_lock);
    return connected_now;
}

bool connected() {
    if (!g_locked) return false;
    critical_section_enter_blocking(&g_lock);
    const bool c = g_connected;
    critical_section_exit(&g_lock);
    return c;
}

uint64_t last_data_us() {
    if (!g_locked) return 0;
    critical_section_enter_blocking(&g_lock);
    const uint64_t t = g_last_data_us;
    critical_section_exit(&g_lock);
    return t;
}

}  // namespace bt_teleop
