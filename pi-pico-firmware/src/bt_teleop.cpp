// Bluetooth gamepad teleop — Bluepad32 (C platform API) → teleop_joy.yaml layout.
//
// The whole cyw43/BTstack stack runs on CORE1: init() (on core0) launches core1,
// which brings cyw43 up on its own threadsafe_background async context bound to a
// core1 alarm pool, so every Bluetooth IRQ/servicing path (data arrival AND the
// timer-driven pairing/reconnect work) is enabled on core1 and never jitters
// core0's 200 Hz control loop. The Bluepad32 platform callbacks therefore fire
// from core1's background context; core0 only ever calls read()/connected()/
// last_data_us(). The shared snapshot is guarded by a critical section (disables
// IRQs + takes a spinlock), the correct cross-core primitive on the RP2350. The
// onboard status LED is a cyw43 SPI ioctl, so core1 owns it too — core0 hands the
// level over via set_led().
//
// NOTE: the callback wiring mirrors bluepad32/examples/pico_w/src/my_platform.c
// for the vendored Bluepad32 version. If your checkout renames a symbol
// (uni_bt_* helpers, MISC_BUTTON_* aliases) adjust the marked spots to match.

#include "bt_teleop.hpp"

#include <cstdio>
#include <cstring>

#include "pico/async_context_threadsafe_background.h"
#include "pico/critical_section.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/time.h"  // time_us_64, alarm pool, sleep_until

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

// Onboard status-LED level: written by core0 (set_led), read by core1's
// keep-alive loop (which owns the cyw43 GPIO). Single-writer / single-reader.
volatile bool g_led_on = false;

// Boot-handshake words core1 pushes to core0 over the inter-core FIFO.
constexpr uint32_t kBtInitOk = 1;
constexpr uint32_t kBtInitFail = 0;

// cyw43's async context, built on a core1 alarm pool so BTstack services on
// core1. Lives for the life of the program (core1_entry never returns).
async_context_threadsafe_background_t g_bt_ctx;

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
    // Deliberately NOT scanning at boot. Scanning is only needed to meet a NEW
    // pad: link keys live in a flash-backed TLV bank (pico_cyw43_driver wires
    // btstack_tlv_flash_bank to hci_set_link_key_db), so a bonded controller
    // reconnects on its own — uni_bt_allow_incoming_connections defaults to
    // true and we leave it there. Scanning permanently, as this used to, meant
    // any stranger's pad could bind first and become the pilot.
    //
    // Hold the front-panel button to open a pairing window (button.cpp).
    HEXA_DBG("[bt] init complete — bonded pads may reconnect; hold the button "
             "to pair a new one\n");
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

// cyw43/BTstack bring-up. Called from CORE1 by main.cpp's core1_entry.
// cyw43_arch_init() runs HERE (not on core0) and, crucially, against an async
// context built on an alarm pool created ON THIS CORE — so the driver's host-wake
// GPIO IRQ AND its timer/alarm worker (BTstack timers: connection setup, retries,
// reconnect) are both enabled on core1. Core0's control tick then never contends
// the cyw43 lock or takes a BT IRQ.
bool core1_init() {
    // Build cyw43's async context on a core1 alarm pool. This SDK has no
    // cyw43_arch_init_with_async_context(); instead cyw43_arch_init() reuses a
    // context previously registered with cyw43_arch_set_async_context() (default
    // otherwise). So: create the pool here (its hardware-alarm IRQ binds to
    // core1), init a threadsafe_background context on it, register it, then let
    // cyw43_arch_init() adopt it — pinning all cyw43/BTstack servicing to core1.
    alarm_pool_t* pool = alarm_pool_create_with_unused_hardware_alarm(16);
    async_context_threadsafe_background_config_t cfg =
        async_context_threadsafe_background_default_config();
    cfg.custom_alarm_pool = pool;
    const bool ctx_ok =
        pool != nullptr && async_context_threadsafe_background_init(&g_bt_ctx, &cfg);
    if (ctx_ok) cyw43_arch_set_async_context(&g_bt_ctx.core);

    if (!ctx_ok || cyw43_arch_init() != 0) {
        HEXA_DBG("[bt] cyw43_arch init failed (core1)\n");
        return false;
    }

    uni_platform_set_custom(get_platform());
    uni_init(0, nullptr);
    // No btstack_run_loop_execute(): the threadsafe_background async context on
    // this core services BTstack in the background off the core1 alarm pool.
    return true;
}

void core1_signal(bool ok) {
    multicore_fifo_push_blocking(ok ? kBtInitOk : kBtInitFail);
}

void core1_poll_led() {
    // The onboard LED is a cyw43 SPI ioctl, kept off core0. Only write on a
    // change, so a steady blink level costs no cyw43-bus traffic against
    // BTstack's own servicing on this core.
    static bool led_written = false;
    static bool led_last = false;
    const bool want = g_led_on;
    if (!led_written || want != led_last) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, want);
        led_last = want;
        led_written = true;
    }
}

bool init() {
    // The snapshot primitive + neutral state must come up BEFORE core1 launches,
    // so any read() from core0 is safe (returns neutral) no matter how far core1
    // has progressed. Core0-only; call before multicore_launch_core1.
    critical_section_init(&g_lock);
    g_locked = true;
    fill_neutral(g_axes, g_buttons);
    multicore_fifo_drain();
    return true;
}

bool wait_ready() {
    // Boot handshake: block until core1 reports the bring-up result. The FIFO pop
    // is also a memory barrier, so the cyw43/snapshot state core1 set up is
    // visible here. Preserves the synchronous "return false on cyw43 failure"
    // contract (main.cpp then loops forever on false).
    return multicore_fifo_pop_blocking() == kBtInitOk;
}

void start_pairing() { uni_bt_start_scanning_and_autoconnect_safe(); }

void stop_pairing() { uni_bt_stop_scanning_safe(); }

bool scanning() { return uni_bt_is_scanning(); }

void set_led(bool on) { g_led_on = on; }

void pump() {
    // No-op: BTstack is serviced in the background on core1. Present so the main
    // loop's call site is stable if a future build switches to a polled arch.
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
