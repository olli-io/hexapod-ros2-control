// Pico 2 W hexapod firmware — live gait + posture pipeline (plan parts 01-10).
//
// Brings up the toolchain-critical surfaces every later part depends on:
//   - USB-CDC stdio for debug logging,
//   - cyw43_arch (onboard LED; Bluetooth via Bluepad32 in part 02),
//   - a time_us_64()-based 200 Hz cooperative control loop.
//
// Part 02 adds Bluetooth teleop (bt_teleop). Part 03 adds the Servo 2040 slave
// link (servo_out) over the Chica UART. Parts 05-09 build up the control brain
// (gait, IK, velocity shaping, posture, failsafes).
//
// Part 10 (Tier 2) factors that brain out of this file into the target-agnostic
// hexa::pipeline::Pipeline, so the exact same tick source drives the Pico here
// AND the Gazebo bridge (hexa_pico_bridge) with no #ifdefs — a link-time swap of
// the hardware seam. main.cpp is now the Pico composition of that seam: it owns
// the input source (bt_teleop over Bluepad32/BTstack), the servo sink (servo_out
// over the Chica UART), the clock (time_us_64), the onboard LED, and all the
// USB-CDC logging. Each 200 Hz tick it samples the seam into a TickInput, runs
// Pipeline::tick(), and applies the TickResult — command the 18 joints, drive
// the relay + status LED, and log the teleop / gait / safety events.
//
// The engine cold-starts FOLDED and stays there (holding the folded pose) until
// the user presses the init button, so an unpaired or idle controller never
// moves the legs. Safety: the legs move to the standing stance during INITIALIZE
// and then walk — bench the robot on a stand with the feet clear before flashing.

#include <malloc.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "bt_teleop.hpp"
#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "leg_index.hpp"
#include "pipeline.hpp"
#include "servo_out.hpp"
#include "supervisor.hpp"

namespace {

// The 200 Hz tick geometry is the pipeline's shared contract (5 ms period, 1 ms
// overrun slack, 0.005 s dt). Reuse those so the scheduler here and the engine /
// supervisor inside the pipeline never drift apart.
using hexa::pipeline::kDt;
using hexa::pipeline::kTickPeriodUs;

constexpr uint64_t kHeartbeatUs = 1'000'000;  // 1 s
constexpr uint64_t kJoyDumpUs = 250'000;      // 4 Hz teleop dump (part 02 verify)

// Status-LED blink half-periods (one LED toggle per interval).
constexpr uint64_t kLedSlowUs = 500'000;  // 1 Hz — idle / stand / scanning
constexpr uint64_t kLedFastUs = 100'000;  // 5 Hz — fault / low battery / link lost

// ── Free-heap probe (RP2350 newlib) ────────────────────────────────────────
// The forked engine keys legs by std::string and allocates per tick (see the
// overview's std::map note). Track free heap over a soak to catch fragmentation
// drift before it exhausts. __StackLimit / end are the linker's heap window.
extern "C" char __StackLimit;  // top of the heap arena
extern "C" char end;           // bottom of the heap (first byte after .bss)

uint32_t heap_total_bytes() {
  return static_cast<uint32_t>(&__StackLimit - &end);
}
uint32_t heap_free_bytes() {
  return heap_total_bytes() - static_cast<uint32_t>(mallinfo().uordblks);
}

const char* mode_name(hexa::teleop::Mode m) {
  switch (m) {
    case hexa::teleop::Mode::Posture: return "posture";
    case hexa::teleop::Mode::Gait: return "gait";
    case hexa::teleop::Mode::Animation: return "animation";
  }
  return "?";
}

// Turn a status-LED pattern into an on/off level for the current wall time.
bool led_level(hexa::supervisor::LedPattern p, uint64_t now_us) {
  using P = hexa::supervisor::LedPattern;
  switch (p) {
    case P::kSolid: return true;
    case P::kFastBlink: return (now_us / kLedFastUs) % 2 == 0;
    case P::kSlowBlink: return (now_us / kLedSlowUs) % 2 == 0;
  }
  return false;
}

void dump_joy(bool connected, const int16_t axes[bt_teleop::kNumAxes], uint32_t buttons) {
    printf("[joy] %s LX=%6d LY=%6d L2=%6d RX=%6d RY=%6d R2=%6d DX=%6d DY=%6d btns=0x%02lx\n",
           connected ? "conn" : "idle",
           axes[bt_teleop::kLeftStickX], axes[bt_teleop::kLeftStickY], axes[bt_teleop::kL2],
           axes[bt_teleop::kRightStickX], axes[bt_teleop::kRightStickY], axes[bt_teleop::kR2],
           axes[bt_teleop::kDpadX], axes[bt_teleop::kDpadY],
           static_cast<unsigned long>(buttons));
}

}  // namespace

int main() {
    stdio_init_all();

    // bt_teleop owns cyw43_arch init (same chip the onboard LED hangs off of).
    if (!bt_teleop::init()) {
        while (true) {
            printf("[boot] bt_teleop init failed\n");
            sleep_ms(1000);
        }
    }

    // Servo 2040 Chica link (part 03): bring up the UART. The rail stays DROPPED
    // at boot — the pipeline's supervisor energizes it only after the link is up
    // and the engine has stood (relay-arming discipline), and drops it on a clean
    // fold or a critical battery.
    servo_out::init();
    servo_out::set_relay(false);
    bool relay_state = false;

    // The whole control brain (parts 05-09): teleop mapping, velocity shaping,
    // gait engine, posture stack, and the failsafe supervisor, built from the
    // baked config. It starts FOLDED and holds the folded pose until the init
    // button is pressed.
    hexa::pipeline::Pipeline pipeline;

    hexa::supervisor::LedPattern led_pattern = hexa::supervisor::LedPattern::kSlowBlink;

    printf("\n=== hexa Pico 2 W firmware ===\n");
    printf("board: pico2_w (RP2350)  build: %s %s\n", __DATE__, __TIME__);
    printf("stdio: USB-CDC   scheduler: %llu us tick (%.0f Hz)\n",
           (unsigned long long)kTickPeriodUs, 1e6 / (double)kTickPeriodUs);
    printf("bt: scanning for a gamepad (pair a PS4/PS5/Xbox pad)\n");
    printf("servo: Chica UART up (uart0 GP0/GP1 @ 921600); rail DROPPED "
           "(arms on link-up + stand)\n");
    printf("teleop: press init (start) to stand; sticks drive once standing\n");
    printf("safety: input timeout %.2f s, battery GET every %d ticks, heap %lu B "
           "free\n",
           (double)hexa::config::kInputTimeoutS, hexa::config::kGetPeriodTicks,
           (unsigned long)heap_free_bytes());
    printf("boot ok — entering loop\n");

    uint64_t next_tick_us = time_us_64();
    uint64_t next_beat_us = time_us_64();
    uint64_t next_joy_us = time_us_64();
    uint32_t heartbeat = 0;
    uint32_t tick_count = 0;  // control ticks since boot (paces the battery GET)

    // Latest part-03 servo-link measurements (logged with the 1 Hz heartbeat).
    uint64_t last_set_burst_us = 0;  // wall time of one command_all() SET burst
    uint64_t last_compute_us = 0;    // GET + Pipeline::tick() per control tick
    uint64_t last_get_rt_us = 0;     // round-trip of the battery GET
    bool last_batt_ok = false;
    float last_batt_v = 0.0f;
    float last_batt_i = 0.0f;
    int last_unreachable = 0;
    uint32_t min_free_heap = heap_free_bytes();  // soak low-water mark

    // Latest commanded joint angles (for the heartbeat dump). Seeded with the
    // standing pose so the pre-first-tick heartbeat has valid values.
    float theta[servo_out::kNumJoints];
    for (int i = 0; i < hexa::kNumLegs; ++i) {
        theta[i * 3 + 0] = hexa::config::kStandingPose[0];
        theta[i * 3 + 1] = hexa::config::kStandingPose[1];
        theta[i * 3 + 2] = hexa::config::kStandingPose[2];
    }

    while (true) {
        const uint64_t now_us = time_us_64();

        // Service hook (no-op in the background-serviced build).
        bt_teleop::pump();

        // 200 Hz control tick: sample the seam (gamepad, battery, clock), run the
        // shared pipeline, and apply its result over the same servo_out path
        // part 05 established.
        if (now_us >= next_tick_us) {
            next_tick_us += kTickPeriodUs;

            const uint64_t c0 = time_us_64();

            int16_t axes[bt_teleop::kNumAxes];
            uint32_t buttons;
            const bool bt_connected = bt_teleop::read(axes, buttons);

            // Battery telemetry: rate-limited Chica GET — one every
            // kGetPeriodTicks control ticks (mirrors hardware.yaml
            // get_period_ticks). Kept inside the tick so its round-trip counts
            // against the measured tick budget rather than hiding on a separate
            // timer.
            bool batt_fresh = false;
            if (tick_count % hexa::config::kGetPeriodTicks == 0) {
                const uint64_t g0 = time_us_64();
                last_batt_ok =
                    servo_out::read_battery(last_batt_v, last_batt_i, 20);
                last_get_rt_us = time_us_64() - g0;
                batt_fresh = last_batt_ok;
            }
            ++tick_count;

            hexa::pipeline::TickInput in;
            in.now_us = now_us;
            in.axes = axes;
            in.buttons = buttons;
            in.bt_connected = bt_connected;
            in.last_input_us = bt_teleop::last_data_us();
            in.battery_valid = batt_fresh;
            in.battery_v = last_batt_v;
            in.dt = kDt;
            const hexa::pipeline::TickResult res = pipeline.tick(in);

            // ── Teleop event logs (the pipeline resolved them; we narrate) ──
            if (res.mode_changed) {
                printf("[teleop] mode=%s\n", mode_name(res.mode));
            }
            if (res.init_request) {
                using IA = hexa::pipeline::InitAction;
                if (res.init_action == IA::kInitialized) {
                    printf("[teleop] init: FOLDED -> INITIALIZE\n");
                } else if (res.init_action == IA::kFoldRequested) {
                    printf("[teleop] init: fold requested (state=%s)\n",
                           hexa::gait::state_value(pipeline.engine().state()).c_str());
                }
            }
            if (res.has_animation_name) {
                if (res.animation_accepted) {
                    printf("[teleop] animation=%s\n", res.animation_name.c_str());
                } else {
                    printf("[teleop] animation=%s dropped (unknown)\n",
                           res.animation_name.c_str());
                }
            }
            if (res.has_gait_select) {
                if (res.gait_accepted) {
                    printf("[teleop] gait -> %s (linear_max=%.3f m/s)\n",
                           res.gait_select.c_str(), (double)res.gait_linear_max);
                } else {
                    printf("[teleop] gait -> %s dropped (state=%s)\n",
                           res.gait_select.c_str(),
                           hexa::gait::state_value(pipeline.engine().state()).c_str());
                }
            }

            // Relay discipline: drive the rail to the armed state, logging edges.
            if (res.relay_energized != relay_state) {
                servo_out::set_relay(res.relay_energized);
                relay_state = res.relay_energized;
                printf("[safety] relay %s (state=%s%s)\n",
                       relay_state ? "ENERGIZED" : "dropped",
                       hexa::gait::state_value(pipeline.engine().state()).c_str(),
                       res.decision.battery_critical ? ", battery critical" : "");
            }
            led_pattern = res.decision.led;

            last_unreachable = res.unreachable;
            std::copy(std::begin(res.theta), std::end(res.theta), std::begin(theta));
            last_compute_us = time_us_64() - c0;

            const uint64_t t0 = time_us_64();
            servo_out::command_all(res.theta);
            last_set_burst_us = time_us_64() - t0;

            const uint32_t fh = heap_free_bytes();  // soak low-water mark
            if (fh < min_free_heap) min_free_heap = fh;
        }

        // Status LED: driven every loop iteration off the latest supervisor
        // pattern (slow = idle/stand, solid = walking, fast = fault). Continuous,
        // not tied to the heartbeat, so the blink rate itself carries state.
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_level(led_pattern, now_us));

        // Teleop dump: the part-02 verification surface.
        if (now_us >= next_joy_us) {
            next_joy_us += kJoyDumpUs;
            int16_t axes[bt_teleop::kNumAxes];
            uint32_t buttons;
            const bool connected = bt_teleop::read(axes, buttons);
            dump_joy(connected, axes, buttons);
        }

        // 1 Hz heartbeat: log gait + timing + safety/health state. (The LED is
        // driven continuously above; the heartbeat is log-only now.)
        if (now_us >= next_beat_us) {
            next_beat_us += kHeartbeatUs;
            printf("[heartbeat] %lu  up=%llu ms\n",
                   (unsigned long)heartbeat++,
                   (unsigned long long)(now_us / 1000));
            if (last_batt_ok) {
                printf("[servo] compute=%llu us  SET burst=%llu us  GET rt=%llu us  "
                       "batt=%.2f V %.2f A\n",
                       (unsigned long long)last_compute_us,
                       (unsigned long long)last_set_burst_us,
                       (unsigned long long)last_get_rt_us,
                       (double)last_batt_v, (double)last_batt_i);
            } else {
                printf("[servo] compute=%llu us  SET burst=%llu us  GET rt=%llu us  "
                       "batt=-- (no reply)\n",
                       (unsigned long long)last_compute_us,
                       (unsigned long long)last_set_burst_us,
                       (unsigned long long)last_get_rt_us);
            }
            // Gait state: current FSM state, active strategy, master phase, and the
            // l_front leg's stance flag + commanded coxa/femur/tibia.
            const int l_front = hexa::leg_index(hexa::Leg::L_FRONT) * 3;
            printf("[gait] state=%s strat=%s phase=%.3f  l_front(c=%.3f f=%.3f "
                   "t=%.3f) unreachable=%d/6\n",
                   hexa::gait::state_value(pipeline.engine().state()).c_str(),
                   pipeline.engine().strategy_name().c_str(),
                   (double)pipeline.engine().master_phase(),
                   (double)theta[l_front], (double)theta[l_front + 1],
                   (double)theta[l_front + 2], last_unreachable);
            // Safety / health: relay + BT + battery flags, tick jitter (measured
            // interval spread vs the 5 ms budget + deadline overruns), and the
            // soak free-heap low-water mark (fragmentation drift).
            const auto& ts = pipeline.supervisor().tick_stats();
            printf("[safety] relay=%s bt=%s batt=%s jitter(last=%llu min=%llu "
                   "max=%llu us over=%lu/%lu) heap=%lu B (min %lu B)\n",
                   relay_state ? "on" : "off",
                   bt_teleop::connected() ? "conn" : "idle",
                   last_batt_ok ? (hexa::config::kBattery.warning_v > 0.0f
                                       ? "ok" : "ok(thr off)")
                                : "n/a",
                   (unsigned long long)ts.last_dt_us,
                   (unsigned long long)ts.min_dt_us,
                   (unsigned long long)ts.max_dt_us,
                   (unsigned long)ts.overruns, (unsigned long)ts.count,
                   (unsigned long)heap_free_bytes(),
                   (unsigned long)min_free_heap);
        }

        // Yield briefly so we don't spin the core at 100%.
        tight_loop_contents();
    }
}
