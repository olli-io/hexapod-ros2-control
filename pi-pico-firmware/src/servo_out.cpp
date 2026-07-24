// Servo 2040 slave link — Chica protocol over the Pico hardware UART (part 03).
//
// Forked from hexa_hardware/src/{servo2040_protocol,joint_calibration}.cpp. The
// protocol (encode_set / encode_get / decode_get_payload), the run-grouping in
// command_all(), and the pulse-width calibration (to_pulse_us) are the ROS2
// sources reused verbatim, with double→float per the RP2350 single-precision
// FPU. The termios `UartTransport` is replaced by the Pico SDK UART below
// (uart_init / uart_write_blocking / uart_is_readable / uart_getc); the USB and
// I2C transports and the hardware factory are dropped.
//
// Wiring and calibration are NOT hand-written here: they come from
// config_generated.hpp (`kJointCals`), baked by tools/gen_config.py from
// hexa_description/config's hardware.yaml + servo_calibration.yaml — the same
// YAMLs hexa_hardware loads at runtime, so firmware and ROS drive identical
// pins with identical endpoints.
//
// Baud/wiring assumption: the Servo 2040 speaks Chica over its hardware UART at
// 921600 8N1. VERIFY against the flashed Servo 2040 firmware and adjust kBaud /
// the GPIO below to match if pairing fails (plan part 03, item 4).

#include "servo_out.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <span>
#include <vector>

#include "config_generated.hpp"
#include "leg_index.hpp"

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "pico/time.h"  // time_us_64

namespace servo_out {
namespace {

// ── UART transport config ───────────────────────────────────────────────────
// uart0 on GP0 (TX) / GP1 (RX). CMakeLists reserves this pair for the Chica
// link (USB-CDC carries stdio, so UART stdio is off — no contention on uart0).
uart_inst_t* const kUart = uart0;
constexpr uint kTxPin = 0;
constexpr uint kRxPin = 1;
// 921600 8N1 → ~11.1 µs/byte. A 18-servo SET frame is 3 + 36 = 39 bytes
// (~0.43 ms); a battery GET round-trip adds the Servo 2040's reply latency.
// Both sit far inside the 5 ms tick budget (measured in main.cpp).
constexpr uint kBaud = 921600;

// ── Chica protocol constants (from servo2040_protocol.hpp) ───────────────────
constexpr std::uint8_t kCmdSet = 'S' | 0x80;
constexpr std::uint8_t kCmdGet = 'G' | 0x80;
constexpr std::uint16_t kValueMax = 0x3FFF;  // 14-bit

// ── Chica command-index map (protocol.md, hexapod-servo2040-driver) ──────────
// Fixed board/protocol indices in the Servo 2040 cmdPins space. CURR (24) and
// VOLT (25) are consecutive, so one GET(24,2) returns both (current then
// voltage). RELAY (26) is a digital SET. STATUS (27) is a read-only latched
// over-current fault register (no pin). These MUST match protocol.md and the
// ROS reference src/hexa_hardware/include/hexa_hardware/servo2040_protocol.hpp.
constexpr std::uint8_t kCurrIndex = 24;
constexpr std::uint8_t kVoltIndex = 25;
constexpr std::uint8_t kRelayPin = 26;
constexpr std::uint8_t kStatusIndex = 27;
constexpr std::uint16_t kStatusTrippedBit = 0x1;  // STATUS bit0 = over-current latch
constexpr float kTripAmpsPerCount = 0.1f;         // STATUS bits1-10: 0.1 A/count
// Battery telemetry wire units: fixed-point centi-units (count = value*100), so
// one count is 0.01 A / 0.01 V. A protocol constant, not per-board calibration.
constexpr float kAmpsPerCount = 0.01f;
constexpr float kVoltsPerCount = 0.01f;

// Per-joint wiring + calibration, baked from hexa_description/config's
// hardware.yaml (pin, `reversed`, `deg_at_center`, pulse clamps) and
// servo_calibration.yaml (endpoint pulse widths) by tools/gen_config.py. Those
// YAMLs stay the single source of truth — nothing here is hand-maintained.
//
// Row order is the pipeline's theta[] order (l_front, l_middle, l_rear, r_front,
// r_middle, r_rear, each {coxa, femur, tibia}), so kJoints[i] calibrates
// theta[i]. The harness order lives in each row's `pin`, NOT in the row order —
// the two differ on this build (l_rear is wired to pins 1-3), so anything that
// needs pin order sorts for it (pin_order / leg_pin_order below).
using hexa::config::JointCal;
constexpr const auto& kJoints = hexa::config::kJointCals;
static_assert(kJoints.size() == static_cast<std::size_t>(kNumJoints),
              "config_generated kJointCals must cover all 18 joints");

// Half-π as a float literal — avoids the double M_PI/2 (-Wdouble-promotion).
constexpr float kHalfPi = 1.57079632679489661923f;

// The table is leg-major: three consecutive rows are one leg's
// {coxa, femur, tibia}, so leg L owns indices 3L..3L+2.
constexpr std::size_t kNumJointsSz = kJoints.size();
constexpr std::size_t kNumLegsSz = static_cast<std::size_t>(hexa::kNumLegs);
constexpr std::size_t kJointsPerLeg = kNumJointsSz / kNumLegsSz;

// Joint indices sorted ascending by pin — the walk order for SET run-grouping,
// which needs consecutive pins to collapse into one frame. Built once; the
// table is static.
const std::array<std::size_t, kNumJointsSz>& pin_order() {
  static const std::array<std::size_t, kNumJointsSz> kOrder = [] {
    std::array<std::size_t, kNumJointsSz> order{};
    for (std::size_t i = 0; i < kNumJointsSz; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [](std::size_t a, std::size_t b) {
      return kJoints[a].pin < kJoints[b].pin;
    });
    return order;
  }();
  return kOrder;
}

// Leg indices ordered by each leg's lowest pin — the order the harness is wired
// in (rear → front with the shipped wiring), which is the order the energize
// sweep brings legs up in.
const std::array<std::size_t, kNumLegsSz>& leg_pin_order() {
  static const std::array<std::size_t, kNumLegsSz> kOrder = [] {
    auto lowest_pin = [](std::size_t leg) {
      std::uint8_t p = kJoints[leg * kJointsPerLeg].pin;
      for (std::size_t k = 1; k < kJointsPerLeg; ++k) {
        p = std::min(p, kJoints[leg * kJointsPerLeg + k].pin);
      }
      return p;
    };
    std::array<std::size_t, kNumLegsSz> order{};
    for (std::size_t i = 0; i < kNumLegsSz; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
      return lowest_pin(a) < lowest_pin(b);
    });
    return order;
  }();
  return kOrder;
}

// Reusable encode buffers (single-core cooperative loop → no synchronisation).
std::vector<std::uint8_t> g_set_buf;
std::vector<std::uint8_t> g_get_buf;

// ── Protocol (verbatim from servo2040_protocol.cpp, double→n/a) ──────────────
void encode_set(std::uint8_t start, std::span<const std::uint16_t> values,
                std::vector<std::uint8_t>& out) {
  out.clear();
  out.reserve(3 + values.size() * 2);
  out.push_back(kCmdSet);
  out.push_back(start & 0x7F);
  out.push_back(static_cast<std::uint8_t>(values.size()) & 0x7F);
  for (std::uint16_t v : values) {
    std::uint16_t c = v > kValueMax ? kValueMax : v;
    out.push_back(static_cast<std::uint8_t>(c & 0x7F));
    out.push_back(static_cast<std::uint8_t>((c >> 7) & 0x7F));
  }
}

void encode_get(std::uint8_t start, std::uint8_t count,
                std::vector<std::uint8_t>& out) {
  out.clear();
  out.reserve(3);
  out.push_back(kCmdGet);
  out.push_back(start & 0x7F);
  out.push_back(count & 0x7F);
}

bool decode_get_payload(std::span<const std::uint8_t> payload,
                        std::uint8_t& start,
                        std::vector<std::uint16_t>& values) {
  if (payload.size() < 2) {
    return false;
  }
  start = payload[0] & 0x7F;
  const std::uint8_t count = payload[1] & 0x7F;
  if (payload.size() < static_cast<std::size_t>(2 + count * 2)) {
    return false;
  }
  values.clear();
  values.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const std::uint8_t lo = payload[2 + i * 2] & 0x7F;
    const std::uint8_t hi = payload[3 + i * 2] & 0x7F;
    values.push_back(static_cast<std::uint16_t>(lo) |
                     (static_cast<std::uint16_t>(hi) << 7));
  }
  return true;
}

// ── Calibration (from joint_calibration.cpp, double→float) ───────────────────
std::uint16_t to_pulse_us(const JointCal& j, float theta_rad) {
  const float center_us = (j.us_at_plus_45 + j.us_at_minus_45) * 0.5f;
  // Endpoints are magnitudes; `direction` (+1 / -1) is the sole sign source.
  const float slope_us_per_rad = std::fabs(j.us_at_plus_45 - j.us_at_minus_45) / kHalfPi;
  const float us = center_us +
                   j.direction * (theta_rad - j.urdf_rad_at_center) * slope_us_per_rad;
  const float clamped = std::clamp(us, static_cast<float>(j.min_us),
                                   static_cast<float>(j.max_us));
  return static_cast<std::uint16_t>(std::lroundf(clamped));
}

// ── Pico UART read helper (replaces termios poll/read) ───────────────────────
// Wait up to `deadline_us` (absolute time_us_64) for one byte. Returns false on
// timeout.
bool read_byte(std::uint8_t& b, std::uint64_t deadline_us) {
  while (!uart_is_readable(kUart)) {
    if (time_us_64() >= deadline_us) return false;
    tight_loop_contents();
  }
  b = static_cast<std::uint8_t>(uart_getc(kUart));
  return true;
}

// GET one aux run: send the request, resync to the reply, decode. Mirrors
// Servo2040Protocol::read_aux with the Pico UART in place of the transport.
bool read_aux(std::uint8_t start_pin, std::uint8_t count,
              std::vector<std::uint16_t>& out, int timeout_ms) {
  encode_get(start_pin, count, g_get_buf);
  uart_write_blocking(kUart, g_get_buf.data(), g_get_buf.size());

  const std::uint64_t deadline = time_us_64() +
                                 static_cast<std::uint64_t>(timeout_ms) * 1000u;

  // Resync: drop bytes until we see a command byte (MSB set). Discard any
  // command that isn't G (e.g. a stray S echo).
  std::uint8_t b = 0;
  while (true) {
    if (!read_byte(b, deadline)) return false;
    if ((b & 0x80) == 0) continue;
    if (b == kCmdGet) break;
  }
  // Read payload: [start][count][2*count value bytes].
  const std::size_t payload_len = 2 + static_cast<std::size_t>(count) * 2;
  std::vector<std::uint8_t> payload(payload_len);
  for (std::size_t i = 0; i < payload_len; ++i) {
    if (!read_byte(payload[i], deadline)) return false;
  }
  std::uint8_t reply_start = 0;
  if (!decode_get_payload(payload, reply_start, out)) return false;
  return reply_start == start_pin && out.size() == count;
}

}  // namespace

// ── Public API ───────────────────────────────────────────────────────────────
void init() {
  uart_init(kUart, kBaud);
  gpio_set_function(kTxPin, GPIO_FUNC_UART);
  gpio_set_function(kRxPin, GPIO_FUNC_UART);
  uart_set_format(kUart, 8, 1, UART_PARITY_NONE);
  uart_set_hw_flow(kUart, false, false);
  uart_set_fifo_enabled(kUart, true);
}

void send_set(std::uint8_t start_pin, const std::uint16_t* pulses_us,
              std::size_t count) {
  encode_set(start_pin, std::span<const std::uint16_t>(pulses_us, count),
             g_set_buf);
  uart_write_blocking(kUart, g_set_buf.data(), g_set_buf.size());
}

void command_all(const float theta_rad[kNumJoints]) {
  std::uint16_t pulses[kNumJointsSz];
  for (std::size_t i = 0; i < kNumJointsSz; ++i) {
    pulses[i] = to_pulse_us(kJoints[i], theta_rad[i]);
  }
  // Walk the joints in PIN order, accumulating runs of consecutive pins, and
  // emit one SET frame per run (pins 1..18 are all wired, so the current harness
  // collapses to a single 18-value frame).
  const auto& order = pin_order();
  std::size_t i = 0;
  while (i < kNumJointsSz) {
    const std::uint8_t run_start = kJoints[order[i]].pin;
    std::uint16_t run[kNumJointsSz];
    std::size_t n = 0;
    std::size_t k = i;
    while (k < kNumJointsSz && kJoints[order[k]].pin == run_start + (k - i)) {
      run[n++] = pulses[order[k]];
      ++k;
    }
    send_set(run_start, run, n);
    i = k;
  }
}

void command_legs(const float theta_rad[kNumJoints], int n_legs) {
  if (n_legs <= 0) return;
  // Once every leg is live, fall back to the whole-table walk — that collapses
  // to a single 18-servo SET frame under the current wiring, so the steady state
  // costs exactly what it did before the sweep existed.
  if (n_legs >= hexa::kNumLegs) {
    command_all(theta_rad);
    return;
  }
  const auto& order = leg_pin_order();
  for (std::size_t l = 0; l < static_cast<std::size_t>(n_legs); ++l) {
    const std::size_t base = order[l] * kJointsPerLeg;
    std::uint16_t pulses[kJointsPerLeg];
    for (std::size_t k = 0; k < kJointsPerLeg; ++k) {
      pulses[k] = to_pulse_us(kJoints[base + k], theta_rad[base + k]);
    }
    // Same consecutive-pin run grouping as command_all, scoped to this leg (one
    // frame per leg while its three joints sit on consecutive pins).
    std::size_t i = 0;
    while (i < kJointsPerLeg) {
      const std::uint8_t run_start = kJoints[base + i].pin;
      std::uint16_t run[kJointsPerLeg];
      std::size_t n = 0;
      std::size_t k = i;
      while (k < kJointsPerLeg && kJoints[base + k].pin == run_start + (k - i)) {
        run[n++] = pulses[k];
        ++k;
      }
      send_set(run_start, run, n);
      i = k;
    }
  }
}

float joint_center_rad(int joint) {
  if (joint < 0 || joint >= kNumJoints) return 0.0f;
  return kJoints[static_cast<std::size_t>(joint)].urdf_rad_at_center;
}

void set_relay(bool energized) {
  const std::uint16_t v = energized ? 1u : 0u;
  send_set(kRelayPin, &v, 1);
}

bool read_battery(float& voltage_v, float& current_a, int timeout_ms) {
  // CURR (24) and VOLT (25) are consecutive: one GET fetches both, with
  // values[0] = current, values[1] = voltage (protocol.md ordering). Both come
  // in a single reply, so it is both-or-neither — no partial/NaN case.
  std::vector<std::uint16_t> raw;
  if (!read_aux(kCurrIndex, 2, raw, timeout_ms) || raw.size() != 2) {
    current_a = std::numeric_limits<float>::quiet_NaN();
    return false;
  }
  current_a = static_cast<float>(raw[0]) * kAmpsPerCount;
  voltage_v = static_cast<float>(raw[1]) * kVoltsPerCount;
  return true;
}

bool read_status(bool& tripped, float& trip_amps, int timeout_ms) {
  // STATUS (27) is a single read-only 14-bit word: bit0 = TRIPPED (over-current
  // latch active), bits1-10 = trip current at 0.1 A/count. 0 = clean.
  std::vector<std::uint16_t> raw;
  if (!read_aux(kStatusIndex, 1, raw, timeout_ms) || raw.size() != 1) {
    return false;
  }
  const std::uint16_t word = raw[0];
  tripped = (word & kStatusTrippedBit) != 0;
  trip_amps = static_cast<float>((word >> 1) & 0x3FF) * kTripAmpsPerCount;
  return true;
}

}  // namespace servo_out
