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
// Baud/wiring assumption: the Servo 2040 speaks Chica over its hardware UART at
// 921600 8N1. VERIFY against the flashed Servo 2040 firmware and adjust kBaud /
// the GPIO below to match if pairing fails (plan part 03, item 4).

#include "servo_out.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <vector>

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

// ── Hardware map (hardcoded from hexa_hardware/config/hardware.yaml) ─────────
// Baked here for part 03; part 04 emits this from the YAMLs into
// config_generated.hpp. Servo pin 24 = rail relay; aux ADCs 26/27 = battery.
constexpr std::uint8_t kRelayPin = 24;
constexpr std::uint8_t kBatteryVoltagePin = 26;
constexpr std::uint8_t kBatteryCurrentPin = 27;
constexpr float kBatteryVoltageScale = 0.00366f;  // 14-bit count → volts
constexpr float kBatteryCurrentScale = 0.00098f;  // 14-bit count → amps

// Per-joint calibration. Endpoint magnitudes (1000/2000 µs at ∓π/4, 1500 µs
// center) and clamps (500/2500 µs) are the hardware.yaml defaults; `direction`
// (+1 normal, -1 mirror-mounted) is the sole travel-sign source;
// `urdf_rad_at_center` is the joint's URDF radian at servo center, precomputed
// from `deg_at_center` (coxa 0°, femur 35°, tibia 68°) via the per-position mapping:
//   coxa  urdf =  deg·π/180        → 0
//   femur urdf = -deg·π/180        → -0.61086524
//   tibia urdf =  π - deg·π/180    →  1.95476876
struct JointCal {
  std::uint8_t pin;
  float us_at_plus_45;
  float us_at_minus_45;
  float urdf_rad_at_center;
  std::int8_t direction;  // +1 normal, -1 mirror-mounted servo
  std::uint16_t min_us;
  std::uint16_t max_us;
};

constexpr float kCoxaCenter = 0.0f;
constexpr float kFemurCenter = -0.61086524f;
constexpr float kTibiaCenter = 1.95476876f;

// Table order (pins 1..18) IS the joint order: l_front, l_middle, l_rear,
// r_front, r_middle, r_rear, each {coxa, femur, tibia}. Sorted by pin so the
// run-grouping in command_all() sees consecutive pins.
constexpr JointCal kJoints[kNumJoints] = {
    {1, 2000.f, 1000.f, kCoxaCenter, 1, 500, 2500},   // l_front_coxa
    {2, 2000.f, 1000.f, kFemurCenter, -1, 500, 2500},  // l_front_femur
    {3, 2000.f, 1000.f, kTibiaCenter, -1, 500, 2500},  // l_front_tibia
    {4, 2000.f, 1000.f, kCoxaCenter, 1, 500, 2500},   // l_middle_coxa
    {5, 2000.f, 1000.f, kFemurCenter, -1, 500, 2500},  // l_middle_femur
    {6, 2000.f, 1000.f, kTibiaCenter, -1, 500, 2500},  // l_middle_tibia
    {7, 2000.f, 1000.f, kCoxaCenter, 1, 500, 2500},   // l_rear_coxa
    {8, 2000.f, 1000.f, kFemurCenter, -1, 500, 2500},  // l_rear_femur
    {9, 2000.f, 1000.f, kTibiaCenter, -1, 500, 2500},  // l_rear_tibia
    {10, 2000.f, 1000.f, kCoxaCenter, 1, 500, 2500},   // r_front_coxa
    {11, 2000.f, 1000.f, kFemurCenter, 1, 500, 2500},  // r_front_femur
    {12, 2000.f, 1000.f, kTibiaCenter, 1, 500, 2500},  // r_front_tibia
    {13, 2000.f, 1000.f, kCoxaCenter, 1, 500, 2500},   // r_middle_coxa
    {14, 2000.f, 1000.f, kFemurCenter, 1, 500, 2500},  // r_middle_femur
    {15, 2000.f, 1000.f, kTibiaCenter, 1, 500, 2500},  // r_middle_tibia
    {16, 2000.f, 1000.f, kCoxaCenter, 1, 500, 2500},   // r_rear_coxa
    {17, 2000.f, 1000.f, kFemurCenter, 1, 500, 2500},  // r_rear_femur
    {18, 2000.f, 1000.f, kTibiaCenter, 1, 500, 2500},  // r_rear_tibia
};

// Half-π as a float literal — avoids the double M_PI/2 (-Wdouble-promotion).
constexpr float kHalfPi = 1.57079632679489661923f;

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
  std::uint16_t pulses[kNumJoints];
  for (int i = 0; i < kNumJoints; ++i) {
    pulses[i] = to_pulse_us(kJoints[i], theta_rad[i]);
  }
  // Walk the pin-sorted table, accumulating runs of consecutive pins, and emit
  // one SET frame per run (pins 1..18 → a single run under the current wiring).
  std::size_t i = 0;
  while (i < kNumJoints) {
    const std::uint8_t run_start = kJoints[i].pin;
    std::uint16_t run[kNumJoints];
    std::size_t n = 0;
    std::size_t k = i;
    while (k < static_cast<std::size_t>(kNumJoints) &&
           kJoints[k].pin == run_start + (k - i)) {
      run[n++] = pulses[k];
      ++k;
    }
    send_set(run_start, run, n);
    i = k;
  }
}

float joint_center_rad(int joint) {
  if (joint < 0 || joint >= kNumJoints) return 0.0f;
  return kJoints[joint].urdf_rad_at_center;
}

void set_relay(bool energized) {
  const std::uint16_t v = energized ? 1u : 0u;
  send_set(kRelayPin, &v, 1);
}

bool read_battery(float& voltage_v, float& current_a, int timeout_ms) {
  std::vector<std::uint16_t> raw;
  if (!read_aux(kBatteryVoltagePin, 1, raw, timeout_ms) || raw.size() != 1) {
    return false;
  }
  voltage_v = static_cast<float>(raw[0]) * kBatteryVoltageScale;

  std::vector<std::uint16_t> raw_i;
  if (read_aux(kBatteryCurrentPin, 1, raw_i, timeout_ms) && raw_i.size() == 1) {
    current_a = static_cast<float>(raw_i[0]) * kBatteryCurrentScale;
  } else {
    current_a = std::numeric_limits<float>::quiet_NaN();
  }
  return true;
}

}  // namespace servo_out
