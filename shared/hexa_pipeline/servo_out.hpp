// Servo 2040 slave link (plan part 03).
//
// Drives a Pimoroni Servo 2040 as a dumb PWM slave over the Pico's hardware
// UART using the existing "Chica" binary protocol. Forked from
// hexa_hardware/src/{servo2040_protocol,joint_calibration}.cpp — the framing,
// run-grouping and pulse-width calibration are reused verbatim (double→float);
// only the transport changes: the host termios `UartTransport` is replaced by
// the Pico SDK UART (see servo_out.cpp). No PWM code runs on the Pico.
//
// The link carries three things, all as Chica frames on one UART:
//   - SET  — servo pulse widths (µs), grouped into runs of consecutive pins,
//   - SET  — the servo-rail relay (a digital Servo 2040 pin),
//   - GET  — battery voltage/current (Servo 2040 aux ADC pins).
//
// Pin assignment and calibration are hardcoded here for part 03; part 04 folds
// them into the build-time `config_generated.hpp` (sourced from hexa_description
// / hexa_hardware YAMLs). Leg count is fixed at 6 → 18 joints.
#pragma once

#include <cstddef>
#include <cstdint>

namespace servo_out {

// 6 legs × 3 joints. Joint order is the calibration-table order (pins 1..18):
// l_front {coxa,femur,tibia}, l_middle, l_rear, r_front, r_middle, r_rear.
constexpr int kNumJoints = 18;

// Configure the hardware UART (GP0 TX / GP1 RX, uart0) and the Chica link.
// Call once at boot before any send/read. Does NOT energize the relay — that
// is gated on link-up + stand (part 09); call set_relay(true) explicitly.
void init();

// Low-level Chica SET: `count` consecutive pulse widths (µs) starting at
// `start_pin`. Fire-and-forget (no reply). Values are clamped to 14 bits by
// the framing; supply already-calibrated pulse widths.
void send_set(std::uint8_t start_pin, const std::uint16_t* pulses_us,
              std::size_t count);

// Command all 18 joints (URDF-convention radians, joint/pin order above).
// Calibrates each angle to a pulse width and emits one SET frame per run of
// consecutive pins (the current wiring, pins 1..18, is a single run).
void command_all(const float theta_rad[kNumJoints]);

// URDF radian a joint sits at when its servo is centered (pulse = midpoint of
// the two calibration endpoints). Useful for demos/tests that sweep
// symmetrically about center; command_all(center) parks every servo at 1500 µs
// nominal.
float joint_center_rad(int joint);

// Servo-rail relay via a Chica digital SET. true = energize the rail.
void set_relay(bool energized);

// Read battery telemetry via Chica GET on the aux ADC pins. Fills `voltage_v`
// and `current_a` (engineering units, raw 14-bit count × per-channel scale).
// Returns false on timeout / malformed reply; `current_a` is set to NaN if the
// voltage read succeeds but the current read does not. `timeout_ms` bounds each
// GET reply (pass e.g. 20).
bool read_battery(float& voltage_v, float& current_a, int timeout_ms);

}  // namespace servo_out
