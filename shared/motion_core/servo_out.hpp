// Drives a Pimoroni Servo 2040 as a dumb PWM slave over the Pico's hardware UART
// using the "Chica" binary protocol; no PWM code runs on the Pico. The link
// carries four things, all Chica frames on one UART:
//   - SET  — servo pulse widths (µs), grouped into runs of consecutive pins,
//   - SET  — the servo-rail relay (digital pin 26),
//   - GET  — battery current/voltage (24/25),
//   - GET  — the latched over-current fault register (STATUS, 27).
// Indices and scales follow the driver's protocol.md. Pin assignment and
// calibration come from config_generated.hpp's kJointCals.
#pragma once

#include <cstddef>
#include <cstdint>

namespace servo_out {

// Joint order is the pipeline's theta[] order, matching kJointCals row for row.
// The servo PIN each joint drives is a separate thing (hardware.yaml), carried
// per row and sorted where pin order matters.
constexpr int kNumJoints = 18;

// Configure the hardware UART (GP0 TX / GP1 RX, uart0) and the Chica link, once
// at boot. Does NOT energize the relay; call set_relay(true) explicitly.
void init();

// `count` consecutive pulse widths (µs) from `start_pin`, fire-and-forget.
// Already-calibrated values only; the framing clamps them to 14 bits.
void send_set(std::uint8_t start_pin, const std::uint16_t* pulses_us,
              std::size_t count);

// All 18 joints in URDF-convention radians: one SET frame per run of consecutive
// pins (the current wiring, pins 1..18, is a single run).
void command_all(const float theta_rad[kNumJoints]);

// Only the first `n_legs` legs in PIN order, leaving the rest undriven. This is
// the inrush stagger at the relay OFF->ON edge: a servo stays limp until its
// first SET, so pacing the SETs paces the current draw (hexa::EnergizeSweep
// decides `n_legs`). n_legs >= 6 is exactly command_all().
void command_legs(const float theta_rad[kNumJoints], int n_legs);

// URDF radian a joint sits at with its servo centered (the midpoint of the two
// calibration endpoints), for demos/tests that sweep symmetrically about it.
float joint_center_rad(int joint);

// Servo-rail relay via a Chica digital SET. true = energize the rail.
void set_relay(bool energized);

// One aux poll, split across two control ticks so the tick never waits on the
// board: request_aux() sends GET(24,2) over CURR + VOLT and GET(27,1) over the
// latched over-current STATUS; collect_aux() on a later tick decodes whatever
// replies have arrived, without blocking.
struct AuxReading {
  bool battery_ok = false;  // CURR + VOLT reply decoded (both-or-neither)
  float voltage_v = 0.0f;
  float current_a = 0.0f;
  bool status_ok = false;   // STATUS reply decoded
  bool tripped = false;     // STATUS bit0; sticky until set_relay(false)
  float trip_amps = 0.0f;   // current captured at the trip
};

// Empties the RX FIFO first, so a late reply from an earlier poll is never read
// as this one's.
void request_aux();
AuxReading collect_aux();

}  // namespace servo_out
