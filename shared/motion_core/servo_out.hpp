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

// One Chica GET(24,2) over CURR + VOLT, in engineering units. Both arrive in one
// reply, so it is both-or-neither: false on timeout / malformed reply.
bool read_battery(float& voltage_v, float& current_a, int timeout_ms);

// The latched over-current fault register, via Chica GET(27,1): `tripped` is
// STATUS bit0 and `trip_amps` the current captured at the trip. Sticky until the
// host clears it with set_relay(false). False on timeout / malformed reply.
bool read_status(bool& tripped, float& trip_amps, int timeout_ms);

}  // namespace servo_out
