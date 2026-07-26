// Pin-ordered leg grouping for the staggered energize sweep.
//
// The board drives a servo only once the host has SET it, so the host decides
// which servos come up when (see the driver's protocol.md, "Energizing servos:
// relay-first, host-ordered"). Staggering that by *leg* needs the wiring grouped
// into legs and those legs ordered the way the harness is laid out — with the
// shipped hardware.yaml that is rear → front, alternating sides.
//
// Kept as a free function on plain data so it is unit-testable without standing
// up the ros2_control plugin.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hexa_hardware {

// One joint's place in the pin-sorted view: its 0-based board index and the
// index of its JointSlot.
struct PinEntry {
  std::uint8_t pin;
  std::size_t joint_idx;
};

// One leg's slice of the pin-sorted view.
struct LegGroup {
  std::string name;                     // "l_rear", "r_middle", …
  std::vector<std::size_t> pin_order_idx;  // indices into pin_order, ascending
};

// One SET frame's worth of that view: a run of consecutive board indices, given
// as the JointSlot indices it drives in pin order. The board addresses a SET by
// start index + count, so only consecutive pins collapse into one frame.
struct PinRun {
  std::uint8_t start_pin;              // board index of the run's first servo
  std::vector<std::size_t> joint_idx;  // JointSlot indices, ascending by pin
};

// Group `pin_order` (ascending by pin) into legs and order the legs by their
// lowest pin. `joint_names[e.joint_idx]` must be a URDF joint name of the form
// `<leg>_<segment>_joint`; anything else throws std::runtime_error.
std::vector<LegGroup> build_leg_order(const std::vector<std::string>& joint_names,
                                      const std::vector<PinEntry>& pin_order);

// Split `slice` (indices into `pin_order`, ascending) into runs of consecutive
// board indices — one SET frame per run. Precomputed rather than rebuilt per
// tick: which frames a given set of joints costs is fixed by the wiring.
std::vector<PinRun> build_pin_runs(const std::vector<PinEntry>& pin_order,
                                   const std::vector<std::size_t>& slice);

// build_pin_runs over the whole view — the steady-state frame plan, one frame
// under a fully consecutive harness.
std::vector<PinRun> build_pin_runs(const std::vector<PinEntry>& pin_order);

// True if `pin_order` (ascending) is exactly the flat board map 0..expected-1
// with no gaps, no offset and nothing missing. The board's all-servos fast path
// carries no start/count header, so it can only express that one layout.
bool is_flat_pin_map(const std::vector<PinEntry>& pin_order,
                     std::size_t expected);

}  // namespace hexa_hardware
