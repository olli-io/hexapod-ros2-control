// Unit tests for the pin-ordered leg grouping the energize sweep staggers at.

#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "hexa_hardware/leg_order.hpp"

namespace hh = hexa_hardware;

namespace {

// Build the (joint_names, pin_order) pair the plugin hands to build_leg_order:
// `wiring` is {joint name, 1-based silkscreen pin} in URDF/controller order,
// pin_order is the same set sorted ascending by 0-based board index.
struct Wiring {
  std::vector<std::string> names;
  std::vector<hh::PinEntry> pin_order;
};

Wiring make(const std::vector<std::pair<std::string, int>>& wiring) {
  Wiring w;
  for (std::size_t i = 0; i < wiring.size(); ++i) {
    w.names.push_back(wiring[i].first);
    w.pin_order.push_back(
        {static_cast<std::uint8_t>(wiring[i].second - 1), i});
  }
  std::sort(w.pin_order.begin(), w.pin_order.end(),
            [](const hh::PinEntry& a, const hh::PinEntry& b) {
              return a.pin < b.pin;
            });
  return w;
}

std::vector<std::string> leg_names(const std::vector<hh::LegGroup>& legs) {
  std::vector<std::string> out;
  for (const auto& l : legs) out.push_back(l.name);
  return out;
}

// The shipped hardware.yaml wiring, listed in the URDF/controller joint order
// (l_front, l_middle, l_rear, r_front, r_middle, r_rear) so the test also
// covers the reorder between that and the harness.
const std::vector<std::pair<std::string, int>> kShippedWiring = {
    {"l_front_coxa_joint", 13},  {"l_front_femur_joint", 14},
    {"l_front_tibia_joint", 15}, {"l_middle_coxa_joint", 7},
    {"l_middle_femur_joint", 8}, {"l_middle_tibia_joint", 9},
    {"l_rear_coxa_joint", 1},    {"l_rear_femur_joint", 2},
    {"l_rear_tibia_joint", 3},   {"r_front_coxa_joint", 16},
    {"r_front_femur_joint", 17}, {"r_front_tibia_joint", 18},
    {"r_middle_coxa_joint", 10}, {"r_middle_femur_joint", 11},
    {"r_middle_tibia_joint", 12}, {"r_rear_coxa_joint", 4},
    {"r_rear_femur_joint", 5},   {"r_rear_tibia_joint", 6},
};

}  // namespace

TEST(BuildLegOrder, ShippedWiringSweepsRearToFront) {
  const auto w = make(kShippedWiring);
  const auto legs = hh::build_leg_order(w.names, w.pin_order);
  EXPECT_EQ(leg_names(legs),
            (std::vector<std::string>{"l_rear", "r_rear", "l_middle",
                                      "r_middle", "l_front", "r_front"}));
}

TEST(BuildLegOrder, EachLegHoldsItsThreeJoints) {
  const auto w = make(kShippedWiring);
  const auto legs = hh::build_leg_order(w.names, w.pin_order);
  ASSERT_EQ(legs.size(), 6u);
  for (const auto& leg : legs) {
    EXPECT_EQ(leg.pin_order_idx.size(), 3u) << leg.name;
    // Indices ascend, so the caller's consecutive-run walk sees rising pins.
    EXPECT_TRUE(std::is_sorted(leg.pin_order_idx.begin(),
                               leg.pin_order_idx.end()))
        << leg.name;
  }
}

TEST(BuildLegOrder, IndicesResolveBackToTheOwningLeg) {
  const auto w = make(kShippedWiring);
  const auto legs = hh::build_leg_order(w.names, w.pin_order);
  for (const auto& leg : legs) {
    for (const std::size_t idx : leg.pin_order_idx) {
      const std::string& name = w.names[w.pin_order[idx].joint_idx];
      EXPECT_EQ(name.rfind(leg.name + "_", 0), 0u)
          << name << " grouped under " << leg.name;
    }
  }
}

TEST(BuildLegOrder, HandlesInterleavedWiring) {
  // A leg whose pins are not contiguous still groups correctly; ordering is by
  // the leg's LOWEST pin.
  const auto w = make({
      {"l_rear_coxa_joint", 1},
      {"r_rear_coxa_joint", 2},
      {"l_rear_femur_joint", 3},
      {"r_rear_femur_joint", 4},
  });
  const auto legs = hh::build_leg_order(w.names, w.pin_order);
  EXPECT_EQ(leg_names(legs), (std::vector<std::string>{"l_rear", "r_rear"}));
  EXPECT_EQ(legs[0].pin_order_idx, (std::vector<std::size_t>{0, 2}));
  EXPECT_EQ(legs[1].pin_order_idx, (std::vector<std::size_t>{1, 3}));
}

TEST(BuildLegOrder, RejectsUnrecognisedJointName) {
  const auto w = make({{"l_rear_knee_joint", 1}});
  EXPECT_THROW(hh::build_leg_order(w.names, w.pin_order), std::runtime_error);
}

TEST(BuildLegOrder, EmptyWiringYieldsNoLegs) {
  EXPECT_TRUE(hh::build_leg_order({}, {}).empty());
}

// ── build_pin_runs — how many SET frames a tick costs ───────────────────────
//
// The stagger is an inrush measure, not a wire format: once every leg is live
// the whole table must collapse back to the runs the harness allows. Splitting
// steady-state traffic into one frame per leg puts the last leg in pin order at
// the tail of a six-frame burst every tick, which is where r_front's commands
// were going missing.

namespace {

std::vector<std::size_t> run_lengths(const std::vector<hh::PinRun>& runs) {
  std::vector<std::size_t> out;
  for (const auto& r : runs) out.push_back(r.joint_idx.size());
  return out;
}

}  // namespace

TEST(BuildPinRuns, ShippedWiringCollapsesToOneFrame) {
  const auto w = make(kShippedWiring);
  const auto runs = hh::build_pin_runs(w.pin_order);
  ASSERT_EQ(runs.size(), 1u);
  EXPECT_EQ(runs[0].start_pin, 0u);
  EXPECT_EQ(runs[0].joint_idx.size(), 18u);
}

TEST(BuildPinRuns, WholeTableRunDrivesJointsInPinOrder) {
  const auto w = make(kShippedWiring);
  const auto runs = hh::build_pin_runs(w.pin_order);
  ASSERT_EQ(runs.size(), 1u);
  // Board index N carries the joint the wiring put on silkscreen pin N+1, so
  // the payload must read l_rear -> ... -> r_front, not the controller order.
  EXPECT_EQ(w.names[runs[0].joint_idx.front()], "l_rear_coxa_joint");
  EXPECT_EQ(w.names[runs[0].joint_idx.back()], "r_front_tibia_joint");
  for (std::size_t i = 0; i < runs[0].joint_idx.size(); ++i) {
    EXPECT_EQ(w.names[runs[0].joint_idx[i]],
              w.names[w.pin_order[i].joint_idx]);
  }
}

TEST(BuildPinRuns, PerLegSliceIsOneFramePerLeg) {
  // What the sweep ramp costs while some legs must stay limp.
  const auto w = make(kShippedWiring);
  const auto legs = hh::build_leg_order(w.names, w.pin_order);
  ASSERT_EQ(legs.size(), 6u);
  for (const auto& leg : legs) {
    const auto runs = hh::build_pin_runs(w.pin_order, leg.pin_order_idx);
    ASSERT_EQ(runs.size(), 1u) << leg.name;
    EXPECT_EQ(runs[0].joint_idx.size(), 3u) << leg.name;
  }
}

TEST(BuildPinRuns, SplitsOnAPinGap) {
  // Board indices 0,1,2 then 4,5 — the gap forces a second frame.
  const auto w = make({
      {"l_rear_coxa_joint", 1},
      {"l_rear_femur_joint", 2},
      {"l_rear_tibia_joint", 3},
      {"r_rear_coxa_joint", 5},
      {"r_rear_femur_joint", 6},
  });
  const auto runs = hh::build_pin_runs(w.pin_order);
  ASSERT_EQ(runs.size(), 2u);
  EXPECT_EQ(runs[0].start_pin, 0u);
  EXPECT_EQ(runs[1].start_pin, 4u);
  EXPECT_EQ(run_lengths(runs), (std::vector<std::size_t>{3u, 2u}));
}

TEST(BuildPinRuns, InterleavedWiringGivesALegOneFramePerJoint) {
  // The worst case the fallback cannot help: a leg wired onto non-consecutive
  // pins pays a frame per joint whether or not the sweep is done.
  const auto w = make({
      {"l_rear_coxa_joint", 1},
      {"r_rear_coxa_joint", 2},
      {"l_rear_femur_joint", 3},
      {"r_rear_femur_joint", 4},
  });
  const auto legs = hh::build_leg_order(w.names, w.pin_order);
  const auto runs = hh::build_pin_runs(w.pin_order, legs[0].pin_order_idx);
  EXPECT_EQ(run_lengths(runs), (std::vector<std::size_t>{1u, 1u}));
  // Whole-table, the same wiring is still one consecutive run.
  EXPECT_EQ(hh::build_pin_runs(w.pin_order).size(), 1u);
}

TEST(BuildPinRuns, EmptyViewYieldsNoFrames) {
  EXPECT_TRUE(hh::build_pin_runs({}).empty());
  const auto w = make(kShippedWiring);
  EXPECT_TRUE(hh::build_pin_runs(w.pin_order, {}).empty());
}

// ── is_flat_pin_map — whether the compact SETALL frame can express the pose ──
//
// SETALL carries no start/count header: it is exactly "all 18 servos, board
// index 0..17". Anything else has to fall back to SET frames, so this predicate
// must reject a near-miss rather than let one through.

TEST(IsFlatPinMap, AcceptsShippedWiring) {
  const auto w = make(kShippedWiring);
  EXPECT_TRUE(hh::is_flat_pin_map(w.pin_order, 18));
}

TEST(IsFlatPinMap, RejectsAGap) {
  // Board indices 0,1,2,4,5 — 18 servos' worth of pins would not be 0..17.
  const auto w = make({
      {"l_rear_coxa_joint", 1},
      {"l_rear_femur_joint", 2},
      {"l_rear_tibia_joint", 3},
      {"r_rear_coxa_joint", 5},
      {"r_rear_femur_joint", 6},
  });
  EXPECT_FALSE(hh::is_flat_pin_map(w.pin_order, 5));
}

TEST(IsFlatPinMap, RejectsNonZeroStart) {
  const auto w = make({
      {"l_rear_coxa_joint", 2},
      {"l_rear_femur_joint", 3},
      {"l_rear_tibia_joint", 4},
  });
  EXPECT_FALSE(hh::is_flat_pin_map(w.pin_order, 3));
}

TEST(IsFlatPinMap, RejectsWrongCount) {
  const auto w = make(kShippedWiring);
  EXPECT_FALSE(hh::is_flat_pin_map(w.pin_order, 17));
  EXPECT_FALSE(hh::is_flat_pin_map(w.pin_order, 19));
  // A consecutive-but-short harness is still not the full-table layout.
  const auto shortw = make({
      {"l_rear_coxa_joint", 1},
      {"l_rear_femur_joint", 2},
      {"l_rear_tibia_joint", 3},
  });
  EXPECT_FALSE(hh::is_flat_pin_map(shortw.pin_order, 18));
  EXPECT_TRUE(hh::is_flat_pin_map(shortw.pin_order, 3));
}

TEST(IsFlatPinMap, EmptyViewIsNotAFullTable) {
  EXPECT_FALSE(hh::is_flat_pin_map({}, 18));
}
