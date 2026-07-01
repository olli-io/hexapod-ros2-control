// gtest port of hexa_gait/test/test_clock.py — exercises the pure C++ GaitClock
// and PhaseOffsets (no ROS). Behavioural parity with the Python suite. Python's
// ValueError maps to std::invalid_argument, and pytest.approx to EXPECT_NEAR.
// TRIPOD_OFFSETS (a non-exported constant in registry.cpp) is mirrored locally,
// as test_engine.cpp does with tripod_offsets().

#include <gtest/gtest.h>

#include <map>
#include <string>

#include "hexa_gait_cpp/clock.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {
namespace {

// All six legs at offset 0.0. Port of _zero_offsets().
PhaseOffsets zero_offsets() {
  std::map<std::string, double> m;
  for (const auto& n : LEG_NAMES) {
    m[n] = 0.0;
  }
  return PhaseOffsets(m);
}

// Tripod phase offsets (the anonymous-namespace TRIPOD_OFFSETS in registry.cpp
// is not exported, so mirror its values here). Leading tripod at 0.0, trailing
// tripod at 0.5.
PhaseOffsets tripod_offsets() {
  return PhaseOffsets({
      {"l_front", 0.0},
      {"r_middle", 0.0},
      {"l_rear", 0.0},
      {"r_front", 0.5},
      {"l_middle", 0.5},
      {"r_rear", 0.5},
  });
}

TEST(Clock, AdvanceWrapsAfterOneCycle) {
  GaitClock clock(zero_offsets());
  clock.advance(0.5, 1.0);
  EXPECT_NEAR(clock.master(), 0.5, 1e-9);
  clock.advance(0.5, 1.0);
  // 1.0 wraps to 0.0.
  EXPECT_NEAR(clock.master(), 0.0, 1e-12);
}

TEST(Clock, ResetSeedsMaster) {
  GaitClock clock(zero_offsets());
  clock.reset(0.3);
  EXPECT_NEAR(clock.master(), 0.3, 1e-9);
}

TEST(Clock, ResetRejectsOutOfRange) {
  GaitClock clock(zero_offsets());
  EXPECT_THROW(clock.reset(1.0), std::invalid_argument);
  EXPECT_THROW(clock.reset(-0.1), std::invalid_argument);
}

TEST(Clock, PhasesApplyOffsetsModuloOne) {
  GaitClock clock(tripod_offsets());
  auto phases = clock.phases();
  // Tripod A (offset 0.0) starts at 0.0.
  EXPECT_NEAR(phases["l_front"], 0.0, 1e-9);
  EXPECT_NEAR(phases["r_middle"], 0.0, 1e-9);
  EXPECT_NEAR(phases["l_rear"], 0.0, 1e-9);
  // Tripod B (offset 0.5) starts at 0.5.
  EXPECT_NEAR(phases["r_front"], 0.5, 1e-9);
  EXPECT_NEAR(phases["l_middle"], 0.5, 1e-9);
  EXPECT_NEAR(phases["r_rear"], 0.5, 1e-9);

  clock.advance(0.4, 1.0);
  phases = clock.phases();
  EXPECT_NEAR(phases["l_front"], 0.4, 1e-9);
  // Tripod B wraps around.
  EXPECT_NEAR(phases["r_front"], 0.9, 1e-9);
}

TEST(Clock, AdvanceRejectsNonPositiveCycleTime) {
  GaitClock clock(zero_offsets());
  EXPECT_THROW(clock.advance(0.1, 0.0), std::invalid_argument);
  EXPECT_THROW(clock.advance(0.1, -1.0), std::invalid_argument);
}

TEST(Clock, PhaseOffsetsValidatesMembership) {
  EXPECT_THROW(PhaseOffsets({{"l_front", 0.0}}), std::invalid_argument);
}

TEST(Clock, PhaseOffsetsValidatesRange) {
  std::map<std::string, double> bad;
  for (const auto& n : LEG_NAMES) {
    bad[n] = 0.0;
  }
  bad["l_front"] = 1.0;  // not in [0, 1)
  // Extra parens avoid the most-vexing-parse (PhaseOffsets(bad) as a decl).
  EXPECT_THROW((PhaseOffsets(bad)), std::invalid_argument);
}

}  // namespace
}  // namespace hexa_gait

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
