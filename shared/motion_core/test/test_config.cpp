// Host invariant checks for the generated config: the *transformations*
// gen_config.py performs — symmetry expansion, deg->rad conventions, velocity-cap
// derivation, servo wiring integrity — never the tuned values, so a retune cannot
// break this suite. Value-level parity is
// hexa_locomotion/test/test_config_loader.cpp's job; this asks whether the
// generator did the right arithmetic with whatever it read.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "config_generated.hpp"
#include "leg_index.hpp"

namespace cfg = hexa::config;
using hexa::Leg;

namespace {
constexpr float kTol = 1e-5f;
constexpr float kPi = static_cast<float>(M_PI);

const cfg::LegSpec& spec(Leg leg) {
  return cfg::kLegSpecs[static_cast<std::size_t>(leg)];
}

// kGaits rows carry a NUL-padded char array, not a string.
std::string_view gait_name(const cfg::GaitSpec& g) {
  return std::string_view(g.name.data());
}

// The two belly-rest poses share a schema and every invariant below, so the
// suite runs each check over both rather than picking one.
const std::array<std::pair<const char*,
                           const std::array<hexa::JointAngles, hexa::kNumLegs>*>,
                 2>
    kRestPoses = {{
        {"folded_pose", &cfg::kFoldedPose},
        {"initialized_pose", &cfg::kInitializedPose},
    }};
}  // namespace

TEST(LegIndex, NameRoundTrip) {
  EXPECT_EQ(hexa::leg_name(Leg::L_MIDDLE), "l_middle");
  EXPECT_EQ(hexa::leg_name(Leg::R_REAR), "r_rear");
  bool found = false;
  EXPECT_EQ(hexa::leg_from_name("r_rear", found), Leg::R_REAR);
  EXPECT_TRUE(found);
  hexa::leg_from_name("nope", found);
  EXPECT_FALSE(found);
  // Enum order must match the libs' LEG_NAMES.
  EXPECT_EQ(hexa::leg_index(Leg::L_FRONT), 0);
  EXPECT_EQ(hexa::leg_index(Leg::R_REAR), 5);
}

TEST(LegSpecs, SymmetryExpansion) {
  // geometry.yaml carries only the two left reference mounts (l_front,
  // l_middle); the generator expands the other four. Asserting the expansion as
  // relations against those references pins the arithmetic for any mount
  // geometry — the reference coordinates themselves are tuning, not logic.

  // The references must be genuinely off-axis, else every mirror below holds
  // vacuously.
  EXPECT_GT(spec(Leg::L_FRONT).mount_xyz.x, 0.0f);
  EXPECT_GT(spec(Leg::L_FRONT).mount_xyz.y, 0.0f);
  EXPECT_GT(spec(Leg::L_MIDDLE).mount_xyz.y, 0.0f);

  // rear mirrors front about the body y axis: x -> -x, yaw -> pi - yaw.
  EXPECT_NEAR(spec(Leg::L_REAR).mount_xyz.x, -spec(Leg::L_FRONT).mount_xyz.x,
              kTol);
  EXPECT_NEAR(spec(Leg::L_REAR).mount_xyz.y, spec(Leg::L_FRONT).mount_xyz.y,
              kTol);
  EXPECT_NEAR(spec(Leg::L_REAR).mount_xyz.z, spec(Leg::L_FRONT).mount_xyz.z,
              kTol);
  EXPECT_NEAR(spec(Leg::L_REAR).mount_yaw, kPi - spec(Leg::L_FRONT).mount_yaw,
              kTol);

  // right mirrors left about the body x axis: y -> -y, yaw -> -yaw, x kept.
  const std::array<std::pair<Leg, Leg>, 3> mirrored = {{
      {Leg::L_FRONT, Leg::R_FRONT},
      {Leg::L_MIDDLE, Leg::R_MIDDLE},
      {Leg::L_REAR, Leg::R_REAR},
  }};
  for (const auto& [left, right] : mirrored) {
    const auto side = hexa::leg_name(right);
    EXPECT_NEAR(spec(right).mount_xyz.x, spec(left).mount_xyz.x, kTol) << side;
    EXPECT_NEAR(spec(right).mount_xyz.y, -spec(left).mount_xyz.y, kTol) << side;
    EXPECT_NEAR(spec(right).mount_xyz.z, spec(left).mount_xyz.z, kTol) << side;
    EXPECT_NEAR(spec(right).mount_yaw, -spec(left).mount_yaw, kTol) << side;
  }
}

TEST(LegSpecs, SegmentsAreUniform) {
  // All six legs are built from the same three segment lengths, so the
  // generator broadcasting one YAML triple across the array is the invariant —
  // the lengths themselves are geometry.
  const auto& ref = spec(Leg::L_FRONT);
  EXPECT_GT(ref.coxa_len, 0.0f);
  EXPECT_GT(ref.femur_len, 0.0f);
  EXPECT_GT(ref.tibia_len, 0.0f);
  for (std::size_t i = 0; i < cfg::kLegSpecs.size(); ++i) {
    const auto& s = cfg::kLegSpecs[i];
    EXPECT_NEAR(s.coxa_len, ref.coxa_len, kTol) << hexa::LEG_NAMES[i];
    EXPECT_NEAR(s.femur_len, ref.femur_len, kTol) << hexa::LEG_NAMES[i];
    EXPECT_NEAR(s.tibia_len, ref.tibia_len, kTol) << hexa::LEG_NAMES[i];
  }
  EXPECT_GT(cfg::kCoxaToBottom, 0.0f);
}

TEST(JointLimits, BracketBothRestPoses) {
  for (std::size_t j = 0; j < cfg::kJointLimits.size(); ++j) {
    EXPECT_LT(cfg::kJointLimits[j].lower, cfg::kJointLimits[j].upper)
        << "joint " << j;
  }

  // Each joint applies its own deg->rad convention (coxa plain, femur negated,
  // tibia pi - x), and a dropped negation throws the rest pose outside its own
  // travel window. Bounds are inclusive: the femur tuck may sit on its stop.
  //
  // Femur and tibia only — the shipped geometry.yaml deliberately tucks the coxa
  // past its travel window at rest.
  for (const auto& [name, pose] : kRestPoses) {
    for (std::size_t i = 0; i < pose->size(); ++i) {
      for (std::size_t j = 1; j < 3; ++j) {
        const float angle = (*pose)[i][j];
        EXPECT_GE(angle, cfg::kJointLimits[j].lower - kTol)
            << name << " " << hexa::LEG_NAMES[i] << " joint " << j;
        EXPECT_LE(angle, cfg::kJointLimits[j].upper + kTol)
            << name << " " << hexa::LEG_NAMES[i] << " joint " << j;
      }
    }
  }
}

TEST(Pose, RestPosesArePerLegSymmetric) {
  // geometry.yaml gives the coxa tuck for l_front and l_middle only; the rest
  // are mirrored. femur/tibia are uniform across the hexapod.
  for (const auto& [name, pose] : kRestPoses) {
    const auto angle = [&](Leg leg, std::size_t joint) {
      return (*pose)[static_cast<std::size_t>(leg)][joint];
    };
    const float front = angle(Leg::L_FRONT, 0);
    EXPECT_NE(front, 0.0f) << name << ": a zero front tuck makes the mirrors "
                                      "vacuous";
    EXPECT_NEAR(angle(Leg::L_REAR, 0), -front, kTol) << name;
    EXPECT_NEAR(angle(Leg::R_FRONT, 0), -front, kTol) << name;
    EXPECT_NEAR(angle(Leg::R_REAR, 0), front, kTol) << name;
    EXPECT_NEAR(angle(Leg::L_MIDDLE, 0), angle(Leg::R_MIDDLE, 0), kTol) << name;

    for (std::size_t i = 0; i < pose->size(); ++i) {
      EXPECT_NEAR((*pose)[i][1], (*pose)[0][1], kTol)
          << name << " " << hexa::LEG_NAMES[i] << " femur differs from leg 0";
      EXPECT_NEAR((*pose)[i][2], (*pose)[0][2], kTol)
          << name << " " << hexa::LEG_NAMES[i] << " tibia differs from leg 0";
    }
  }
}

TEST(VelocityCaps, DerivedFromEngineKnobs) {
  // linear_max is the stride covered in one stance, so it keys off the realized
  // split rather than the nominal duty: swing_phase_margin hands part of each
  // swing window back to stance. Recomputing it here from kEngine pins the
  // derivation that gen_config.py ports out of pipeline_config_loader.cpp,
  // for every gait rather than just tripod.
  for (const auto& g : cfg::kGaits) {
    // The margin is per LEG SET, and the baked table carries no leg set, so the
    // quadruped gaits are named here. A new one would have to be added — which
    // is the point: its cap must not silently come out on the six-leg margin.
    const bool quadruped =
        gait_name(g) == "quad_walk" || gait_name(g) == "quad_canter";
    const float margin = quadruped ? cfg::kEngine.quadruped_swing_phase_margin
                                   : cfg::kEngine.swing_phase_margin;
    const float swing_end = (1.0f - g.duty_factor) * (1.0f - margin);
    const float want = cfg::kEngine.stride_length * swing_end /
                       (cfg::kEngine.min_swing_time * (1.0f - swing_end));
    EXPECT_NEAR(g.linear_max, want, 1e-6f) << gait_name(g);
    EXPECT_GT(g.linear_max, 0.0f) << gait_name(g);
  }

  // yaw_bias is the raw tuning.yaml knob re-keyed to each gait's nominal duty as
  //   0.5 + (raw - 0.5) * (1.5 - duty)
  // The raw value is consumed at load time and never baked, so recover it as a
  // ratio: it must come out the same for every gait.
  const float ref = (cfg::kGaits[0].yaw_bias - 0.5f) /
                    (1.5f - cfg::kGaits[0].duty_factor);
  for (const auto& g : cfg::kGaits) {
    EXPECT_NEAR((g.yaw_bias - 0.5f) / (1.5f - g.duty_factor), ref, 1e-6f)
        << gait_name(g);
  }
  // Duties must actually differ across the catalogue, else the ratio holds
  // trivially.
  EXPECT_NE(cfg::kGaits.front().duty_factor, cfg::kGaits.back().duty_factor);

  // No angular cap is baked: it is linear_max over the standing stance radius,
  // derived at load time by load_velocity_caps_from_config.
}

TEST(VelocityCaps, StabilityFlags) {
  // surf and crawl are the unstable ones (registry.cpp source of truth — C++,
  // not YAML, so pinning the names and duties here is single-sourced).
  auto by_name = [](std::string_view n) -> const cfg::GaitSpec& {
    for (const auto& g : cfg::kGaits)
      if (gait_name(g) == n) return g;
    ADD_FAILURE() << "gait not found: " << n;
    return cfg::kGaits[0];
  };
  EXPECT_TRUE(by_name("surf").unstable);
  EXPECT_TRUE(by_name("crawl").unstable);
  EXPECT_FALSE(by_name("tetrapod").unstable);
  EXPECT_FALSE(by_name("ripple").unstable);
  EXPECT_NEAR(by_name("ripple").duty_factor, 5.0f / 6.0f, kTol);
}

TEST(Teleop, GaitCycleMatchesRegistry) {
  // kGaitCycle is the runtime rotation baked from teleop_joy.yaml, already
  // filtered by allow_unstable_gaits. Which gaits it lists is a preference; that
  // every entry resolves against the linked registry and honours the filter is
  // the invariant the firmware cycler depends on.
  ASSERT_FALSE(cfg::kGaitCycle.empty());
  std::vector<std::string_view> seen;
  for (const std::string_view name : cfg::kGaitCycle) {
    const auto it =
        std::find_if(cfg::kGaits.begin(), cfg::kGaits.end(),
                     [&](const cfg::GaitSpec& g) { return gait_name(g) == name; });
    ASSERT_NE(it, cfg::kGaits.end()) << name << " is not a registered gait";
    if (!cfg::kAllowUnstableGaits) {
      EXPECT_FALSE(it->unstable)
          << name << " is unstable but unstable gaits are disabled";
    }
    EXPECT_EQ(std::count(seen.begin(), seen.end(), name), 0)
        << name << " appears twice in the cycle";
    seen.push_back(name);
  }
  // The cycler starts on the default gait, so it has to be in the rotation.
  EXPECT_NE(std::find(cfg::kGaitCycle.begin(), cfg::kGaitCycle.end(),
                      cfg::kDefaultGait),
            cfg::kGaitCycle.end())
      << cfg::kDefaultGait << " is not in the cycle";
}

TEST(Posture, PoseEnvelope) {
  // The z pair is ABSOLUTE belly clearance; nominal_body_height is carried
  // alongside so PostureController can convert it to the offsets BodyPose::z
  // is expressed in. Codegen refuses to emit a nominal outside the envelope,
  // and the nominal must track the standing pose it is derived from.
  EXPECT_NEAR(cfg::kPosture.nominal_body_height, cfg::kStandingPose.body_height,
              kTol);
  EXPECT_GT(cfg::kPosture.body_height_min, 0.0f);
  EXPECT_LT(cfg::kPosture.body_height_min, cfg::kPosture.nominal_body_height);
  EXPECT_LT(cfg::kPosture.nominal_body_height, cfg::kPosture.body_height_max);

  // Pose limits are magnitudes — a non-positive one would clamp its axis dead.
  EXPECT_GT(cfg::kPosture.pose_limit_x, 0.0f);
  EXPECT_GT(cfg::kPosture.pose_limit_y, 0.0f);
  EXPECT_GT(cfg::kPosture.pose_limit_roll, 0.0f);
  EXPECT_GT(cfg::kPosture.pose_limit_pitch, 0.0f);
  EXPECT_GT(cfg::kPosture.pose_limit_yaw, 0.0f);
}

TEST(Posture, PoseFilter) {
  // tau <= 0 is a legal bypass, so the floor is not "positive" but "far enough
  // above the tick period to integrate cleanly" — below ~4 ticks a tau runs
  // into PoseSmoother's omega*dt stability cap instead of meaning what it says.
  constexpr float kDt = 0.005f;  // hexa::pipeline::kDt
  EXPECT_GT(cfg::kPosture.pose_filter_tau, 4.0f * kDt);

  // zeta = 0 is an undamped oscillator that never settles.
  EXPECT_GT(cfg::kPosture.pose_filter_damping_ratio, 0.0f);
}

TEST(Hardware, ServoCalibration) {
  ASSERT_EQ(cfg::kJointCals.size(), 18u);

  // Row order is the pipeline's theta[] order (LEG_NAMES x {coxa, femur, tibia}),
  // NOT pin order — kJointCals[i] must calibrate theta[i]. The wiring is carried
  // per row in `pin`. Which pin a joint lands on is hardware.yaml's business; that
  // the table is a consistent, physically realisable harness is this test's.
  for (std::size_t i = 0; i < cfg::kJointCals.size(); ++i) {
    const auto& j = cfg::kJointCals[i];
    EXPECT_TRUE(j.direction == 1 || j.direction == -1)
        << "row " << i << " direction " << static_cast<int>(j.direction);
    EXPECT_LT(j.min_us, j.max_us) << "row " << i;
    // Endpoints are measured magnitudes — they move every calibration run, so
    // only the invariants are pinned: both inside the electrical clamp, and
    // distinct (equal endpoints would make the slope zero).
    EXPECT_GT(j.us_at_plus_45, static_cast<float>(j.min_us)) << "row " << i;
    EXPECT_LT(j.us_at_plus_45, static_cast<float>(j.max_us)) << "row " << i;
    EXPECT_GT(j.us_at_minus_45, static_cast<float>(j.min_us)) << "row " << i;
    EXPECT_LT(j.us_at_minus_45, static_cast<float>(j.max_us)) << "row " << i;
    EXPECT_NE(j.us_at_plus_45, j.us_at_minus_45) << "row " << i;
  }
  // Every servo pin is wired exactly once.
  bool used[19] = {};
  for (const auto& j : cfg::kJointCals) {
    ASSERT_GE(j.pin, 1);
    ASSERT_LE(j.pin, 18);
    EXPECT_FALSE(used[j.pin]) << "pin " << static_cast<int>(j.pin) << " wired twice";
    used[j.pin] = true;
  }
  // urdf_rad_at_center comes from hardware.yaml deg_at_center, which is uniform
  // per segment across the build — so every leg's coxa/femur/tibia centre must
  // agree with leg 0's. The angles themselves are a property of the physical
  // build, not of the generator.
  for (std::size_t i = 0; i < cfg::kJointCals.size(); ++i) {
    EXPECT_NEAR(cfg::kJointCals[i].urdf_rad_at_center,
                cfg::kJointCals[i % 3].urdf_rad_at_center, kTol)
        << "row " << i << " segment centre differs from leg 0";
  }
  // Chica command-index map + scales per protocol.md (hexapod-servo2040-driver):
  // CURR=24, VOLT=25 (consecutive), RELAY=26, STATUS=27; telemetry in 0.01
  // centi-units; STATUS trip current at 0.1 A/count. These are literals in
  // gen_config.py mirroring an external wire contract, not repo YAML, so pinning
  // them is single-sourced against the driver spec.
  EXPECT_EQ(cfg::kRelayPin, 26);
  EXPECT_EQ(cfg::kBatteryCurrentPin, 24);
  EXPECT_EQ(cfg::kBatteryVoltagePin, 25);
  EXPECT_EQ(cfg::kStatusPin, 27);
  EXPECT_NEAR(cfg::kBatteryVoltageScale, 0.01f, 1e-7f);
  EXPECT_NEAR(cfg::kBatteryCurrentScale, 0.01f, 1e-7f);
  EXPECT_NEAR(cfg::kTripAmpsPerCount, 0.1f, 1e-7f);
}

TEST(Hardware, LegsAreWiredRearToFront) {
  // Sorting the rows by pin must yield the harness order the energize sweep
  // brings legs up in. Both consumers derive this the same way — the firmware's
  // servo_out leg_pin_order() off kJointCals, hexa_hardware's build_leg_order()
  // off hardware.yaml — so pinning it here pins both. Rear-first is a stability
  // requirement of the sweep, so a rewire should fail loudly.
  std::array<std::size_t, hexa::kNumLegs> order{};
  for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
  auto lowest_pin = [](std::size_t leg) {
    std::uint8_t p = cfg::kJointCals[leg * 3].pin;
    for (std::size_t k = 1; k < 3; ++k) {
      p = std::min(p, cfg::kJointCals[leg * 3 + k].pin);
    }
    return p;
  };
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return lowest_pin(a) < lowest_pin(b);
  });

  std::vector<std::string_view> names;
  for (const std::size_t leg : order) {
    names.push_back(hexa::LEG_NAMES[leg]);
  }
  EXPECT_EQ(names, (std::vector<std::string_view>{"l_rear", "r_rear", "l_middle",
                                                  "r_middle", "l_front",
                                                  "r_front"}));
  // Each leg's three joints sit on consecutive pins, so a leg is one SET frame.
  for (std::size_t leg = 0; leg < hexa::kNumLegs; ++leg) {
    EXPECT_EQ(cfg::kJointCals[leg * 3 + 1].pin, cfg::kJointCals[leg * 3].pin + 1)
        << hexa::LEG_NAMES[leg];
    EXPECT_EQ(cfg::kJointCals[leg * 3 + 2].pin, cfg::kJointCals[leg * 3].pin + 2)
        << hexa::LEG_NAMES[leg];
  }
}

TEST(FacePanel, GpiosAreDistinctAndUnclaimed) {
  // The SH1122 face panel shares the RP2350's GPIO bank with the Servo 2040
  // Chica link and the CYW43 radio. Which pins the panel uses is display.yaml's
  // business; that they collide with nothing is this suite's. A silent overlap
  // would show up as a dead panel or a dead servo link, whichever lost the
  // gpio_set_function race — not as a build error.
  const std::array<std::uint8_t, 5> panel = {cfg::kFacePanel.sck,
                                             cfg::kFacePanel.mosi,
                                             cfg::kFacePanel.cs,
                                             cfg::kFacePanel.dc,
                                             cfg::kFacePanel.rst};
  for (std::size_t i = 0; i < panel.size(); ++i) {
    for (std::size_t j = i + 1; j < panel.size(); ++j) {
      EXPECT_NE(panel[i], panel[j])
          << "face panel GPIO " << static_cast<int>(panel[i]) << " used twice";
    }
  }

  // uart0 GP0/GP1 is the Chica link (servo_out.cpp); GP23/24/25/29 are the
  // CYW43 radio's on a Pico 2 W and are not ours to drive.
  for (const std::uint8_t pin : panel) {
    EXPECT_NE(pin, 0) << "face panel collides with the Chica UART TX";
    EXPECT_NE(pin, 1) << "face panel collides with the Chica UART RX";
    for (const std::uint8_t radio : {23, 24, 25, 29}) {
      EXPECT_NE(pin, radio) << "face panel collides with the CYW43 radio";
    }
  }

  EXPECT_LE(cfg::kFacePanel.spi_index, 1u) << "spi_index must select spi0 or spi1";
}

// ── Quadruped mode's stance ──
// There is no parked pose here: the middle pair parks at kFoldedPose, which the
// rest-pose suite above already pins.

// The whole static-margin analysis rests on the four corners forming a
// rectangle centred on the body origin, which needs the front and rear groups
// to share a reach and a splay: standing_pose_from negates the rear splay, and
// cos(150 deg - t) = -cos(30 deg + t) makes the two cancel exactly.
TEST(QuadStance, FrontAndRearAreSymmetric) {
  const auto& front = cfg::kQuadStandingPose.front;
  const auto& rear = cfg::kQuadStandingPose.rear;
  EXPECT_NEAR(front.tip_reach, rear.tip_reach, kTol);
  EXPECT_NEAR(front.coxa, rear.coxa, kTol);
  // Pulled in from the six-leg stance to buy leg reach for the support shift.
  EXPECT_LT(front.tip_reach, cfg::kStandingPose.groups[0].tip_reach);
  EXPECT_NEAR(cfg::kQuadStandingPose.body_height,
              cfg::kStandingPose.body_height, kTol);
}

