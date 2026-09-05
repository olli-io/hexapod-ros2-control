// The BodyVelocityLimiter's constant-max-accel slew, and the Control stage's
// scale-to-envelope + limiter-reset-on-leaving-walking wiring.

#include <cmath>
#include <map>
#include <string>
#include <tuple>

#include <gtest/gtest.h>

#include "config_generated.hpp"
#include "control.hpp"
#include "gait/engine.hpp"

namespace {

namespace ctl = hexa::control;
using hexa::gait::EngineState;

// A six-leg standing stance to shape the envelope against. Concentric on
// purpose: every foot at the same radius makes the pure-yaw ceiling exactly
// linear_max / kStanceRadius, which keeps the expectations below arithmetic
// rather than geometry. Control never solves IK, so this test does not link the
// engine — the stance comes in through the constructor.
constexpr float kStanceRadius = 0.2f;

std::map<std::string, hexa::Vec3> test_stance() {
  std::map<std::string, hexa::Vec3> stance;
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    const float angle =
        2.0f * static_cast<float>(M_PI) * static_cast<float>(i) / 6.0f;
    stance[hexa::gait::LEG_NAMES[i]] =
        hexa::Vec3(kStanceRadius * std::cos(angle),
                   kStanceRadius * std::sin(angle), 0.0f);
  }
  return stance;
}

hexa::gait::VelocityCaps test_caps() {
  return hexa::gait::load_velocity_caps_from_config(
      ::hexa::config::kDefaultPreset, kStanceRadius);
}

ctl::Control make_control() {
  // No leg contexts: this fixture's stance is a synthetic regular hexagon, not
  // the real geometry, so there is no leg axis to price a heading against. An
  // empty map leaves the radial budget inert and the envelope cut isotropic,
  // which is what these tests are about.
  return ctl::Control(::hexa::config::kControl, test_caps(), test_stance(), {},
                      hexa::config::kPresets[hexa::config::kDefaultPreset].stride_length,
                      hexa::config::kPresets[hexa::config::kDefaultPreset].stride_length_radial,
                      std::string(::hexa::config::kDefaultGait));
}

TEST(BodyVelocityLimiter, LinearRampReachesTargetAtBoundedRate) {
  ctl::BodyVelocityLimiter lim(2.0f, 10.0f);  // accel_linear=2 m/s^2
  const float dt = 0.1f;                      // max_step = 0.2 m/s per tick
  auto [vx, vy, w] = lim.step(1.0f, 0.0f, 0.0f, dt);
  EXPECT_NEAR(vx, 0.2f, 1e-6f);
  EXPECT_NEAR(vy, 0.0f, 1e-6f);
  for (int i = 0; i < 4; ++i) {
    std::tie(vx, vy, w) = lim.step(1.0f, 0.0f, 0.0f, dt);
  }
  EXPECT_NEAR(vx, 1.0f, 1e-6f);  // 5 steps * 0.2 = 1.0
  // Once the target is reached it stays put.
  std::tie(vx, vy, w) = lim.step(1.0f, 0.0f, 0.0f, dt);
  EXPECT_NEAR(vx, 1.0f, 1e-6f);
}

TEST(BodyVelocityLimiter, LinearSlewIsVectorial) {
  ctl::BodyVelocityLimiter lim(1.0f, 10.0f);
  const float dt = 0.1f;  // max_step = 0.1 along the (dx, dy) vector
  auto [vx, vy, w] = lim.step(3.0f, 4.0f, 0.0f, dt);
  (void)w;
  // Advances 0.1 along the unit vector (0.6, 0.8).
  EXPECT_NEAR(vx, 0.06f, 1e-6f);
  EXPECT_NEAR(vy, 0.08f, 1e-6f);
  EXPECT_NEAR(std::hypot(vx, vy), 0.1f, 1e-6f);
}

TEST(BodyVelocityLimiter, FlipThroughZeroTraversesAtOneBoundedRate) {
  ctl::BodyVelocityLimiter lim(2.0f, 10.0f);
  lim.reset(1.0f, 0.0f, 0.0f);
  const float dt = 0.1f;  // max_step = 0.2
  float prev = 1.0f;
  bool crossed_zero = false;
  for (int i = 0; i < 10; ++i) {
    auto [vx, vy, w] = lim.step(-1.0f, 0.0f, 0.0f, dt);
    (void)vy;
    (void)w;
    // Each step moves by at most 0.2 (no unbounded snap at the zero crossing).
    EXPECT_LE(std::fabs(vx - prev), 0.2f + 1e-6f);
    if (std::fabs(vx) < 1e-6f) crossed_zero = true;
    prev = vx;
  }
  EXPECT_TRUE(crossed_zero);
  EXPECT_NEAR(prev, -1.0f, 1e-6f);
}

TEST(BodyVelocityLimiter, SnapsSubToleranceToZero) {
  ctl::BodyVelocityLimiter lim(10.0f, 10.0f, 1e-3f, 1e-3f);
  lim.reset(5e-4f, 0.0f, 5e-4f);
  auto [vx, vy, w] = lim.step(5e-4f, 0.0f, 5e-4f, 0.1f);
  EXPECT_EQ(vx, 0.0f);
  EXPECT_EQ(vy, 0.0f);
  EXPECT_EQ(w, 0.0f);
}

TEST(BodyVelocityLimiter, AngularScalarSlew) {
  ctl::BodyVelocityLimiter lim(10.0f, 1.0f);  // accel_angular=1 rad/s^2
  const float dt = 0.1f;                      // max_step = 0.1
  auto [vx, vy, w] = lim.step(0.0f, 0.0f, 1.0f, dt);
  (void)vx;
  (void)vy;
  EXPECT_NEAR(w, 0.1f, 1e-6f);
}

TEST(BodyVelocityLimiter, RejectsNonPositiveAccel) {
  EXPECT_THROW(ctl::BodyVelocityLimiter(0.0f, 1.0f), std::invalid_argument);
  EXPECT_THROW(ctl::BodyVelocityLimiter(1.0f, -1.0f), std::invalid_argument);
}

// A pure forward command well beyond the envelope settles at the active gait's
// linear cap (omega=0 collapses scale_to_envelope to |v_x| <= linear_max).
TEST(Control, SettlesAtGaitLinearCap) {
  ctl::Control control = make_control();
  const auto caps = test_caps();
  const float cap = caps.linear_max(control.active_gait());
  float vx = 0.0f, vy = 0.0f, w = 0.0f;
  for (int i = 0; i < 200; ++i) {
    std::tie(vx, vy, w) = control.shape(5.0f, 0.0f, 0.0f, EngineState::GAIT, 0.02f);
  }
  EXPECT_NEAR(vx, cap, 1e-3f);
  EXPECT_NEAR(vy, 0.0f, 1e-3f);
}

// Leaving the walking set ({ENGAGING, GAIT}) resets the limiter, so the shaped
// velocity drops to exactly zero on the first non-walking tick.
TEST(Control, ResetsLimiterOnLeavingWalking) {
  ctl::Control control = make_control();
  float vx = 0.0f, vy = 0.0f, w = 0.0f;
  for (int i = 0; i < 50; ++i) {
    std::tie(vx, vy, w) = control.shape(0.3f, 0.0f, 0.0f, EngineState::GAIT, 0.02f);
  }
  EXPECT_GT(vx, 0.05f);  // ramped up while walking
  std::tie(vx, vy, w) = control.shape(0.0f, 0.0f, 0.0f, EngineState::STAND, 0.02f);
  EXPECT_EQ(vx, 0.0f);
  EXPECT_EQ(vy, 0.0f);
  EXPECT_EQ(w, 0.0f);
}

// A gait switch recomputes both accel caps so the ramp times stay constant.
TEST(Control, GaitSwitchRecomputesBothAccels) {
  ctl::Control control = make_control();
  const auto caps = test_caps();
  const float t_lin = ::hexa::config::kControl.vmax_ramp_time_linear;
  const float t_ang = ::hexa::config::kControl.vmax_ramp_time_angular;
  EXPECT_NEAR(control.limiter().accel_linear(),
              caps.linear_max(control.active_gait()) / t_lin, 1e-6f);
  EXPECT_NEAR(control.limiter().accel_angular(),
              caps.angular_max(control.active_gait()) / t_ang, 1e-6f);
  control.set_gait("ripple");
  EXPECT_EQ(control.active_gait(), "ripple");
  EXPECT_NEAR(control.limiter().accel_linear(),
              caps.linear_max("ripple") / t_lin, 1e-6f);
  // The angular cap is per-gait too — it used to be a single shared scalar, so
  // this half of the switch silently did nothing.
  EXPECT_NEAR(control.limiter().accel_angular(),
              caps.angular_max("ripple") / t_ang, 1e-6f);
}

// The yaw ramp takes vmax_ramp_time_angular to reach the cap, which is the
// whole point of deriving the cap instead of tuning it: an over-large
// angular_z_max used to make this ~4x faster than the configured ramp.
TEST(Control, YawRampTakesTheConfiguredTimeToReachTheCap) {
  ctl::Control control = make_control();
  const auto caps = test_caps();
  const float cap = caps.angular_max(control.active_gait());
  const float t_ang = ::hexa::config::kControl.vmax_ramp_time_angular;
  const float dt = 0.005f;

  float vx = 0.0f, vy = 0.0f, w = 0.0f;
  int ticks = 0;
  const int limit = static_cast<int>(4.0f * t_ang / dt);
  while (ticks < limit) {
    std::tie(vx, vy, w) = control.shape(0.0f, 0.0f, 10.0f, EngineState::GAIT, dt);
    ++ticks;
    if (w >= cap - 1e-4f) break;
  }
  EXPECT_NEAR(w, cap, 1e-3f);
  EXPECT_NEAR(static_cast<float>(ticks) * dt, t_ang, 0.05f);
}

// A yaw command far past the envelope settles at linear_max / stance_radius
// with no separate angular clamp involved.
TEST(Control, SettlesAtTheGeometricYawCeiling) {
  ctl::Control control = make_control();
  const auto caps = test_caps();
  const float cap = caps.angular_max(control.active_gait());
  // Concentric stance, so the ceiling is exactly the linear cap over the radius.
  EXPECT_NEAR(cap, caps.linear_max(control.active_gait()) / kStanceRadius,
              1e-6f);
  float vx = 0.0f, vy = 0.0f, w = 0.0f;
  for (int i = 0; i < 2000; ++i) {
    std::tie(vx, vy, w) = control.shape(0.0f, 0.0f, 50.0f, EngineState::GAIT, 0.005f);
  }
  EXPECT_NEAR(w, cap, 1e-3f);
  EXPECT_NEAR(vx, 0.0f, 1e-3f);
  EXPECT_NEAR(vy, 0.0f, 1e-3f);
}

}  // namespace
