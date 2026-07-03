// gtest port of hexa_gait/test/test_limits.py — exercises the pure C++
// VelocityCaps loader and scale_to_envelope (no ROS). Behavioural parity with
// the Python suite. Python's math.isclose default rel_tol=1e-9 maps to
// EXPECT_NEAR(..., 1e-9); pytest.raises(KeyError) on unknown gait maps to
// std::out_of_range, while a missing YAML key surfaces as YAML::Exception.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <tuple>

#include <yaml-cpp/yaml.h>

#include "hexa_gait_cpp/limits.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {
namespace {

// Gait-agnostic knobs written to a temp gait.yaml. Duty factors are sourced from
// the strategy classes in the registry, not YAML — this only carries the shared
// knobs. Mirrors _write_yaml's base dict; callers mutate fields to override.
struct GaitYaml {
  double stride_length = 0.12;
  double min_swing_time = 0.30;
  double max_swing_time = 1.0;
  double step_height = 0.035;
  double swing_width = 0.0;
  double controller_dt = 0.02;
  double cmd_zero_tol = 1.0e-4;
  double max_reset_time = 0.6;
  double angular_z_max = 1.0;
  double yaw_bias = 0.75;
};

std::string write_yaml(const GaitYaml& g = {}) {
  const std::string path = std::string(::testing::TempDir()) + "/gait.yaml";
  std::ofstream f(path);
  f << std::setprecision(17);
  // gait.yaml is a ros2 params file; load_velocity_caps unwraps this.
  f << "gait_node:\n  ros__parameters:\n";
  f << "    stride_length: " << g.stride_length << "\n";
  f << "    min_swing_time: " << g.min_swing_time << "\n";
  f << "    max_swing_time: " << g.max_swing_time << "\n";
  f << "    step_height: " << g.step_height << "\n";
  f << "    swing_width: " << g.swing_width << "\n";
  f << "    controller_dt: " << g.controller_dt << "\n";
  f << "    cmd_zero_tol: " << g.cmd_zero_tol << "\n";
  f << "    max_reset_time: " << g.max_reset_time << "\n";
  f << "    angular_z_max: " << g.angular_z_max << "\n";
  f << "    yaw_bias: " << g.yaw_bias << "\n";
  return path;
}

// Write an arbitrary raw YAML body (for the missing-key tests). The body's
// flat lines are indented under the gait_node/ros__parameters wrapper so the
// loader's unwrap applies, matching the real gait.yaml layout.
std::string write_raw(const std::string& body) {
  const std::string path = std::string(::testing::TempDir()) + "/gait.yaml";
  std::ofstream f(path);
  f << "gait_node:\n  ros__parameters:\n";
  std::istringstream in(body);
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty()) {
      f << "    " << line << "\n";
    }
  }
  return path;
}

// ── VelocityCaps loader ────────────────────────────────────────────────────

TEST(Limits, LinearMaxTripodDerivedFromStrideSwingTimeAndDuty) {
  // tripod linear_max = 0.12 * (1 - 0.5) / (0.30 * 0.5) = 0.40 m/s.
  const auto caps = load_velocity_caps(write_yaml());
  EXPECT_NEAR(caps.linear_max("tripod"), 0.40, 1e-9);
}

TEST(Limits, LinearMaxPerGaitStrictlyDecreasingWithDuty) {
  // Slower gait (higher beta) gives a lower linear cap because the
  // swing window shrinks while the stance window grows.
  const auto caps = load_velocity_caps(write_yaml());
  // crawl  = 0.12 * (1/3) / (0.30 * 2/3) = 0.20 m/s
  // ripple = 0.12 * (1/6) / (0.30 * 5/6) = 0.08 m/s
  EXPECT_NEAR(caps.linear_max("crawl"), 0.20, 1e-9);
  EXPECT_NEAR(caps.linear_max("ripple"), 0.08, 1e-9);
  EXPECT_GT(caps.linear_max("tripod"), caps.linear_max("crawl"));
  EXPECT_GT(caps.linear_max("crawl"), caps.linear_max("ripple"));
}

TEST(Limits, LinearMaxUnknownGaitRaises) {
  // Per-gait caps fail fast on typos rather than silently falling back — the
  // control layer must agree with the registry names.
  const auto caps = load_velocity_caps(write_yaml());
  EXPECT_THROW(caps.linear_max("gallop"), std::out_of_range);
}

TEST(Limits, LinearMaxScalesWithStrideLength) {
  // Double stride_length -> double linear_max for every gait.
  GaitYaml g;
  g.stride_length = 0.24;
  const auto caps = load_velocity_caps(write_yaml(g));
  EXPECT_NEAR(caps.linear_max("tripod"), 0.80, 1e-9);
  EXPECT_NEAR(caps.linear_max("ripple"), 0.16, 1e-9);
}

TEST(Limits, LinearMaxScalesInverselyWithMinSwingTime) {
  // Slower min_swing_time -> lower linear_max.
  GaitYaml g;
  g.min_swing_time = 0.60;
  const auto caps = load_velocity_caps(write_yaml(g));
  EXPECT_NEAR(caps.linear_max("tripod"), 0.20, 1e-9);
}

TEST(Limits, AngularMaxPassesThroughRaw) {
  // angular_z_max is an explicit knob, not derived from geometry.
  GaitYaml g;
  g.angular_z_max = 1.5;
  const auto caps = load_velocity_caps(write_yaml(g));
  EXPECT_NEAR(caps.angular_max, 1.5, 1e-9);
}

TEST(Limits, YawBiasAnchorsAtTripodAndEasesWithDuty) {
  // yaw_bias is per-gait, easing back toward neutral as beta grows:
  //   yaw_bias_eff(beta) = 0.5 + (yaw_bias_yaml - 0.5) * (1.5 - beta)
  // The YAML value anchors at tripod (beta=0.5); slower gaits sit closer to
  // neutral because their smaller linear_max can't absorb an aggressive cut on
  // top of the gait's intrinsic slowness.
  GaitYaml g;
  g.yaw_bias = 0.6;
  const auto caps = load_velocity_caps(write_yaml(g));
  EXPECT_NEAR(caps.yaw_bias("tripod"), 0.60, 1e-9);
  // crawl beta=2/3 -> 0.5 + 0.1 * (1.5 - 2/3) = 0.5833
  EXPECT_NEAR(caps.yaw_bias("crawl"), 0.5 + 0.1 * (1.5 - 2.0 / 3.0), 1e-9);
  // ripple beta=5/6 -> 0.5 + 0.1 * (1.5 - 5/6) = 0.5667
  EXPECT_NEAR(caps.yaw_bias("ripple"), 0.5 + 0.1 * (1.5 - 5.0 / 6.0), 1e-9);
  // Strict monotone: deviation shrinks as duty grows.
  auto dev = [&](const std::string& name) { return caps.yaw_bias(name) - 0.5; };
  EXPECT_GT(dev("tripod"), dev("crawl"));
  EXPECT_GT(dev("crawl"), dev("ripple"));
  EXPECT_GT(dev("ripple"), 0.0);
}

TEST(Limits, YawBiasUniformWhenYamlIsNeutral) {
  // yaw_bias_yaml = 0.5 => no deviation => every gait stays at 0.5.
  GaitYaml g;
  g.yaw_bias = 0.5;
  const auto caps = load_velocity_caps(write_yaml(g));
  for (const std::string& name : {"tripod", "crawl", "ripple"}) {
    EXPECT_NEAR(caps.yaw_bias(name), 0.5, 1e-9);
  }
}

TEST(Limits, YawBiasUnknownGaitRaises) {
  const auto caps = load_velocity_caps(write_yaml());
  EXPECT_THROW(caps.yaw_bias("gallop"), std::out_of_range);
}

TEST(Limits, MissingAngularZMaxRaises) {
  const std::string path = write_raw(
      "stride_length: 0.12\n"
      "min_swing_time: 0.30\n"
      "yaw_bias: 0.75\n");
  EXPECT_THROW(load_velocity_caps(path), YAML::Exception);
}

TEST(Limits, MissingYawBiasRaises) {
  const std::string path = write_raw(
      "stride_length: 0.12\n"
      "min_swing_time: 0.30\n"
      "angular_z_max: 1.0\n");
  EXPECT_THROW(load_velocity_caps(path), YAML::Exception);
}

TEST(Limits, AcceptsStringPath) {
  // Trivial in C++: load_velocity_caps already takes std::string. Kept as a
  // parity duplicate of the Python str(path) test.
  const auto caps = load_velocity_caps(write_yaml());
  EXPECT_NEAR(caps.linear_max("tripod"), 0.40, 1e-9);
}

// ── scale_to_envelope ──────────────────────────────────────────────────────

// Mount positions matching geometry.yaml's expansion. Only the (r_x, r_y)
// components are read; r_z is ignored. Symmetric six-leg hexapod.
const std::map<std::string, Vec3>& mounts() {
  static const std::map<std::string, Vec3> m = {
      {"l_front", Vec3(0.083, 0.0575, 0.0)},
      {"l_middle", Vec3(0.0, 0.082, 0.0)},
      {"l_rear", Vec3(-0.083, 0.0575, 0.0)},
      {"r_front", Vec3(0.083, -0.0575, 0.0)},
      {"r_middle", Vec3(0.0, -0.082, 0.0)},
      {"r_rear", Vec3(-0.083, -0.0575, 0.0)},
  };
  return m;
}

constexpr double kLinearMax = 0.40;
constexpr double kAngularMax = 1.0;
constexpr double kYawBias = 0.75;
constexpr double kUniformBias = 0.5;

// Max per-leg planar speed for a commanded triple, over all mounts.
double max_leg_speed(double v_x, double v_y, double omega_z) {
  double max_v = 0.0;
  for (const auto& [name, r] : mounts()) {
    (void)name;
    max_v = std::max(max_v, std::hypot(v_x - omega_z * r[1], v_y + omega_z * r[0]));
  }
  return max_v;
}

TEST(Limits, ScalePassthroughWhenWithinEnvelope) {
  // Modest forward + modest yaw — every leg under 0.40 m/s.
  auto [v_x, v_y, omega_z] = scale_to_envelope(
      0.1, 0.0, 0.5, mounts(), kLinearMax, kAngularMax, kYawBias);
  EXPECT_NEAR(v_x, 0.1, 1e-9);
  EXPECT_NEAR(v_y, 0.0, 1e-9);
  EXPECT_NEAR(omega_z, 0.5, 1e-9);
}

TEST(Limits, ScaleZeroCommandStaysZero) {
  auto [v_x, v_y, omega_z] = scale_to_envelope(
      0.0, 0.0, 0.0, mounts(), kLinearMax, kAngularMax, kYawBias);
  EXPECT_DOUBLE_EQ(v_x, 0.0);
  EXPECT_DOUBLE_EQ(v_y, 0.0);
  EXPECT_DOUBLE_EQ(omega_z, 0.0);
}

TEST(Limits, ScalePureLinearAtCapUnchanged) {
  // v_x = linear_max, no yaw: max leg speed equals the cap exactly, so the cut
  // must be a no-op.
  auto [v_x, v_y, omega_z] = scale_to_envelope(
      0.40, 0.0, 0.0, mounts(), kLinearMax, kAngularMax, kYawBias);
  EXPECT_NEAR(v_x, 0.40, 1e-9);
  EXPECT_NEAR(v_y, 0.0, 1e-9);
  EXPECT_NEAR(omega_z, 0.0, 1e-9);
}

TEST(Limits, ScaleUniformBiasPreservesRatioAtFullForwardPlusFullYaw) {
  // yaw_bias = 0.5 => rho = 1 => uniform scaling — the cut falls equally on v_x
  // and omega_z, so the commanded translation:yaw ratio survives. Regression
  // guard for the unbiased baseline.
  auto [v_x, v_y, omega_z] = scale_to_envelope(
      0.40, 0.0, 1.0, mounts(), kLinearMax, kAngularMax, kUniformBias);
  EXPECT_NEAR(max_leg_speed(v_x, v_y, omega_z), kLinearMax, 1e-9);
  // 0.40 / 1.0 = 0.40 ratio preserved.
  EXPECT_NEAR(v_x / omega_z, 0.40, 1e-9);
  EXPECT_NEAR(v_y, 0.0, 1e-9);
}

TEST(Limits, ScaleBiasedCutFavoursYawAtFullForwardPlusFullYaw) {
  // yaw_bias = 0.75 => rho = 3: at the cut, translation absorbs three times the
  // cut fraction omega does. The resulting v_x sits well below uniform, omega
  // sits well above, and the binding leg is at the per-leg cap exactly.
  auto [v_x_u, v_y_u, omega_u] = scale_to_envelope(
      0.40, 0.0, 1.0, mounts(), kLinearMax, kAngularMax, kUniformBias);
  (void)v_y_u;
  auto [v_x_b, v_y_b, omega_b] = scale_to_envelope(
      0.40, 0.0, 1.0, mounts(), kLinearMax, kAngularMax, kYawBias);

  EXPECT_LT(v_x_b, v_x_u);
  EXPECT_GT(omega_b, omega_u);
  EXPECT_NEAR(v_y_b, 0.0, 1e-9);

  // rho = 0.75 / 0.25 = 3 => (1 - s_v) / (1 - s_w) = 3.
  const double s_v = v_x_b / 0.40;
  const double s_w = omega_b / 1.0;
  EXPECT_NEAR((1.0 - s_v) / (1.0 - s_w), 3.0, 1e-9);

  // Binding leg lands on the per-leg cap, no overshoot.
  EXPECT_NEAR(max_leg_speed(v_x_b, v_y_b, omega_b), kLinearMax, 1e-9);
}

TEST(Limits, ScaleClampsOmegaToAngularMaxFirst) {
  // omega beyond cap is clamped before the cut. With v=0 and omega clamped to
  // 1.0, max leg speed is omega * r_outer ~ 0.101, well under linear_max, so no
  // further cut happens.
  auto [v_x, v_y, omega_z] = scale_to_envelope(
      0.0, 0.0, 5.0, mounts(), kLinearMax, kAngularMax, kYawBias);
  EXPECT_NEAR(v_x, 0.0, 1e-9);
  EXPECT_NEAR(v_y, 0.0, 1e-9);
  EXPECT_NEAR(omega_z, kAngularMax, 1e-9);
}

TEST(Limits, ScaleClampsNegativeOmega) {
  auto [v_x, v_y, omega_z] = scale_to_envelope(
      0.0, 0.0, -5.0, mounts(), kLinearMax, kAngularMax, kYawBias);
  (void)v_x;
  (void)v_y;
  EXPECT_NEAR(omega_z, -kAngularMax, 1e-9);
}

TEST(Limits, ScaleBiasedCutFavoursYawForLateralPlusYaw) {
  // v_y exercises the r_x-coupled term — the cut split has to handle the lateral
  // direction the same way as forward.
  auto [v_x_b, v_y_b, omega_b] = scale_to_envelope(
      0.0, 0.40, 1.0, mounts(), kLinearMax, kAngularMax, kYawBias);
  EXPECT_NEAR(max_leg_speed(v_x_b, v_y_b, omega_b), kLinearMax, 1e-9);
  EXPECT_NEAR(v_x_b, 0.0, 1e-9);
  const double s_v = v_y_b / 0.40;
  const double s_w = omega_b / 1.0;
  EXPECT_NEAR((1.0 - s_v) / (1.0 - s_w), 3.0, 1e-9);
}

TEST(Limits, ScaleYawOnlyViolationZerosTranslation) {
  // Slow-gait corner: angular_max * r_outer exceeds linear_max, so omega alone
  // breaks the per-leg envelope. The bias-toward-yaw contract pins translation
  // at zero and scales omega to fit.
  const double tiny_linear = 0.05;  // angular_max * r_outer ~ 0.101 > 0.05
  auto [v_x, v_y, omega] = scale_to_envelope(
      0.10, 0.0, 1.0, mounts(), tiny_linear, kAngularMax, kYawBias);
  EXPECT_NEAR(v_x, 0.0, 1e-9);
  EXPECT_NEAR(v_y, 0.0, 1e-9);
  // omega scales to tiny_linear / (angular_max * r_outer).
  double r_outer = 0.0;
  for (const auto& [name, r] : mounts()) {
    (void)name;
    r_outer = std::max(r_outer, std::hypot(r[0], r[1]));
  }
  EXPECT_NEAR(omega, tiny_linear / r_outer, 1e-9);
}

TEST(Limits, ScaleUsesPerGaitLinearMax) {
  // The whole point of the refactor: passing a smaller linear_max (e.g.
  // ripple's cap) cuts the command down accordingly.
  const auto caps = load_velocity_caps(write_yaml());
  auto [v_x, v_y, omega] = scale_to_envelope(
      0.40, 0.0, 0.0, mounts(), caps.linear_max("ripple"), caps.angular_max,
      caps.yaw_bias("ripple"));
  (void)v_y;
  // 0.40 was tripod's cap; ripple cap is 0.08, so the input gets scaled to 0.08
  // (no yaw -> max leg speed = |v_x|, bias is irrelevant).
  EXPECT_NEAR(v_x, 0.08, 1e-9);
  EXPECT_NEAR(omega, 0.0, 1e-9);
}

}  // namespace
}  // namespace hexa_gait

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
