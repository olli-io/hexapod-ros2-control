// Prints the generated config for eyeball verification (plan part 04).
//
// Chiefly the symmetry-expanded leg specs — the plan's part-04 acceptance step
// is "print all 6 LegSpecs, confirm r_rear = (-0.083, -0.0575, -150deg)". Also
// dumps the derived per-gait velocity caps. Not a test; a human-readable dump.

#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <string_view>

#include "config_generated.hpp"
#include "gait/engine.hpp"  // standing_pose_from_config / nominal_stance_from_config
#include "gait/limits.hpp"  // outer_stance_radius / load_velocity_caps_from_config
#include "leg_index.hpp"

int main() {
  namespace cfg = hexa::config;
  const float rad2deg = 180.0f / static_cast<float>(M_PI);

  // printf varargs promote float->double regardless; cast explicitly to keep
  // -Wdouble-promotion clean (matches main.cpp's printf style).
  std::printf("Leg specs (mount xyz [m], mount yaw [deg], segments [m]):\n");
  for (int i = 0; i < hexa::kNumLegs; ++i) {
    const auto& s = cfg::kLegSpecs[static_cast<std::size_t>(i)];
    const auto name = hexa::leg_name(hexa::leg_from_index(i));
    std::printf("  %-9.*s  mount=(% .4f, % .4f, % .4f)  yaw=% 8.3f  "
                "coxa=%.3f femur=%.3f tibia=%.3f\n",
                static_cast<int>(name.size()), name.data(),
                (double)s.mount_xyz.x, (double)s.mount_xyz.y,
                (double)s.mount_xyz.z, (double)(s.mount_yaw * rad2deg),
                (double)s.coxa_len, (double)s.femur_len, (double)s.tibia_len);
  }

  std::printf("\nStanding pose: body_height=%.4f m\n",
              (double)cfg::kStandingPose.body_height);
  for (int gi = 0; gi < hexa::kNumLegGroups; ++gi) {
    const auto& grp = cfg::kStandingPose.groups[static_cast<std::size_t>(gi)];
    const auto name = hexa::LEG_GROUP_NAMES[static_cast<std::size_t>(gi)];
    std::printf("  %-6.*s  tip_reach=%.4f m  coxa=%.3f deg\n",
                static_cast<int>(name.size()), name.data(),
                (double)grp.tip_reach, (double)(grp.coxa * rad2deg));
  }
  std::printf("Solved per-leg angles (deg: coxa / femur above horizontal / "
              "tibia interior):\n");
  const auto standing = hexa::gait::standing_pose_from_config();
  for (int i = 0; i < hexa::kNumLegs; ++i) {
    const auto& a = standing[static_cast<std::size_t>(i)];
    const auto name = hexa::leg_name(hexa::leg_from_index(i));
    std::printf("  %-9.*s  coxa=% 8.3f  femur=% 8.3f  tibia=% 8.3f\n",
                static_cast<int>(name.size()), name.data(),
                (double)(a[0] * rad2deg), (double)(-a[1] * rad2deg),
                (double)((static_cast<float>(M_PI) - a[2]) * rad2deg));
  }

  // The angular cap is not a knob: it is each gait's linear cap divided by the
  // lever arm a yaw rate acts through, i.e. the outermost standing foot's
  // planar radius.
  const auto nominal = hexa::gait::nominal_stance_from_config();
  const float r_outer = hexa::gait::outer_stance_radius(nominal);
  std::printf("\nStanding stance (body frame, planar radius):\n");
  for (int i = 0; i < hexa::kNumLegs; ++i) {
    const auto name = hexa::leg_name(hexa::leg_from_index(i));
    const auto& p = nominal.at(hexa::gait::LEG_NAMES[i]);
    std::printf("  %-9.*s  x=% 8.4f  y=% 8.4f  r=%.4f\n",
                static_cast<int>(name.size()), name.data(), (double)p[0],
                (double)p[1], (double)std::hypot(p[0], p[1]));
  }
  std::printf("  outer stance radius = %.4f m (yaw lever arm)\n",
              (double)r_outer);

  const auto caps = hexa::gait::load_velocity_caps_from_config(r_outer);
  std::printf("\nGait velocity caps:\n");
  for (const auto& g : cfg::kGaits) {
    const std::string name(g.name.data());
    std::printf("  %-9s duty=%.4f  linear_max=%.4f m/s  "
                "angular_max=%.4f rad/s (%.1f deg/s)  yaw_bias=%.4f  %s\n",
                g.name.data(), (double)g.duty_factor, (double)g.linear_max,
                (double)caps.angular_max(name),
                (double)(caps.angular_max(name) * rad2deg), (double)g.yaw_bias,
                g.unstable ? "(unstable)" : "");
  }
  return 0;
}
