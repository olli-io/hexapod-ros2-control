// Prints the generated config for eyeball verification (plan part 04).
//
// Chiefly the symmetry-expanded leg specs — the plan's part-04 acceptance step
// is "print all 6 LegSpecs, confirm r_rear = (-0.083, -0.0575, -150deg)". Also
// dumps the derived per-gait velocity caps. Not a test; a human-readable dump.

#include <array>
#include <cmath>
#include <cstdio>
#include <string_view>

#include "config_generated.hpp"
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

  std::printf("\nStanding pose (rad): coxa=%.4f femur=%.4f tibia=%.4f\n",
              (double)cfg::kStandingPose[0], (double)cfg::kStandingPose[1],
              (double)cfg::kStandingPose[2]);

  std::printf("\nGait velocity caps (angular_max=%.2f rad/s):\n",
              (double)cfg::kAngularMax);
  for (const auto& g : cfg::kGaits) {
    std::printf("  %-9s duty=%.4f  linear_max=%.4f m/s  yaw_bias=%.4f  %s\n",
                g.name.data(), (double)g.duty_factor, (double)g.linear_max,
                (double)g.yaw_bias, g.unstable ? "(unstable)" : "");
  }
  return 0;
}
