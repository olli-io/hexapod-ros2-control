// Diagnostic probe (not a test): sweep the quadruped support-shift knobs and
// report, per configuration and over eight headings, the four numbers the
// tuning trades against each other —
//
//   margin    worst static margin, body origin to the support polygon's edge
//   headroom  worst leg reach left, femur joint to foot against femur + tibia
//   body      peak body speed and cornering acceleration, which is what
//             "smoother" means numerically
//
// The two tripwires in test_pipeline.cpp
// (Quadruped.CreepKeepsTheBodyInsideTheSupportTriangle and
// Quadruped.CreepNeverAsksForAnUnreachableFoot) only say pass/fail at the
// shipped values; this is where the numbers behind them come from. The grid in
// main() is meant to be edited — re-measure rather than reason about it.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "kinematics/body_transform.hpp"
#include "kinematics/leg_ik.hpp"
#include "pipeline.hpp"

namespace pl = hexa::pipeline;
using hexa::gait::EngineState;

namespace {

constexpr float kContactBand = 0.005f;
constexpr float kDt = pl::kDt;

pl::TickResult tick_cmd(pl::Pipeline& p, const pl::CommandIntent& cmd,
                        std::uint64_t& now_us) {
  pl::TickInput in;
  in.now_us = now_us;
  in.bt_connected = true;
  in.last_input_us = now_us;
  in.dt = pl::kDt;
  now_us += pl::kTickPeriodUs;
  return p.tick(cmd, in);
}

pl::CommandIntent drive(float vx, float vy) {
  pl::CommandIntent cmd;
  cmd.linear_x = vx;
  cmd.linear_y = vy;
  return cmd;
}

std::array<hexa::Vec3, hexa::kNumLegs> feet_from_theta(
    const pl::TickResult& r) {
  std::array<hexa::Vec3, hexa::kNumLegs> out{};
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    const hexa::JointAngles a = {r.theta[i * 3 + 0], r.theta[i * 3 + 1],
                                 r.theta[i * 3 + 2]};
    out[i] = hexa::leg_to_body(
        hexa::forward_kinematics(a, hexa::config::kLegSpecs[i]),
        hexa::config::kLegSpecs[i]);
  }
  return out;
}

float support_margin(const std::array<hexa::Vec3, hexa::kNumLegs>& feet) {
  float lowest = feet[0].z;
  for (const auto& f : feet) {
    lowest = std::min(lowest, f.z);
  }
  static constexpr std::array<std::size_t, 6> kRing = {0, 3, 4, 5, 2, 1};
  std::vector<std::pair<float, float>> hull;
  for (const std::size_t i : kRing) {
    if (feet[i].z <= lowest + kContactBand) {
      hull.push_back({feet[i].x, feet[i].y});
    }
  }
  if (hull.size() < 3) {
    return -1.0f;
  }
  float margin = 1.0f;
  float sign = 0.0f;
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const auto& a = hull[i];
    const auto& b = hull[(i + 1) % hull.size()];
    const float ex = b.first - a.first;
    const float ey = b.second - a.second;
    const float len = std::hypot(ex, ey);
    if (len <= 0.0f) {
      continue;
    }
    const float d = (ex * (0.0f - a.second) - ey * (0.0f - a.first)) / len;
    if (sign == 0.0f) {
      sign = d < 0.0f ? -1.0f : 1.0f;
    }
    margin = std::min(margin, sign * d);
  }
  return margin;
}

// (femur + tibia) minus the femur-joint-to-foot distance the tick asked for.
// The corners only: a parked middle leg is held at fixed angles.
float reach_headroom(const pl::TickResult& r) {
  float worst = 1.0f;
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    if (i == 1 || i == 4) {
      continue;  // l_middle / r_middle are parked
    }
    const auto& spec = hexa::config::kLegSpecs[i];
    const hexa::JointAngles a = {r.theta[i * 3 + 0], r.theta[i * 3 + 1],
                                 r.theta[i * 3 + 2]};
    const hexa::Vec3 p = hexa::forward_kinematics(a, spec);
    const float d =
        std::hypot(std::hypot(p.x, p.y) - spec.coxa_len, p.z);
    worst = std::min(worst, spec.femur_len + spec.tibia_len - d);
  }
  return worst;
}

// Stand up on four legs from the belly: the gait carries the leg set, the init
// that rides with it climbs the ladder for that set.
bool enter_quadruped(pl::Pipeline& p, std::uint64_t& now_us) {
  pl::CommandIntent select;
  select.has_gait_select = true;
  select.gait_select = "quadruped_wave";
  select.init_request = true;
  select.init_quadruped = true;
  tick_cmd(p, select, now_us);
  for (int i = 0; i < 4000; ++i) {
    const pl::TickResult r = tick_cmd(p, drive(0.0f, 0.0f), now_us);
    if (r.engine_state == EngineState::STAND &&
        p.engine().leg_set() == hexa::gait::LegSet::QUADRUPED) {
      return true;
    }
  }
  return false;
}

struct Result {
  float margin_mm = 0.0f;
  float headroom_mm = 0.0f;
  // Body-pose rate and acceleration, recovered from a corner foot that has been
  // grounded for a while: its commanded body-frame velocity is -(v_command +
  // pose_rate), and the stance target itself runs at a constant velocity, so the
  // second difference is the body's own acceleration. This is what "smoother"
  // means numerically — a corner in the support-shift path is an acceleration
  // spike, and the peak speed is how fast the body lunges.
  float speed_mm_s = 0.0f;
  float accel_mm_s2 = 0.0f;
  int unreachable = 0;
  int worst_margin_heading = -1;
};

Result sweep(const pl::PipelineConfig& cfg) {
  Result out;
  out.margin_mm = 1000.0f;
  out.headroom_mm = 1000.0f;
  const float swing_end = hexa::gait::swing_end_phase(
      3.0f / 4.0f, cfg.engine.quadruped_swing_phase_margin);
  const float speed = cfg.engine.stride_length * swing_end /
                      (cfg.engine.min_swing_time * (1.0f - swing_end));

  static constexpr std::array<std::size_t, 4> kCorners = {0, 2, 3, 5};
  for (int h = 0; h < 8; ++h) {
    const float theta = 2.0f * 3.14159265f * static_cast<float>(h) / 8.0f;
    const float vx = speed * std::cos(theta);
    const float vy = speed * std::sin(theta);
    pl::Pipeline p(cfg);
    std::uint64_t now_us = 0;
    if (!enter_quadruped(p, now_us)) {
      std::printf("  heading %d: never parked\n", h);
      continue;
    }
    std::array<hexa::Vec3, 4> prev{};
    std::array<hexa::Vec3, 4> prev2{};
    std::array<int, 4> grounded{};
    int walked = 0;
    for (int i = 0; i < 3000; ++i) {
      const pl::TickResult r = tick_cmd(p, drive(vx, vy), now_us);
      if (r.engine_state != EngineState::GAIT) {
        continue;
      }
      ++walked;
      const auto feet = feet_from_theta(r);
      const float m = support_margin(feet);
      if (m < out.margin_mm) {
        out.margin_mm = m;
        out.worst_margin_heading = h;
      }
      out.headroom_mm = std::min(out.headroom_mm, reach_headroom(r));
      out.unreachable += r.unreachable;

      float lowest = feet[0].z;
      for (const auto& f : feet) {
        lowest = std::min(lowest, f.z);
      }
      for (std::size_t k = 0; k < kCorners.size(); ++k) {
        const hexa::Vec3& f = feet[kCorners[k]];
        const bool down = f.z <= lowest + kContactBand;
        // Only well inside a stance: the seam ticks carry the swing's own
        // velocity, which is not the body's.
        if (down && grounded[k] >= 4 && walked > 400) {
          const float ax = (f.x - 2.0f * prev[k].x + prev2[k].x) / (kDt * kDt);
          const float ay = (f.y - 2.0f * prev[k].y + prev2[k].y) / (kDt * kDt);
          out.accel_mm_s2 = std::max(out.accel_mm_s2, std::hypot(ax, ay));
          const float sx = (f.x - prev[k].x) / kDt + vx;
          const float sy = (f.y - prev[k].y) / kDt + vy;
          out.speed_mm_s = std::max(out.speed_mm_s, std::hypot(sx, sy));
        }
        grounded[k] = down ? grounded[k] + 1 : 0;
        prev2[k] = prev[k];
        prev[k] = f;
      }
    }
  }
  out.margin_mm *= 1000.0f;
  out.headroom_mm *= 1000.0f;
  out.speed_mm_s *= 1000.0f;
  out.accel_mm_s2 *= 1000.0f;
  return out;
}

void report(const char* label, const pl::PipelineConfig& cfg) {
  const Result r = sweep(cfg);
  std::printf(
      "%-42s margin %7.2f mm (h%d)  headroom %6.2f mm  body %6.1f mm/s "
      "%8.1f mm/s^2  unreachable %d\n",
      label, static_cast<double>(r.margin_mm), r.worst_margin_heading,
      static_cast<double>(r.headroom_mm), static_cast<double>(r.speed_mm_s),
      static_cast<double>(r.accel_mm_s2), r.unreachable);
}

// ── The ladders on either side of the walk ──
//
// sweep() only reads GAIT ticks. The engagement, the settle and the reseat carry
// the same body over the same four feet and are where the margin is thinnest, so
// this walks one heading start to finish and reports the worst tick per state.
// `verbose` dumps every tick under a millimetre of margin, which is how you find
// out *which* lift-off is the bad one.

const char* state_word(EngineState s) {
  switch (s) {
    case EngineState::STAND: return "STAND";
    case EngineState::ENGAGING: return "ENGAGING";
    case EngineState::GAIT: return "GAIT";
    case EngineState::SETTLING: return "SETTLING";
    case EngineState::RESEATING: return "RESEATING";
    default: return "other";
  }
}

int grounded_corners(const std::array<hexa::Vec3, hexa::kNumLegs>& feet) {
  float lowest = feet[0].z;
  for (const auto& f : feet) {
    lowest = std::min(lowest, f.z);
  }
  int n = 0;
  for (const std::size_t i : {std::size_t{0}, std::size_t{2}, std::size_t{3},
                              std::size_t{5}}) {
    if (feet[i].z <= lowest + kContactBand) {
      ++n;
    }
  }
  return n;
}

void ladders(const pl::PipelineConfig& cfg, bool verbose,
             int trace_heading = 0, float trace_below = 0.001f) {
  const float swing_end = hexa::gait::swing_end_phase(
      3.0f / 4.0f, cfg.engine.quadruped_swing_phase_margin);
  const float speed = cfg.engine.stride_length * swing_end /
                      (cfg.engine.min_swing_time * (1.0f - swing_end));

  auto slot = [](EngineState s) {
    switch (s) {
      case EngineState::ENGAGING: return 1;
      case EngineState::GAIT: return 2;
      case EngineState::SETTLING: return 3;
      case EngineState::RESEATING: return 4;
      default: return 0;
    }
  };
  // Where in the cycle the stick is let go. A settle inherits whatever phase the
  // walk was at, so one release point says almost nothing.
  static constexpr int kReleases = 8;
  const int cycle_ticks = static_cast<int>(
      cfg.engine.min_swing_time / swing_end / kDt);

  for (int h = 0; h < 8; ++h) {
    const float theta = 2.0f * 3.14159265f * static_cast<float>(h) / 8.0f;
    const float vx = speed * std::cos(theta);
    const float vy = speed * std::sin(theta);

    float worst[6] = {1, 1, 1, 1, 1, 1};
    int fewest[6] = {4, 4, 4, 4, 4, 4};
    // The first kReleases runs walk and stop; the rest abandon the engagement
    // part-way, which is the route that always ends on the reseat ladder.
    for (int rel = 0; rel < 2 * kReleases; ++rel) {
    pl::Pipeline p(cfg);
    std::uint64_t now_us = 0;
    if (!enter_quadruped(p, now_us)) {
      std::printf("  heading %d: never parked\n", h);
      continue;
    }

    // Engage, walk three cycles plus a slice of a fourth, then let go and stop.
    const int drive_ticks =
        rel < kReleases
            ? 3 * cycle_ticks + 900 + rel * cycle_ticks / kReleases
            : (rel - kReleases + 1) * (cycle_ticks + 240) / kReleases;
    for (int i = 0; i < drive_ticks + 4000; ++i) {
      const bool driving = i < drive_ticks;
      const pl::TickResult r =
          tick_cmd(p, driving ? drive(vx, vy) : drive(0.0f, 0.0f), now_us);
      const auto feet = feet_from_theta(r);
      const float m = support_margin(feet);
      const int k = slot(r.engine_state);
      worst[k] = std::min(worst[k], m);
      fewest[k] = std::min(fewest[k], grounded_corners(feet));
      if (verbose && h == trace_heading && m < trace_below) {
        float lowest = feet[0].z;
        for (const auto& f : feet) {
          lowest = std::min(lowest, f.z);
        }
        float cx = 0.0f, cy = 0.0f;
        int n = 0;
        for (const std::size_t j : {std::size_t{0}, std::size_t{2},
                                    std::size_t{3}, std::size_t{5}}) {
          if (feet[j].z <= lowest + kContactBand) {
            cx += feet[j].x;
            cy += feet[j].y;
            ++n;
          }
        }
        std::printf(
            "    t%5d %-9s margin %7.2f mm  down %d  down-centroid "
            "(%6.1f,%6.1f) mm\n",
            i, state_word(r.engine_state), static_cast<double>(m * 1000.0f), n,
            static_cast<double>(cx / std::max(n, 1) * 1000.0f),
            static_cast<double>(cy / std::max(n, 1) * 1000.0f));
      }
      if (!driving && r.engine_state == EngineState::STAND) {
        break;
      }
    }
    }
    std::printf(
        "  heading %d: engage %6.2f mm (down %d)  gait %6.2f mm  "
        "settle %6.2f mm (down %d)  reseat %6.2f mm (down %d)\n",
        h, static_cast<double>(worst[1] * 1000.0f), fewest[1],
        static_cast<double>(worst[2] * 1000.0f),
        static_cast<double>(worst[3] * 1000.0f), fewest[3],
        static_cast<double>(worst[4] * 1000.0f), fewest[4]);
  }
}

}  // namespace

int main() {
  const pl::PipelineConfig baked = pl::PipelineConfig::baked();
  std::printf("shipped: gain %.2f lead %.2f tau %.2f quad margin %.2f\n",
              static_cast<double>(baked.posture.support_shift_gain),
              static_cast<double>(baked.posture.support_shift_lead),
              static_cast<double>(baked.posture.support_shift_tau),
              static_cast<double>(baked.engine.quadruped_swing_phase_margin));
  report("baked", baked);

  char label[128];

  std::printf("\nladders (shift_time %.2f s):\n",
              static_cast<double>(baked.engine.quadruped_shift_time));
  ladders(baked, false);
  for (const float settle : {0.45f, 0.6f, 0.8f}) {
    pl::PipelineConfig cfg = baked;
    cfg.engine.settle_swing_time = settle;
    std::printf("settle_swing %.2f:\n", static_cast<double>(settle));
    ladders(cfg, false);
  }
  return 0;
  // The values this replaced, on the six-leg margin: the "before" row.
  {
    pl::PipelineConfig cfg = baked;
    cfg.engine.quadruped_swing_phase_margin = 0.12f;
    cfg.posture.support_shift_gain = 0.60f;
    cfg.posture.support_shift_lead = 0.05f;
    cfg.posture.support_shift_tau = 0.04f;
    report("BEFORE margin 0.12 gain 0.60 lead 0.05 tau 0.04", cfg);
  }

  static constexpr float kGains[] = {0.40f, 0.45f, 0.50f};
  static constexpr float kLeads[] = {0.20f, 0.28f, 0.35f};
  static constexpr float kTaus[] = {0.10f, 0.12f, 0.14f, 0.16f, 0.18f};

  for (const float gain : kGains) {
    for (const float lead : kLeads) {
      for (const float tau : kTaus) {
        pl::PipelineConfig cfg = baked;
        cfg.posture.support_shift_gain = gain;
        cfg.posture.support_shift_lead = lead;
        cfg.posture.support_shift_tau = tau;
        std::snprintf(label, sizeof(label), "gain %.2f lead %.2f tau %.2f",
                      static_cast<double>(gain), static_cast<double>(lead),
                      static_cast<double>(tau));
        report(label, cfg);
      }
    }
  }
  return 0;
}
