// Diagnostic probe (not a test): walk at the cap along +y, swing the command to
// +x quickly, and find what jumps. Full pipeline (shaper, engine, posture, IK).
//
// Per run, baseline (steady +y walk) against the window after the turn:
//   dtheta   worst joint step in one tick, and the leg / joint
//   ddtheta  worst second difference of a joint (a velocity step)
//   pose     worst body-pose step (x, y in mm; roll, pitch, yaw in mrad)
//   slip     worst world-frame motion of a planted foot in one tick (mm)
// then a CSV of the ten worst ticks in the window.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "config_generated.hpp"
#include "kinematics/body_transform.hpp"
#include "kinematics/leg_ik.hpp"
#include "pipeline.hpp"

namespace pl = hexa::pipeline;
using hexa::gait::EngineState;

namespace {

constexpr float kDt = pl::kDt;
constexpr float kContactBand = 0.0005f;

pl::TickResult tick_cmd(pl::Pipeline& p, const pl::CommandIntent& cmd,
                        std::uint64_t& now_us) {
  pl::TickInput in;
  in.now_us = now_us;
  in.bt_connected = true;
  in.last_input_us = now_us;
  in.dt = kDt;
  now_us += pl::kTickPeriodUs;
  return p.tick(cmd, in);
}

pl::CommandIntent drive(float vx, float vy) {
  pl::CommandIntent cmd;
  cmd.linear_x = vx;
  cmd.linear_y = vy;
  return cmd;
}

// Commanded feet in the engine's unposed body frame: undo the body pose.
std::array<hexa::Vec3, hexa::kNumLegs> feet(const pl::TickResult& r) {
  std::array<hexa::Vec3, hexa::kNumLegs> out{};
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    const hexa::JointAngles a = {r.theta[i * 3 + 0], r.theta[i * 3 + 1],
                                 r.theta[i * 3 + 2]};
    const hexa::Vec3 f = hexa::leg_to_body(
        hexa::forward_kinematics(a, hexa::config::kLegSpecs[i]),
        hexa::config::kLegSpecs[i]);
    const auto& b = r.body_pose;
    const float cr = std::cos(b.roll), sr = std::sin(b.roll);
    const float cp = std::cos(b.pitch), sp = std::sin(b.pitch);
    const float cy = std::cos(b.yaw), sy = std::sin(b.yaw);
    const float x1 = f[0], y1 = cr * f[1] - sr * f[2], z1 = sr * f[1] + cr * f[2];
    const float x2 = cp * x1 + sp * z1, y2 = y1, z2 = -sp * x1 + cp * z1;
    out[i] = hexa::Vec3(cy * x2 - sy * y2 + b.x, sy * x2 + cy * y2 + b.y,
                        z2 + b.z);
  }
  return out;
}

// Worst (femur + tibia) minus the femur-joint-to-foot distance, all six legs.
float reach_headroom(const pl::TickResult& r) {
  float worst = 1.0f;
  for (std::size_t i = 0; i < hexa::kNumLegs; ++i) {
    const auto& spec = hexa::config::kLegSpecs[i];
    const hexa::JointAngles a = {r.theta[i * 3 + 0], r.theta[i * 3 + 1],
                                 r.theta[i * 3 + 2]};
    const hexa::Vec3 q = hexa::forward_kinematics(a, spec);
    const float d = std::hypot(std::hypot(q.x, q.y) - spec.coxa_len, q.z);
    worst = std::min(worst, spec.femur_len + spec.tibia_len - d);
  }
  return worst;
}

struct Tick {
  int t;
  float dth, ddth, dpose, slip;
  int leg_th, leg_slip;
  float vx, vy, phase;
  bool stance[6];
  float pose[5];
};

struct Worst {
  float dth = 0, ddth = 0, dpose_mm = 0, dpose_mrad = 0, slip = 0;
};

}  // namespace

// The stick's fit for a planar deflection: the shipped one, or the old
// inf-norm (yaw zero, so its peak is the 2-norm).
std::array<float, 2> fit(float x, float y, bool old) {
  if (old) {
    const float m = std::hypot(x, y);
    const float k = m > 0.0f ? std::max(std::fabs(x), std::fabs(y)) / m : 0.0f;
    return {x * k, y * k};
  }
  const auto f = hexa::teleop::fit_drive_to_envelope(x, y, 0.0f);
  return {f[0], f[1]};
}

pl::Pipeline& standing(pl::Pipeline& p, std::uint64_t& now) {
  pl::CommandIntent init;
  init.init_request = true;
  tick_cmd(p, init, now);
  for (int i = 0; i < 4000 && p.engine().state() != EngineState::STAND; ++i)
    tick_cmd(p, drive(0, 0), now);
  return p;
}

constexpr float kCap = 0.131f;  // default preset forward cap, measured

// Steady shaped speed per stick heading, stick on a round gate's rim.
void sweep() {
  std::printf("steady speed per heading (m/s), old fit vs new fit\n");
  for (int deg = 0; deg <= 90; deg += 15) {
    float got[2];
    for (int old = 0; old < 2; ++old) {
      pl::Pipeline p;
      std::uint64_t now = 0;
      standing(p, now);
      const float a = float(deg) * float(M_PI) / 180.0f;
      const auto f = fit(std::cos(a), std::sin(a), old == 0);
      for (int i = 0; i < 600; ++i) tick_cmd(p, drive(f[0] * kCap, f[1] * kCap), now);
      const auto [vx, vy, wz] = p.control().limiter().state();
      (void)wz;
      got[old] = std::hypot(vx, vy);
    }
    std::printf("  %2d deg: old %.3f  new %.3f\n", deg, got[0], got[1]);
  }
}

// The stick swept along its rim from +y to +x over `ramp` ticks; shaped speed
// sampled from the start of the sweep until it has settled.
void rim_turn(int ramp) {
  for (int old = 1; old >= 0; --old) {
    pl::Pipeline p;
    std::uint64_t now = 0;
    standing(p, now);
    float lo = 1e9f, start = 0, end = 0;
    int t_lo = 0;
    for (int t = -600; t < 300; ++t) {
      const float s = t < 0 ? 0.0f : std::min(1.0f, float(t + 1) / float(ramp));
      const float a = 0.5f * float(M_PI) * (1.0f - s);
      const auto f = fit(std::cos(a), std::sin(a), old == 1);
      tick_cmd(p, drive(f[0] * kCap, f[1] * kCap), now);
      const auto [vx, vy, wz] = p.control().limiter().state();
      (void)wz;
      const float v = std::hypot(vx, vy);
      if (t == -1) start = v;
      if (t >= 0 && v < lo) { lo = v; t_lo = t; }
      end = v;
    }
    std::printf("  rim y->x over %3d ms, %s fit: %.3f -> min %.3f (at %3d ms) -> %.3f m/s\n",
                ramp * 5, old ? "old" : "new", start, lo, t_lo * 5, end);
  }
}


// The ground speed the planted feet actually lay down, against the shaped
// command, through a fast rim sweep. A foot counts while it has been on the
// ground plane for three ticks; its body-frame velocity is -v_body.
struct TurnStats {
  float feet_min = 1e9f;
  int braked = 0;
  float headroom = 1.0f;
  int unreachable = 0;
  float spacing = 1.0f;  // closest pair of feet, xy
};

// The ground speed the planted feet lay down against the shaped command, through
// a stick sweep from heading `from` to `to`. A foot counts once it has been on
// the ground plane for three ticks; its body-frame velocity is -v_body. A tick is
// braked when one counted foot is off the command by more than 1 cm/s; single
// ticks are the touchdown artefact of the three-tick filter and are dropped.
TurnStats foot_speed(int ramp, int offset, float from, float to) {
  pl::Pipeline p;
  std::uint64_t now = 0;
  standing(p, now);
  std::array<hexa::Vec3, 6> prev{};
  std::array<int, 6> low{};
  float ground = 0;
  {
    const auto r = tick_cmd(p, drive(0, 0), now);
    for (const auto& f : feet(r)) ground = std::min(ground, f.z);
  }
  TurnStats st;
  int run = 0;
  for (int t = -400 - offset; t < 400; ++t) {
    const float s = t < 0 ? 0.0f : std::min(1.0f, float(t + 1) / float(ramp));
    const float a = from + (to - from) * s;
    const auto f = fit(std::cos(a), std::sin(a), false);
    const auto r = tick_cmd(p, drive(f[0] * kCap, f[1] * kCap), now);
    const auto [vx, vy, wz] = p.control().limiter().state();
    (void)wz;
    const auto ft = feet(r);
    float sx = 0, sy = 0, worst = 0;
    int n = 0;
    for (int i = 0; i < 6; ++i) {
      low[i] = ft[i].z <= ground + 0.0005f ? low[i] + 1 : 0;
      if (low[i] >= 3) {
        const float ux = -(ft[i].x - prev[i].x) / kDt;
        const float uy = -(ft[i].y - prev[i].y) / kDt;
        sx += ux; sy += uy; ++n;
        worst = std::max(worst, std::hypot(ux - vx, uy - vy));
      }
      prev[i] = ft[i];
    }
    if (t < 0) continue;
    st.headroom = std::min(st.headroom, reach_headroom(r));
    st.unreachable += r.unreachable;
    for (int i = 0; i < 6; ++i)
      for (int j = i + 1; j < 6; ++j)
        st.spacing = std::min(st.spacing, std::hypot(ft[i].x - ft[j].x, ft[i].y - ft[j].y));
    run = worst > 0.01f ? run + 1 : 0;
    if (run >= 2) {
      st.braked += run == 2 ? 2 : 1;
      st.feet_min = std::min(st.feet_min, n ? std::hypot(sx / n, sy / n) : 0.0f);
    }
  }
  return st;
}

void turns(const char* name, float from, float to) {
  TurnStats w;
  w.feet_min = 1e9f;
  for (int ramp : {1, 40}) {
    for (int off = 0; off < 160; off += 20) {
      const TurnStats s = foot_speed(ramp, off, from, to);
      w.feet_min = std::min(w.feet_min, s.feet_min);
      w.braked = std::max(w.braked, s.braked);
      w.headroom = std::min(w.headroom, s.headroom);
      w.unreachable += s.unreachable;
      w.spacing = std::min(w.spacing, s.spacing);
    }
  }
  std::printf("  %-14s worst braked %3d ms, feet min %s m/s, headroom %5.1f mm, unreachable ticks %d, foot spacing %5.1f mm\n",
              name, w.braked * 5,
              w.feet_min > 1e8f ? "  -  " : (std::to_string(w.feet_min).substr(0, 5)).c_str(),
              w.headroom * 1e3f, w.unreachable, w.spacing * 1e3f);
}

int main() {
  std::printf("grace %.2f\n", hexa::gait::kStanceExcursionGrace);
  const float q = 0.5f * float(M_PI);
  turns("y->x", q, 0.0f);
  turns("x->y", 0.0f, q);
  turns("x->-y", 0.0f, -q);
  turns("straight x", 0.0f, 0.0f);
  turns("straight y", q, q);
  return 0;
}
