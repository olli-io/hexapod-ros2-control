// Diagnostic probe (not a test): walk along +x, reverse the command through the
// reversal ladder, and log what the planted feet do after the mirror fires.
// One CSV row per tick around the mirror, then a summary per preset:
//
//   crossing   ticks after the mirror with |v| below the knee, and how far the
//              master advanced across them while the body netted no travel
//   overshoot  per leg, worst |excursion_x| - band after the mirror; positive
//              is into the grace zone, beyond 0.25 * band the anchor is pinned
//   drag       per leg, world-frame metres a planted foot travelled from the
//              mirror to its next lift-off
//   landing    excursion_x at touchdown for legs landing after the mirror
//
// Drives the engine directly with the shaper's slew reduced to one axis, as
// test_reversal.cpp does. Output is analysed off-line; nothing here asserts.
#include <cmath>
#include <cstdio>
#include <map>
#include <string>

#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "gait/gaits/registry.hpp"

namespace g = hexa::gait;

namespace {

constexpr float kDt = 0.005f;

float slew_toward(float v, float target, float accel, float dt) {
  const float step = accel * dt;
  const float delta = target - v;
  return std::fabs(delta) <= step ? target : v + std::copysign(step, delta);
}

float wrap_half(float d) {
  if (d > 0.5f) return d - 1.0f;
  if (d < -0.5f) return d + 1.0f;
  return d;
}

struct LegAcc {
  hexa::Vec3 world = hexa::Vec3::Zero();
  bool have = false;
  bool swing = false;
  float drag = 0.0f;
  float worst_overshoot = -1.0f;
  float drag_banked = 0.0f;
  float landing_ex = 0.0f;
  bool landed = false;
};

void run(const std::string& preset_id, std::size_t preset_idx) {
  const auto& p = hexa::config::kPresets[preset_idx];
  const auto cfg = g::engine_config_from_config();
  const std::string gait = "tripod";
  auto e = g::make_default_engine(gait);
  if (!e->request_preset(preset_id)) {
    std::fprintf(stderr, "preset %s refused\n", preset_id.c_str());
    return;
  }
  const float duty = g::strategies().at(gait)()->duty_factor();
  const float swing_end = g::swing_end_phase(duty, cfg.swing_phase_margin);
  const float stance_fraction = 1.0f - swing_end;
  const float stride = g::effective_stride_length(
      g::build_leg_contexts_from_config(), {1.0f, 0.0f}, 0.0f,
      p.stride_length, p.stride_length_radial);
  const float speed =
      stride * swing_end / (p.min_swing_time * stance_fraction);
  const float knee = stride * swing_end / (p.max_swing_time * stance_fraction);
  const float accel = speed / hexa::config::kControl.vmax_ramp_time_linear;
  const float band = 0.5f * stride;
  const float max_cycle = p.max_swing_time / swing_end;

  for (int i = 0; i < 6000 && e->state() != g::EngineState::STAND; ++i) {
    e->update(kDt, {0.0f, 0.0f}, 0.0f);
    e->start_initialize();
  }
  std::map<std::string, hexa::Vec3> nominal;
  for (const auto& [n, leg] : e->update(kDt, {0.0f, 0.0f}, 0.0f)) {
    nominal[n] = leg.foot_target;
  }

  float v = 0.0f;
  hexa::Vec3 body_w = hexa::Vec3::Zero();
  bool holding = false;
  std::map<std::string, g::LegOutput> out;
  const auto tick = [&](float target) {
    const auto [rx, ry, rw] = e->shape_reversal(kDt, {target, 0.0f}, 0.0f);
    (void)ry;
    (void)rw;
    holding = rx != target;
    v = slew_toward(v, rx, accel, kDt);
    body_w.x += v * kDt;
    out = e->update(kDt, {v, 0.0f}, 0.0f);
  };

  for (int i = 0; i < 6000 && e->state() != g::EngineState::GAIT; ++i) {
    tick(speed);
  }
  for (int i = 0; i < 600; ++i) tick(speed);

  std::printf("# preset=%s stride=%.4f band=%.4f grace=%.4f knee=%.4f speed=%.4f "
              "accel=%.3f max_cycle=%.3f predicted_overshoot=%.4f\n",
              preset_id.c_str(), stride, band, 0.25f * band, knee, speed, accel,
              max_cycle, knee * knee / (2.0f * accel));
  std::printf("preset,i,state,v,master,mirror,hold");
  for (const auto& sv : hexa::LEG_NAMES) {
    const std::string n(sv);
    std::printf(",%s_st,%s_ph,%s_ex,%s_dw", n.c_str(), n.c_str(), n.c_str(),
                n.c_str());
  }
  std::printf("\n");

  std::map<std::string, LegAcc> acc;
  float prev_master = e->master_phase();
  int mirror_tick = -1;
  int crossing_ticks = 0;
  float crossing_master = 0.0f;
  bool crossing_done = false;
  float hold_ticks = 0;

  for (int i = 0; i < 1400; ++i) {
    tick(-speed);
    const float dm = wrap_half(e->master_phase() - prev_master);
    prev_master = e->master_phase();
    const bool mirror = e->state() == g::EngineState::GAIT && std::fabs(dm) > 0.1f;
    if (mirror && mirror_tick < 0) mirror_tick = i;
    if (holding) hold_ticks += 1;
    const bool after = mirror_tick >= 0;
    if (after && !mirror && !crossing_done) {
      if (std::fabs(v) < knee) {
        ++crossing_ticks;
        crossing_master += dm;
      } else if (crossing_ticks > 0) {
        crossing_done = true;
      }
    }

    const bool log = i >= 0 && (mirror_tick < 0 ? i > 1400 - 1 : i <= mirror_tick + 500);
    if (log || after) {
      std::printf("%s,%d,%d,%.4f,%.4f,%d,%d", preset_id.c_str(), i,
                  static_cast<int>(e->state()), v, e->master_phase(),
                  mirror ? 1 : 0, holding ? 1 : 0);
    }
    for (const auto& sv : hexa::LEG_NAMES) {
      const std::string n(sv);
      const g::LegOutput& leg = out.at(n);
      LegAcc& a = acc[n];
      const hexa::Vec3 world(leg.foot_target.x + body_w.x,
                             leg.foot_target.y + body_w.y, leg.foot_target.z);
      const float ex = leg.foot_target.x - nominal.at(n).x;
      float dw = 0.0f;
      if (leg.stance && a.have && !a.swing) {
        dw = std::hypot(world.x - a.world.x, world.y - a.world.y);
      }
      if (after) {
        if (leg.stance) {
          a.drag += dw;
          a.worst_overshoot = std::max(a.worst_overshoot, std::fabs(ex) - band);
        }
        if (!leg.stance && a.have && !a.swing) {  // lift-off: bank the stance
          if (a.drag_banked == 0.0f) a.drag_banked = a.drag;
          a.drag = 0.0f;
        }
        if (leg.stance && a.have && a.swing && !a.landed) {  // touchdown
          a.landing_ex = ex;
          a.landed = true;
        }
      }
      a.world = world;
      a.swing = !leg.stance;
      a.have = true;
      if (log || after) {
        std::printf(",%d,%.3f,%.4f,%.5f", leg.stance ? 1 : 0, leg.phase, ex, dw);
      }
    }
    if (log || after) std::printf("\n");
    if (after && i > mirror_tick + 500) break;
  }

  std::printf("# summary preset=%s mirror_tick=%d hold_ticks=%.0f crossing_ticks=%d "
              "crossing_time=%.3f crossing_master=%.3f\n",
              preset_id.c_str(), mirror_tick, hold_ticks, crossing_ticks,
              crossing_ticks * kDt, crossing_master);
  for (const auto& sv : hexa::LEG_NAMES) {
    const std::string n(sv);
    const LegAcc& a = acc[n];
    std::printf("# leg=%s overshoot=%.4f (%.0f%% of grace) drag_first_stance=%.4f "
                "landing_ex=%.4f\n",
                n.c_str(), a.worst_overshoot,
                100.0f * a.worst_overshoot / (0.25f * band),
                a.drag_banked > 0.0f ? a.drag_banked : a.drag, a.landing_ex);
  }
}

}  // namespace

int main() {
  for (std::size_t i = 0; i < hexa::config::kPresets.size(); ++i) {
    const std::string id(hexa::config::kPresets[i].id);
    if (id == "normal" || id == "fast") run(id, i);
  }
  return 0;
}
