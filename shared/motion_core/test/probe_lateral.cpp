// Diagnostic probe (not a test): drive engine + posture + compose/IK with a
// body-frame velocity command and report, per leg, the commanded joint-space
// velocities (vs the sim-enforced servo limit) and the world-frame approach
// profile near touchdown. Compares direction scenarios to isolate why a stride
// radial to a leg's coxa drags at touchdown.
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "config_generated.hpp"
#include "gait/engine.hpp"
#include "kinematics/body_transform.hpp"
#include "kinematics/leg_ik.hpp"
#include "posture/posture.hpp"

using hexa::Vec3;
using namespace hexa::gait;

namespace {

// From geometry.yaml joints (ds3235sg, sim-enforced): rad/s.
constexpr float kJointVelLimit = 5.25f;

struct Rec {
  float t;
  bool stance;
  Vec3 target;
  Vec3 world;
  bool ik_ok;
  hexa::JointAngles theta;
  EngineState state;
};

const hexa::config::LegSpec& spec_for(int i) {
  return hexa::config::kLegSpecs[static_cast<std::size_t>(i)];
}

void run_scenario(float vx_frac, float vy_frac, const char* label) {
  const std::string gait_name(hexa::config::kDefaultGait);
  auto engine = make_default_engine(gait_name);
  hexa::posture::PostureController posture;
  const float dt = hexa::config::kEngine.controller_dt;

  float vmax = 0.0f;
  for (const auto& g : hexa::config::kGaits) {
    if (std::string(g.name.data()) == gait_name) vmax = g.linear_max;
  }
  // The cap is derated by the same ratio the stride is, or the command outruns
  // the stride the gait can lay down and the feet scrub for the excess. This
  // probe calls engine->update directly, so Control::shape is not in the loop
  // to do it. A disabled budget leaves the ratio at exactly 1.
  const float dir = std::hypot(vx_frac, vy_frac);
  float ratio = 1.0f;
  float eff = hexa::config::kEngine.stride_length;
  if (dir > 0.0f) {
    eff = effective_stride_length(
        build_leg_contexts_from_config(), {vx_frac / dir, vy_frac / dir}, 0.0f,
        hexa::config::kEngine.stride_length,
        hexa::config::kEngine.stride_length_radial);
    ratio = eff / hexa::config::kEngine.stride_length;
  }
  const float vx = vx_frac * vmax * ratio;
  const float vy = vy_frac * vmax * ratio;
  std::printf(
      "\n#### %s: cmd=(%.3f, %.3f) m/s, gait=%s, stride=%.4f m (%.0f%% cap)\n",
      label, vx, vy, gait_name.c_str(), static_cast<double>(eff),
      static_cast<double>(ratio * 100.0f));

  const auto nominal = nominal_stance_from_config();

  float t = 0.0f;
  std::map<std::string, LegOutput> out;
  engine->start_initialize();
  for (int i = 0; i < 40000 && engine->state() != EngineState::STAND; ++i) {
    out = engine->update(dt, {0.0f, 0.0f}, 0.0f);
    posture.update(out, engine->master_phase(), false, engine->state(),
                   engine->strategy_name(), dt, t);
    t += dt;
  }

  const int kTicks = 4000;
  std::map<std::string, std::vector<Rec>> trace;
  std::map<std::string, Vec3> prev_targets;
  std::map<std::string, bool> prev_stance;
  Vec3 body_w = Vec3::Zero();
  std::map<std::string, int> ik_fail;

  for (int i = 0; i < kTicks; ++i) {
    const auto st = engine->state();
    out = engine->update(dt, {vx, vy}, 0.0f);
    const hexa::posture::BodyPose pose =
        posture.update(out, engine->master_phase(), true, st,
                       engine->strategy_name(), dt, t);
    t += dt;

    if (!prev_targets.empty()) {
      Vec3 d = Vec3::Zero();
      int n = 0;
      for (const auto& name : LEG_NAMES) {
        if (out.at(name).stance && prev_stance[name]) {
          d = Vec3(d[0] + prev_targets[name][0] - out.at(name).foot_target[0],
                   d[1] + prev_targets[name][1] - out.at(name).foot_target[1],
                   d[2] + prev_targets[name][2] - out.at(name).foot_target[2]);
          ++n;
        }
      }
      if (n > 0) {
        body_w = Vec3(body_w[0] + d[0] / n, body_w[1] + d[1] / n,
                      body_w[2] + d[2] / n);
      }
    }

    for (int li = 0; li < 6; ++li) {
      const std::string name = LEG_NAMES[li];
      const LegOutput& lo = out.at(name);
      Rec r;
      r.t = t;
      r.stance = lo.stance;
      r.state = st;
      r.target = lo.foot_target;
      r.world = Vec3(lo.foot_target[0] + body_w[0],
                     lo.foot_target[1] + body_w[1],
                     lo.foot_target[2] + body_w[2]);
      r.ik_ok = true;
      const Vec3 posed = hexa::apply_body_pose(lo.foot_target, pose);
      const Vec3 in_leg = hexa::body_to_leg(posed, spec_for(li));
      try {
        r.theta = hexa::inverse_kinematics(in_leg, spec_for(li));
      } catch (const hexa::UnreachableTarget&) {
        r.ik_ok = false;
        ++ik_fail[name];
      }
      trace[name].push_back(r);
      prev_targets[name] = lo.foot_target;
      prev_stance[name] = lo.stance;
    }
  }

  // ── joint-velocity demand, steady-state half, split swing/stance ──
  std::printf("%-9s %6s | swing max dθ/dt (c/f/t) rad/s | stance max | %%swing>limit\n",
              "leg", "ikfail");
  for (const auto& name : LEG_NAMES) {
    const auto& tr = trace[name];
    float sw[3] = {0, 0, 0}, st[3] = {0, 0, 0};
    int over = 0, swing_ticks = 0;
    for (std::size_t k = tr.size() / 2 + 1; k < tr.size(); ++k) {
      if (!tr[k].ik_ok || !tr[k - 1].ik_ok) continue;
      bool tick_over = false;
      for (int j = 0; j < 3; ++j) {
        const float w = std::fabs(tr[k].theta[j] - tr[k - 1].theta[j]) / dt;
        // skip seam ticks (stance flag flips) to avoid teleport artefacts
        if (tr[k].stance != tr[k - 1].stance) continue;
        if (tr[k].stance) {
          st[j] = std::max(st[j], w);
        } else {
          sw[j] = std::max(sw[j], w);
          if (w > kJointVelLimit) tick_over = true;
        }
      }
      if (!tr[k].stance && tr[k].stance == tr[k - 1].stance) {
        ++swing_ticks;
        if (tick_over) ++over;
      }
    }
    std::printf("%-9s %6d | %5.1f %5.1f %5.1f | %4.1f %4.1f %4.1f | %5.1f%%\n",
                name.c_str(), ik_fail[name], sw[0], sw[1], sw[2], st[0], st[1],
                st[2],
                swing_ticks ? 100.0f * over / swing_ticks : 0.0f);
  }

  // ── near-floor world horizontal speed, worst middle leg, one swing ──
  for (const std::string name :
       {std::string("l_middle"), std::string("r_middle")}) {
    const auto& tr = trace[name];
    const float ground_z = nominal.at(name)[2];
    std::size_t half = tr.size() / 2;
    for (std::size_t i = half; i + 1 < tr.size(); ++i) {
      if (!(tr[i].stance && !tr[i + 1].stance)) continue;
      std::size_t j = i + 1;
      while (j + 1 < tr.size() && !tr[j + 1].stance) ++j;
      if (j + 1 >= tr.size()) break;
      std::printf("%s swing: ", name.c_str());
      const float hs[] = {0.005f, 0.002f};
      for (float h : hs) {
        for (std::size_t k = j; k > i + 1; --k) {
          const float zk = tr[k].target[2] - ground_z;
          const float zk1 = tr[k - 1].target[2] - ground_z;
          if (zk1 >= h && zk < h) {
            const float wx = (tr[k].world[0] - tr[k - 1].world[0]) / dt;
            const float wy = (tr[k].world[1] - tr[k - 1].world[1]) / dt;
            std::printf("z=%.0fmm v=(%+.3f,%+.3f)  ", h * 1000.0f, wx, wy);
            break;
          }
        }
      }
      std::printf("\n");
      break;  // one swing per leg is enough here
    }
  }

  // ── velocity-limited tracking model for l_middle: actual joints slew toward
  // the command at most kJointVelLimit rad/s (the URDF limit the sim enforces).
  // FK the actual angles back to a world tip trajectory: where does the
  // physical tip meet the floor, how fast is it moving, and how far does it
  // sweep before the command's stance phase takes over? ──
  for (const std::string model_leg :
       {std::string("l_middle"), std::string("r_middle")}) {
    const auto& tr = trace[model_leg];
    int spec_i = 0;
    for (int li = 0; li < 6; ++li)
      if (LEG_NAMES[li] == model_leg) spec_i = li;
    const auto& sp = spec_for(spec_i);
    const float ground_z = nominal.at(model_leg)[2];

    hexa::JointAngles act = tr.front().theta;
    std::vector<Vec3> act_world(tr.size());
    for (std::size_t k = 0; k < tr.size(); ++k) {
      for (int j = 0; j < 3; ++j) {
        const float step = tr[k].theta[j] - act[j];
        const float cap = kJointVelLimit * dt;
        act[j] += std::max(-cap, std::min(cap, step));
      }
      // FK gives the posed-frame tip; pose oscillation is mm-scale, and the
      // stance legs track (their rates are far under the cap), so body_w from
      // the commanded reconstruction still holds. Treat pose as identity for
      // the actual-vs-commanded comparison: both sides drop the same term.
      const Vec3 leg_tip = hexa::forward_kinematics(act, sp);
      const Vec3 body_tip = hexa::leg_to_body(leg_tip, sp);
      act_world[k] =
          Vec3(body_tip[0] + (tr[k].world[0] - tr[k].target[0]),
               body_tip[1] + (tr[k].world[1] - tr[k].target[1]),
               body_tip[2]);
    }
    std::printf("%s ACTUAL-tip model (joint rate capped at %.2f rad/s):\n",
                model_leg.c_str(), kJointVelLimit);
    for (std::size_t i = 1; i + 1 < tr.size(); ++i) {
      if (!(tr[i].stance && !tr[i + 1].stance)) continue;
      std::size_t j = i + 1;
      while (j + 1 < tr.size() && !tr[j + 1].stance) ++j;
      if (j + 40 >= tr.size()) break;
      // scan swing + early stance for the actual tip's floor contact
      float min_z = 1e9f;
      std::size_t touch = 0;
      for (std::size_t k = i + 2; k <= j + 40; ++k) {
        const float z = act_world[k][2] - ground_z;
        min_z = std::min(min_z, z);
        if (touch == 0 && z <= 0.0f) touch = k;
      }
      if (touch == 0) {
        std::printf("  t=%5.2f state=%-8s actual tip never reaches floor, "
                    "min_z=%+.1fmm\n",
                    tr[i + 1].t, state_value(tr[i + 1].state).c_str(),
                    min_z * 1000.0f);
        continue;
      }
      // at floor contact: world speed, then sweep while at/below floor level
      const float tv = std::hypot(act_world[touch][0] - act_world[touch - 1][0],
                                  act_world[touch][1] - act_world[touch - 1][1]) /
                       dt;
      float sweep = 0.0f;
      std::size_t k = touch;
      for (; k + 1 <= j + 40; ++k) {
        if (act_world[k + 1][2] - ground_z > 0.0005f) break;
        sweep += std::hypot(act_world[k + 1][0] - act_world[k][0],
                            act_world[k + 1][1] - act_world[k][1]);
      }
      const float cmd_z_at_touch = tr[touch].target[2] - ground_z;
      std::printf(
          "  t=%5.2f state=%-8s touch at cmd_z=%4.1fmm (%.0fms before cmd "
          "touchdown), tip speed %.3f m/s, sweeps %.1fmm over %.0fms\n",
          tr[i + 1].t, state_value(tr[i + 1].state).c_str(),
          cmd_z_at_touch * 1000.0f,
          (tr[j].t - tr[touch].t) * 1000.0f, tv, sweep * 1000.0f,
          (k - touch) * dt * 1000.0f);
    }
  }

  // ── per-swing body-frame sweep metrics for l_middle: every swing in the run,
  // tagged with the engine state it ran under. The "sweep" a body-following
  // observer sees: how far beyond the final touchdown point the body-frame
  // target goes, at what height it turns around, and how much body-frame
  // horizontal path is traced below 3 mm. ──
  {
    const auto& tr = trace["l_middle"];
    const float ground_z = nominal.at("l_middle")[2];
    std::printf("l_middle swings (body frame):\n");
    for (std::size_t i = 1; i + 1 < tr.size(); ++i) {
      if (!(tr[i].stance && !tr[i + 1].stance)) continue;
      std::size_t j = i + 1;
      while (j + 1 < tr.size() && !tr[j + 1].stance) ++j;
      if (j + 1 >= tr.size()) break;
      const Vec3 td = tr[j + 1].target;  // touchdown point (body frame)
      float max_beyond = -1e9f, max_beyond_z = 0.0f;
      float low_path = 0.0f;  // body-frame horiz path below 3mm
      float turn_z = -1.0f;   // height where body-frame y motion reverses
      float wv[4] = {-1, -1, -1, -1};  // world speed at 5/3/2/1mm (descent)
      const float hs[4] = {0.005f, 0.003f, 0.002f, 0.001f};
      for (std::size_t k = i + 2; k <= j; ++k) {
        const float beyond = tr[k].target[1] - td[1];
        const float z = tr[k].target[2] - ground_z;
        const float zp = tr[k - 1].target[2] - ground_z;
        if (beyond > max_beyond) {
          max_beyond = beyond;
          max_beyond_z = z;
        }
        const float dy = tr[k].target[1] - tr[k - 1].target[1];
        const float dyp = tr[k - 1].target[1] - tr[k - 2].target[1];
        if (turn_z < 0.0f && dy < 0.0f && dyp >= 0.0f) turn_z = z;
        if (z < 0.003f) {
          low_path += std::hypot(tr[k].target[0] - tr[k - 1].target[0], dy);
        }
        for (int h = 0; h < 4; ++h) {
          if (zp >= hs[h] && z < hs[h]) {
            wv[h] = std::hypot(tr[k].world[0] - tr[k - 1].world[0],
                               tr[k].world[1] - tr[k - 1].world[1]) /
                    dt;
          }
        }
      }
      std::printf(
          "  t=%5.2f state=%-8s beyond_td=%+6.1fmm at z=%4.1fmm turn_z=%4.1fmm "
          "path<3mm=%5.1fmm  wv@5/3/2/1mm=%.3f/%.3f/%.3f/%.3f\n",
          tr[i + 1].t, state_value(tr[i + 1].state).c_str(),
          max_beyond * 1000.0f, max_beyond_z * 1000.0f, turn_z * 1000.0f,
          low_path * 1000.0f, wv[0], wv[1], wv[2], wv[3]);
    }
  }
}

}  // namespace

int main() {
  // Forward is the control: no leg travels along its own axis, so the budget
  // never binds and these numbers must not move when it is switched on.
  run_scenario(1.0f, 0.0f, "FORWARD 100% speed");
  run_scenario(0.7071f, 0.7071f, "DIAGONAL 100% speed");
  run_scenario(0.0f, 1.0f, "LATERAL 100% speed");
  run_scenario(0.0f, 0.9f, "LATERAL 90% speed");
  run_scenario(0.0f, 0.8f, "LATERAL 80% speed");
  run_scenario(0.0f, 0.65f, "LATERAL 65% speed");
  run_scenario(0.0f, 0.5f, "LATERAL 50% speed");
  return 0;
}
