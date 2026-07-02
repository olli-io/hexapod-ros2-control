# Part 05 — Kinematics (FK/IK + body transforms, float)

**Goal:** port the leg kinematics so the robot can hold a commanded stance. First
motion of the real legs. Proves geometry, calibration, and the compose→IK→pulse
path end-to-end.

**Depends on:** 03 (servo link), 04 (vec3, config, harness). **Blocks:** 06, 08.

## Scope

- `src/kinematics/leg_ik.{hpp,cpp}` — fork `hexa_kinematics_cpp/src/leg_ik.cpp` (~63 lines):
  - **FK:** `r = coxa + femur·cos(θf) + tibia·cos(θf+θt)`; `z = −femur·sin(θf) − tibia·sin(θf+θt)`; return `(r·cosθc, r·sinθc, z)`.
  - **IK (knee-up):** `θc=atan2(y,x)`; `r'=hypot(x,y)−coxa`; `d=hypot(r',z)`; reachability throw if `d>f+t+1e-6` or `d<|f−t|−1e-6`; law-of-cosines w/ clamp; `θf=α−β` (`α=atan2(−z,r')`), `θt=π−γ`.
  - Convert all `double`→`float` (`sinf/cosf/atan2f/acosf/hypotf`). Keep `UnreachableTarget`.
- `src/kinematics/body_transform.{hpp,cpp}` — fork `body_transform.cpp`: `body_to_leg` (subtract `mount_xyz`, rotate `−mount_yaw`), `leg_to_body`, `apply_body_pose` (`R^T(p−t)`, `R=Rz(yaw)Ry(pitch)Rx(roll)`).
- In `main.cpp`, add the **compose step** for a static stance: for each leg take the nominal foot target → `apply_body_pose` (identity pose for now) → `body_to_leg` → `inverse_kinematics` → `to_pulse_us` (part 03) → Chica SET. Mirror the exact ordering + joint→pin mapping in `hexa_kinematics_cpp/src/ik_node.cpp` / `joint_command_bridge.cpp`.
- Compute the nominal stance from `config_generated.hpp` (standing pose angles → FK → `leg_to_body`), matching `nominal_stance_from_yaml` in `engine.cpp`.

## Done when / verification

- **Host:** FK/IK round-trip (`fk(ik(p))≈p`) within 1e-4; golden-trace vs the double `hexa_kinematics_cpp` over a target sweep within ~1e-3 rad.
- **Target (on a stand, feet off ground first):** command the standing pose → all 18 joints move to a symmetric, correct-looking stance; measure a couple of link angles against expected.
- Apply a small nonzero `BodyPose` (e.g. z −0.02 m, pitch 5°) statically → body visibly translates/tilts while feet stay planted, proving `apply_body_pose` composition.
- Feed a deliberately unreachable target → IK throws, caught, leg holds last-good angle (no servo command spike).
