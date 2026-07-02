# Part 08 — Posture (body pose + animation stack)

**Goal:** port the posture chain so the body translates/tilts on command and the
gait-coupled animations (sway, bounce, body-roll) ride on top of walking.
Completes full parity.

**Depends on:** 04, 05, 06. **Blocks:** 09.

## Scope

- `src/posture/pose.{hpp,cpp}` — fork `hexa_posture/pose.py`: `BodyPose{x,y,z,roll,pitch,yaw}`, `add` (component-wise, small-offset valid), `scale`, `PoseLimits{x0.05,y0.05,z0.04,roll0.30,pitch0.30,yaw0.50}`, `clamp`. Float.
- `src/posture/animations.{hpp,cpp}` — fork `animations/base.py` + the seven pure animations:
  - `AnimationContext{t, walking, master_phase?, gait_name?, support_centroid_xy?, swing_lift_z?}`; `Stack` sums layers via `pose.add`.
  - `Still`, `Breathing` (idle `z=A·sin(2πt/period)`, A0.005 P4), `GaitSway` (walking+centroid: `x=k·cx, y=k·cy`, `k=gain·strength`), `GaitBounce` (tripod-only: `z=arc·clamp(swing_lift/step_ref,0,1)`), `VerticalBodyRoll` (tripod: `z=−zA·cos2πφ`, `pitch=−pA·sin2π(φ+off)`), `HorizontalBodyRoll` (`y`,`yaw` mirror), `BodyRoll3D` (quarter-cycle offset combine). Phase-locked ones gate on `walking ∧ master_phase ∧ gait_name=="tripod"`.
- `src/posture/` node logic (fork `posture_node.py`, ~584 lines) into the tick:
  - Derive signals from the engine's `LegOutput`s (no topics): support centroid = mean stance `foot_target.xy` (≥3 stance legs else hold), swing lift = max swing z − mean stance z (≥0), `master_phase = master % 1`.
  - First-order LPFs `alpha=dt/(tau+dt)` (`support_centroid_tau0.1`, `swing_lift_tau0.04`), seed from raw, hold on degenerate frames.
  - Gate: only run the stack when engine state ∈ `{stand,engaging,gait,pausing,paused,resuming,reseating}` else emit identity.
  - Select stack by animation mode (`enabled_animations` default; per-mode `still+one` stacks from `animation_mode_animations`).
  - `body_pose_target = clamp(add(user_pose_from_map_joy, animated), PoseLimits)` → this is the pose fed to the compose step in part 05.
- Wire `map_joy`'s pose output (part 07) as `user_pose`, and the selected `animation_name` to switch the active stack.

## Done when / verification

- **Pose mode** (`cmd_vel`=0): sticks translate/tilt the body while feet stay planted; height persists across mode toggles; record + revert behave.
- **Gait-active:** walking tripod shows the body swaying/bouncing/rolling in phase with the gait; the animations vanish (identity) in non-tripod gaits and when not walking (Breathing appears only when idle).
- Signal derivation sanity: log filtered centroid/swing-lift during a tripod walk → smooth, bounded, tracks the stance polygon.
- Host: compare the full posture stack output for a recorded engine trace against the Python `posture_node` within tolerance.
