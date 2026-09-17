# motion_core

Target-agnostic control brain for the hexapod: pure C++, `float`, no ROS, no
Pico SDK, no I/O. Compiled unchanged (link-time seam swap, no `#ifdef`) into
[`hexa_locomotion`](../../src/hexa_locomotion/README.md),
[`hexa_pico_bridge`](../../src/hexa_pico_bridge/README.md) and the Pi Pico
firmware. One brain, one 200 Hz tick, bug-for-bug identical across sim and
hardware. The seams each caller supplies — input, config source, clock, output
— are documented in the caller's README, not here.

## Layout

- **`pipeline.*`** — `hexa::pipeline::Pipeline`, the one entry point. Owns every
  stage below and runs them in order once per tick.
- **`pipeline_config.*`** — `PipelineConfig`: geometry + tuning, the only
  runtime-supplied scope. `baked()` reconstructs `config_generated.hpp`
  (emitted by `tools/gen_config.py`). Joystick, servo calibration and
  supervisor thresholds stay baked.
- **`joy_mapping.*`** — `map_joy`: gamepad snapshot → `CommandIntent`. Mode FSM
  (posture / gait / animation), cyclers, two-press init. Joy path only.
- **`supervisor.*`** — failsafes: stale-input watchdog, undervolt ladder
  (warn → fold → cutoff, monotonic per power cycle), relay arming, LED policy,
  tick-jitter stats.
- **`control.*`** — velocity shaping: cut `(vx, vy, wz)` to the active gait's
  foot-speed envelope, price the heading against the radial stride budget,
  then rate-cap toward it (`BodyVelocityLimiter`).
- **`gait/`** — the only stateful part of the gait chain. `Engine` holds the
  state machine, `GaitClock` (master phase, per-leg offsets, mirror),
  `StanceIntegrator`, `SwingPlanner`, and the ladders: `stand_transition`
  (fold ↔ stand), `engagement` (stand → gait), `reseat` (feet → new stance),
  `reversal` (travel reversal), pair fold/unfold. `gaits/` holds the
  strategies — `tripod`, `surf`, `tetrapod`, `crawl`, `ripple`, `quad_walk`,
  `quad_canter` — each a pure `(phase, stride, leg) → foot_target`.
- **`posture/`** — `PostureController`: user body pose + animation stack →
  `BodyPose` offset. Animations are pure `AnimationContext → BodyPose`
  (`breathing`, `gait_sway`, `support_shift`, `gait_bounce`, body rolls).
- **`gesture/`** — keyframed leg + body motions played from a stand
  (`gestures.yaml`). `keyframe` samples one track (`ease` smoothstep or a
  `continuous` Hermite run, slope-capped so a component never leaves the
  range its knots span); `GesturePlayer` completes a gesture's per-leg joint
  tables and body table with the implicit start and return knots (the stance
  solved through IK once) and plays them off one clock, reporting each
  tracked leg `direct` with its joint angles; `validate_gestures` checks every
  leg knot against the joint limits and the ground plane at construction.
  Its leg track is neither gait nor animation; its body track is a posture
  term, not an animation layer.
- **`kinematics/`** — `apply_body_pose`, `body_to_leg`, `inverse_kinematics`
  (knee-up branch, throws `UnreachableTarget`).
- **`energize_sweep.hpp`, `servo_out.hpp`** — hardware-side helpers the Pico
  seam uses; not part of the tick.
- **`test/`** — standalone gtest harness, see `test/README.md`.

## The tick

`Pipeline::tick(CommandIntent, TickInput) → TickResult`, every 5 ms. The joy
overload runs `map_joy` first, then the same core.

1. **Record tick edge** — supervisor jitter accounting.
2. **Leg set / preset / gait / gesture requests** — an init edge on the belly
   resolves a leg set (start = six, select = four) to a preset. A preset
   select from a stand is held until the body pose is neutral (3 s timeout,
   then dropped and reported as `gait_blocked_by_posture`), then committed
   together with the gait that walks it. A gait select naming the other leg
   set is refused. A gesture select is accepted only from a stand on the
   default preset with nothing armed; the engine starts it on its next tick.
3. **Follow the applied preset** — when the engine reports a new preset, copy
   its velocity caps, nominal stance and stride into `Control` and the joy
   scaling. The engine's report, not the request, drives this.
4. **Init edge** — `start_initialize()` from FOLDED / FAULT, else
   `request_fold()`. A latched `hardware_fault` then wins over a same-tick
   start.
5. **Animation mode** — unknown names are ignored and reported.
6. **Supervisor step** — watchdog + battery → `Decision`. `force_zero` zeros
   the command; the fold rung queues one `request_fold()`.
7. **Reversal shaping** — `Engine::shape_reversal` holds a reversing command
   until all six feet are planted and on schedule, then mirrors the clock and
   holds the gait clock while the shaped command crosses zero.
8. **Velocity shaping** — `Control::shape` on the applied leg set.
9. **Gait engine** — `Engine::update(dt, v, wz)` → per-leg `LegOutput`
   (foot target, stance flag, phase, parked, direct joints). In `GESTURE` the
   player's legs come out here and the command is ignored.
10. **Posture** — user pose (pinned to identity while the middle pair is in
    flight) + animation stack, gated on `walking` and engine state, plus a
    running gesture's body term, all under one clamp → `BodyPose`.
11. **Compose / IK** — per leg: `apply_body_pose` → `body_to_leg` →
    `inverse_kinematics`. An unreachable target holds that leg's last-good
    angles. A parked leg writes the folded pose directly; a direct leg (a
    gesture's tracked leg) writes its joint angles directly, so the body pose
    moves the body under it, not the leg.
12. **Report** — 18 joint angles, engine state, applied leg set / preset,
    master phase, supervisor decision, and the raw intent for the face.

## Engine states

`FOLDED → INITIALIZE → STAND ⇄ ENGAGING → GAIT → SETTLING → STAND`,
`STAND → FOLDING → FOLDED`, `STAND → RESEATING` (height change, preset change,
settle hand-off, abandoned engagement), `STAND ⇄ FOLDING_PAIR / UNFOLDING_PAIR`
(leg-set change), `STAND → GESTURE → STAND` (default preset only), `FAULT`
from anywhere. Walking drops any armed change; a
stop is always a **settle** (gait runs on at zero stride) unless the gait is too
slow, then a **reseat**.

## Rules

- No clocks, no I/O. The caller supplies `now_us` and `dt`.
- Strategies and animations are pure functions. State lives in `Engine` and
  `PostureController` only.
- Leg count is fixed at 6. Vocabulary per `docs/leg-phases.md` and the
  repo `CLAUDE.md`.
