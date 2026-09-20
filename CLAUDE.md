# Repository conventions for Claude

Rules that are not derivable from the code. Package behaviour lives in each
`src/hexa_*/README.md` and `shared/*/README.md`; architecture in
`docs/architecture.md`; config in `docs/configuration.md`. Read those, do not
restate them here.

## Stack and commands

- ROS2 Jazzy, Gazebo Harmonic, colcon. Everything runs **inside the Docker sim
  container** via the `hexa` host script. Never assume native ROS2 on the host.
- `hexa sim up` / `hexa sim down` / `hexa sim logs -f` — stack lifecycle.
- `hexa sim build` — `colcon build --symlink-install` in an ephemeral container.
- `hexa sim <cmd>` — one-off command, e.g. `hexa sim ros2 topic list`.
- `./hexa sim python3 -m pytest src/<pkg>/test -q` — pytest exists only in the container.
- Pair every `hexa sim up` with a `hexa sim down`.

## Hard rules

- **Leg count is fixed at 6.** Do not parameterise it.
- `hexa_description` is the single source of truth for URDF, joint limits and
  leg geometry. Load at runtime, never duplicate.
- Gait choice, geometry, joystick mapping and presets come from YAML in
  `config/`. No magic numbers in node code.
- **Sim-first**: every feature runs against the Gazebo model before servo code.
- `hexa_simulation` owns all Gazebo code; real-robot bringup must not import it.
- `hexa_interfaces` is a leaf. `hexa_display` and `hexa_buzzer` are pure sinks:
  nothing imports or subscribes to them. `hexa_buttons` produces into the
  display by topic only.
- Locomotion is **one node**, `hexa_locomotion`, running `shared/motion_core`
  in a single 200 Hz tick. `shared/motion_core` and `shared/display_core` are
  pure C++ with no rclcpp or hardware; they are compiled by link-time seam swap
  (no `#ifdef`) into the ROS node, the Pico firmware and the host tests.
  ROS glue lives only in the node files.
- The runtime config loader is parity-tested field-by-field against
  `PipelineConfig::baked()`; keep YAML and codegen in step.
- Gait strategies are pure functions `(phase, params) → foot_target`; posture
  animations are pure functions `AnimationContext → BodyPose`. No state, I/O
  or clocks in either; those live in the gait engine and posture stack.
- REP-103 body frame (`+x` forward, `+y` left, `+z` up). SI units and radians
  throughout; convert only at UI/teleop boundaries.
- `/cmd_vel` (`geometry_msgs/Twist`) is the entry point. Stay compatible with
  `teleop_twist_*`, `twist_mux` and Nav2; no adapter topics.
- `/gait/*` topics are the engine's reports of what it has applied, never commands.

## Canonical vocabulary

Use exactly these names in identifiers, logs and docstrings. Definitions are in
`docs/leg-phases.md`; do not introduce synonyms.

- **stance** / **swing** — not *support* (for a leg's phase), *transfer*, *protraction*, *recovery*.
- **lift-off** / **touchdown**, **PEP** / **AEP**, **phase** `[0, 1)` with 0 at lift-off,
  **duty factor**, **cycle time**, **phase offset**.
- **posture** — body pose and its subsystem. Not *body trim*, *body control*.
- **animation** — a posture-stack layer only, never gait or kinematic motion.
- **gesture** — a keyframed leg + body motion played from a stand on
  `/cmd_gesture`; its leg track is neither gait nor animation, its body track
  is a posture term, not an animation layer.
- **pose keyframe** — the gesture keyframe that carries values (`pose:`);
  the others are **start**, **hold**, **home**. Not *joint keyframe*,
  *position keyframe*.
- **pose mode** — `/cmd_vel` zero, body moves on planted feet.
  **gait-active** — `/cmd_vel` non-zero.
- **settle** — the stop. Not *pause*, *stop sequence*, *re-plant*.
- **reseat** — the mirrored-pair re-plant ladder, always across one ground plane.
- **plane ramp** — the body-height half of a preset change. Not *height reseat*, *body lift*.
- **preset** — leg set + pose + stride bundle on `/cmd_preset`. Not *mode*, *profile*.
- **leg set** — `hexapod` or `quadruped`. Not *leg subset*, *active legs*.
- **park** — middle pair held at the folded pose. Not *tuck*, *stow*.
  **fold the pair** / **unfold the pair** are the only compounds for that move;
  bare **fold** is the whole-robot belly rest.
- **preset change** / **leg-set change** — not *mode switch*, *transition*.
- **support shift** — not *CoM shift*, *weight transfer*.
- **reversal ladder**, **mirror**, **knee** — not *flip*, *turnaround*,
  *phase flip*, *saturation point*.
- **crossing** — the shaped command's pass through zero after the mirror.
  Not *zero crossing*, *slew-through*.

## Documentation and comments

- **No markdown tables in `.md` files.** Use bullet lists of `**term** — definition`.
- Comments are succinct; do not comment trivial choices.

## Package READMEs

Read only the one for the package you are touching. When you change a
package's behaviour, topics, config or layout, update its README in the same
change.

- `src/hexa_bringup/README.md`
- `src/hexa_buttons/README.md`
- `src/hexa_buzzer/README.md`
- `src/hexa_description/README.md`
- `src/hexa_display/README.md`
- `src/hexa_hardware/README.md`
- `src/hexa_interfaces/README.md`
- `src/hexa_locomotion/README.md`
- `src/hexa_pico_bridge/README.md`
- `src/hexa_simulation/README.md`
- `src/hexa_teleop/README.md`
- `src/hexa_webteleop/README.md`
- `shared/motion_core/README.md`
- `shared/display_core/README.md`
