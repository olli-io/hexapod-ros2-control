# Repository conventions for Claude

Ground rules for AI assistants working in this hexapod ROS2 codebase.

## Stack

- ROS2 Jazzy (Ubuntu 24.04), Gazebo Harmonic, colcon workspace.
- All build/run commands execute **inside the Docker sim container**, driven by the `hexa` host script in the repo root. The sim stack runs as the container's PID 1, so lifecycle is docker-native: `hexa sim up` (bring the stack up detached), `hexa sim down`, `hexa sim logs -f`. Build the workspace with `hexa sim build` (ephemeral `compose run --rm`, which runs `colcon build --symlink-install` in the container), and run one-off commands with `hexa sim <cmd>` (e.g. `hexa sim ros2 topic list`). Do not assume native ROS2 on the host.
- Tests: `./hexa sim python3 -m pytest src/<pkg>/test -q` from the repo root (pytest only exists inside the container).
- ROS2 packages live under `src/hexa_*/`. The top-level `README.md` documents the dependency graph and runtime data flow.
- **Leg count is fixed at 6.** Do not parameterise it.

## Architectural rules

- Locomotion is a **single node**, `hexa_locomotion`, which compiles the shared
  control brain (`shared/hexa_pipeline`) and runs the whole velocity → gait →
  posture → compose/IK pipeline in one 200 Hz tick. It is the sole locomotion
  path (the former `hexa_control → hexa_gait → hexa_posture → hexa_kinematics`
  topic-wired node chain has been deleted). `hexa_teleop → /cmd_vel` (+ the
  discrete command topics) is the entry point; `hexa_locomotion` publishes joint
  commands. See the section below for `shared/hexa_pipeline`.
- `hexa_interfaces` depends on nothing hexapod-specific (leaf).
- `hexa_description` is the **single source of truth** for URDF, joint limits, and leg geometry. Never duplicate these values elsewhere — load them at runtime.
- `hexa_simulation` owns **all** Gazebo-specific code. The real-robot bringup must not import it.
- The face is a single-node sink (`hexa_display`, ament_cmake C++), launched only by `hexa_bringup`. It subscribes to topics `hexa_locomotion` publishes (`/gait/state`, etc.), maps robot state through an expression/gaze policy, and rasterizes the eyes on a Pi-attached SH1122 OLED (headless in sim) — all in one process, no intermediate `/display/*` topic hop. It is a **pure sink** of robot state; nothing imports it or subscribes to it. It owns **all** panel/SPI/GPIO code and the vendored firmware eye core (`src/hexa_display/vendor/`). The policy half (`expression_policy`, `face_animation`, `face_animation_runner`) is pure C++, unit-testable without rclcpp; the ROS glue and rendering live in `display_node.cpp`.
- The control brain in `shared/hexa_pipeline/` is pure C++, importable without `rclcpp` (unit-testable standalone via `shared/hexa_pipeline/test/`). ROS glue lives only in `hexa_locomotion`'s node files (e.g. `locomotion_node.cpp`, `pipeline_config_loader.cpp`).
- Gait strategies are pure functions: `(phase, params) → foot_target`. No state, no I/O, no clocks. The phase clock and per-leg transition state live in the gait engine, not in strategies.
- Posture animations are pure functions: `AnimationContext → BodyPose`. No state outside the animation instance, no I/O, no clocks. The clock and walking-vs-idle state live in the posture stack, not in animations.

## Configurability

- Gait choice, body geometry, leg dimensions, joystick mapping: load from YAML in `config/`. No magic numbers in node code.
- **Sim-first**: every feature must run against the Gazebo model before any servo code is touched.

## Frames, units, conventions

- REP-103 body frame: right-handed, `+x` forward, `+y` left, `+z` up.
- Linear in m/s, angular in rad/s, angles in radians throughout code. Convert only at UI/teleop boundaries.
- `cmd_vel` (`geometry_msgs/Twist`) is the high-level entry point. Stay plug-compatible with `teleop_twist_*`, `twist_mux`, and Nav2 — do not introduce adapter topics.

## Canonical vocabulary

Use exactly these names in identifiers, log messages, and docstrings — not the literature synonyms:

- **stance** — foot on ground (not *support*, *retraction*, *power stroke*).
- **swing** — foot in air (not *transfer*, *protraction*, *recovery*).
- **lift-off** — stance → swing transition.
- **touchdown** — swing → stance transition.
- **PEP** — Posterior Extreme Position (lift-off point in body frame).
- **AEP** — Anterior Extreme Position (touchdown point in body frame).
- **phase** — float in `[0, 1)`, `phase = 0` at lift-off.
- **duty factor** (β) — fraction of cycle in stance.
- **cycle time** — duration of one full PEP → PEP cycle, in seconds.
- **phase offset** — leg's cycle start relative to a reference leg.
- **posture** — body pose state and the subsystem that controls it. Covers both static positioning (feet grounded, body translates/yaws/tilts) and gait-coupled body animation (sway, lean, bob). Not *body trim*, *body control*, *body animation* as standalone terms.
- **animation** — a pure function from `AnimationContext` to a `BodyPose` offset; one ingredient in the posture stack. Use this word only for the posture animation-stack layers (`shared/hexa_pipeline/posture`), never for gait or kinematic motion.
- **pose mode** — `/cmd_vel` is zero, body posture changes while feet stay planted.
- **gait-active** — `/cmd_vel` is non-zero; posture animations run on top of the walking gait.

Full definitions in `docs/leg-phases.md`. Do not introduce new synonyms.

## Consolidated single-node locomotion (`hexa_locomotion` + `shared/hexa_pipeline`)

`hexa_locomotion` is the locomotion controller: one node running the whole
velocity → gait → posture → compose/IK pipeline in a single 200 Hz loop, mirroring
the Pi Pico firmware's single loop. It replaced the old multi-node chain
(`hexa_control` / `hexa_gait` / `hexa_posture` / `hexa_kinematics`, all now
deleted, along with their `*_cpp` ports).

- **`shared/hexa_pipeline/`** is the target-agnostic **float** control brain
  (`hexa::pipeline::Pipeline` + `hexa::gait`/`hexa::posture`/`hexa::control`/
  `hexa::supervisor`), extracted from `pi-pico-firmware/` (which now keeps only its
  Pico hardware seams). It is compiled directly — a link-time seam swap, no
  `#ifdef` — by the Pico firmware, `hexa_pico_bridge`, and `hexa_locomotion`. One
  brain, shared bug-for-bug across firmware and sim. Its host test harness lives in
  `shared/hexa_pipeline/test/`. It is the **sole** locomotion implementation; the
  earlier double-precision `hexa_gait_cpp`/`hexa_posture_cpp`/`hexa_kinematics_cpp`
  libraries and the Python originals have been removed.
- **Seams** (caller-owned, the only things that differ per target): the Pipeline
  `tick` has a joy overload (Pico/bridge run `map_joy`) and a core
  `tick(CommandIntent, TickInput)` (the ROS node builds `CommandIntent` from
  `/cmd_vel` + `/cmd_gait` + `/gait/initialize` + `/body/pose` + `/animation/mode`,
  bypassing `map_joy`); config comes from a `PipelineConfig` (baked constexpr on
  the Pico, loaded from `geometry.yaml` + `tuning.yaml` at startup in
  `hexa_locomotion` via `pipeline_config_loader.cpp`, scope = geometry+tuning
  only). The runtime loader is parity-tested field-by-field against
  `PipelineConfig::baked()` (`hexa_locomotion/test/test_config_loader.cpp`) so a
  YAML edit can never silently drift from the baked codegen.
- **`hexa_locomotion`** folds `ik_node` + `joint_command_bridge` (publishes
  `Float64MultiArray` on `/joint_group_position_controller/commands` directly) and
  re-publishes `/gait/state` for the face sink. Because it composes both the
  velocity/gait and body-pose halves of the pipeline **in one process**, the node
  is the in-code composition point (there is no longer a separate chain for
  `hexa_bringup` to wire via launch). `/cmd_vel` stays the entry point
  (Nav2/twist_mux compatible); the runtime-YAML load honors the
  load-config-at-runtime rule.

## Documentation formatting

- **No markdown tables in `.md` files.** Anywhere — package READMEs, `/docs/`, top-level README.
- Use bullet lists with `**term** — definition` pairs instead. Example:

      - **stance** — also called *support*, *retraction*.
      - **swing** — also called *transfer*, *protraction*.

- Broader principle: prefer documentation formats equally readable to humans and AI agents (bulleted lists, definition pairs). Keep comments succinct.
