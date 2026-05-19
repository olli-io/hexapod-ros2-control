# Repository conventions for Claude

Ground rules for AI assistants working in this hexapod ROS2 codebase.

## Stack

- ROS2 Jazzy (Ubuntu 24.04), Gazebo Harmonic, colcon workspace.
- All build/run commands execute **inside the Docker dev container**: `hexa --dev` (the `hexa` host script in the repo root) opens a shell in the container. Inside the container the workspace CLI is `pod` (e.g. `pod build`, `pod sim`, `pod teleop`). Do not assume native ROS2 on the host.
- ROS2 packages live under `src/hexa_*/`. The top-level `README.md` documents the dependency graph and runtime data flow.
- **Leg count is fixed at 6.** Do not parameterise it.

## Architectural rules

- Two parallel chains converge in `hexa_kinematics`:
  - Velocity / gait chain: `hexa_teleop → hexa_control → hexa_gait → hexa_kinematics → hexa_hardware`.
  - Body-pose chain: `hexa_teleop → hexa_posture → hexa_kinematics`.
  No back-edges, no cycles. The two chains never import each other; `hexa_kinematics` composes their outputs. `hexa_bringup` composes both chains via launch files only.
- `hexa_interfaces` depends on nothing hexapod-specific (leaf).
- `hexa_description` is the **single source of truth** for URDF, joint limits, and leg geometry. Never duplicate these values elsewhere — load them at runtime.
- `hexa_simulation` owns **all** Gazebo-specific code. The real-robot bringup must not import it.
- Library code in `hexa_kinematics/` and `hexa_posture/` must be importable without `rclpy` (pure Python, unit-testable standalone). ROS glue lives in separate node files (e.g. `ik_node.py`, `posture_node.py`).
- Gait strategies are pure functions: `(phase, params) → foot_target`. No state, no I/O, no clocks. The phase clock and per-leg transition state live in the gait engine, not in strategies.
- Posture animations are pure functions: `AnimationContext → BodyPose`. No state outside the animation instance, no I/O, no clocks. The clock and walking-vs-idle state live in the posture node, not in animations.

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
- **animation** — a pure function from `AnimationContext` to a `BodyPose` offset; one ingredient in the posture stack. Use this word only inside `hexa_posture` for animation-stack layers, never for gait or kinematic motion.
- **pose mode** — `/cmd_vel` is zero, body posture changes while feet stay planted.
- **gait-active** — `/cmd_vel` is non-zero; posture animations run on top of the walking gait.

Full definitions in `docs/leg-phases.md`. Do not introduce new synonyms.

## Language choice

- `ament_python`: gait, kinematics, posture, control, teleop, and other node code.
- `ament_cmake` (C++): only where required — pluginlib (`hexa_hardware`), description, simulation, and bringup composition.
- Do not reach for C++ speculatively for performance. Profile first.

## Documentation formatting

- **No markdown tables in `.md` files.** Anywhere — package READMEs, `/docs/`, top-level README.
- Use bullet lists with `**term** — definition` pairs instead. Example:

      - **stance** — also called *support*, *retraction*.
      - **swing** — also called *transfer*, *protraction*.

- Broader principle: prefer documentation formats equally readable to humans and AI agents (prose, bulleted lists, definition pairs).
