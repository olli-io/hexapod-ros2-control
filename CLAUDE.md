# Repository conventions for Claude

Ground rules for AI assistants working in this hexapod ROS2 codebase.

## Stack

- ROS2 Jazzy (Ubuntu 24.04), Gazebo Harmonic, colcon workspace.
- All build/run commands execute **inside the Docker dev container**: `./scripts/dev.sh`. Do not assume native ROS2 on the host.
- ROS2 packages live under `src/hexa_*/`. The top-level `README.md` documents the dependency graph and runtime data flow.
- **Leg count is fixed at 6.** Do not parameterise it.

## Architectural rules

- Strict one-way dependency chain: `hexa_teleop → hexa_control → hexa_gait → hexa_kinematics → hexa_hardware`. No back-edges, no cycles. `hexa_bringup` composes the chain via launch files only.
- `hexa_interfaces` depends on nothing hexapod-specific (leaf).
- `hexa_description` is the **single source of truth** for URDF, joint limits, and leg geometry. Never duplicate these values elsewhere — load them at runtime.
- `hexa_simulation` owns **all** Gazebo-specific code. The real-robot bringup must not import it.
- Library code in `hexa_kinematics/` must be importable without `rclpy` (pure Python, unit-testable standalone). ROS glue lives in separate node files (e.g. `ik_node.py`).
- Gait strategies are pure functions: `(phase, params) → foot_target`. No state, no I/O, no clocks. The phase clock and per-leg transition state live in the engine, not in strategies.

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

Full definitions in `docs/leg-phases.md`. Do not introduce new synonyms.

## Language choice

- `ament_python`: gait, kinematics, control, teleop, and other node code.
- `ament_cmake` (C++): only where required — pluginlib (`hexa_hardware`), description, simulation, and bringup composition.
- Do not reach for C++ speculatively for performance. Profile first.

## Documentation formatting

- **No markdown tables in `.md` files.** Anywhere — package READMEs, `/docs/`, top-level README.
- Use bullet lists with `**term** — definition` pairs instead. Example:

      - **stance** — also called *support*, *retraction*.
      - **swing** — also called *transfer*, *protraction*.

- Broader principle: prefer documentation formats equally readable to humans and AI agents (prose, bulleted lists, definition pairs).
