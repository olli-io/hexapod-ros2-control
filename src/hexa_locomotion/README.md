# hexa_locomotion

The locomotion controller: one node that compiles the shared control brain
([`shared/motion_core`](../../shared/motion_core)) and runs the whole
velocity → gait → posture → compose/IK pipeline in a single 200 Hz tick. It is
the sole locomotion path; `hexa_teleop` / `hexa_webteleop` feed it, and it
publishes joint commands. See [`docs/architecture.md`](../../docs/architecture.md)
for where it sits in the graph.

The brain is the same float source the Pi Pico firmware and
[`hexa_pico_bridge`](../hexa_pico_bridge/README.md) compile, a link-time seam
swap with no `#ifdef`. Only the seams differ here:

- **Input** — `CommandIntent` built from `/cmd_vel` plus the discrete command
  topics, handed to the core `tick`. `map_joy` is bypassed, so the node is
  `cmd_vel`-native (Nav2, `twist_mux`, `teleop_twist_*`).
- **Config** — `PipelineConfig` loaded at startup from
  [`geometry.yaml`](../hexa_description/config/geometry.yaml),
  [`tuning.yaml`](../hexa_description/config/tuning.yaml) and
  [`gestures.yaml`](../hexa_description/config/gestures.yaml) by
  `src/pipeline_config_loader.cpp`. On a load error it falls back to the baked
  defaults, which [`gen_config.py`](../../shared/motion_core/tools/gen_config.py)
  bakes from the same YAMLs at build time. The knobs themselves are documented in
  [`docs/configuration.md`](../../docs/configuration.md).
- **Clock** — a ROS timer on the node clock, so the tick follows sim time under
  `use_sim_time`.
- **Output** — the 18 joint angles as `Float64MultiArray`. Firmware joint order
  equals the controller's `joints:` list, so there is no remap.

## Topics

Subscribed:

- **`/cmd_vel`** (`geometry_msgs/Twist`) — velocity command, m/s and rad/s. A
  stale publisher settles the gait via the supervisor's input timeout.
- **`/cmd_gait`**, **`/cmd_preset`**, **`/animation/mode`** (`std_msgs/String`,
  latched) — gait, preset and posture-animation selection. The preset is
  re-asserted every tick so a restarted node comes back on the operator's preset.
- **`/gait/initialize`** (`std_msgs/Empty`) — stand up from the belly, or fold.
- **`/cmd_gesture`** (`std_msgs/String`, volatile) — play a gesture by id from
  `gestures.yaml`. Accepted only from a stand on the default preset; a
  refusal is logged with the state and preset. Consumed once, never replayed.
- **`/body/pose`** (`hexa_interfaces/BodyPose`) — posture offsets.
- **`/hardware/fault`** (`std_msgs/Bool`, latched) — over-current level from
  [`hexa_hardware`](../hexa_hardware/README.md).
- **`battery_topic`** (`sensor_msgs/BatteryState`) — pack voltage for the
  undervoltage ladder. Absent in sim.

Published, all latched and on change only:

- **`/joint_group_position_controller/commands`** (`Float64MultiArray`) — the
  joint command, every tick. Topic name is the `command_topic` parameter.
- **`/gait/state`** (`String`) — engine state; the input of the
  [`hexa_display`](../hexa_display/README.md) face.
- **`/gait/leg_set`**, **`/gait/preset`** (`String`) — what the engine has
  *applied*. Report topics, never commands: the command topics are latched, so
  a refused request stays on them.
- **`/gait/gesture`** (`String`) — the running gesture's id, `""` between
  gestures.
- **`/hardware/relay_cmd`** (`Bool`) — servo-rail arm intent for `hexa_hardware`.
- **`/hardware/undervoltage`** (`UInt8`) — undervoltage rung 0–3, escalate-only.

Service **`~/reload_config`** (`std_srvs/Trigger`) re-reads all three YAMLs
and swaps in a fresh pipeline without a restart (`./hexa sim reload`). The
pipeline cold-starts at `FOLDED`; a bad file, including a gesture keyframe the
legs cannot reach, keeps the current pipeline; a latched undervoltage cutoff
refuses the swap.

## Vocabulary

Gait, posture and transition terms follow
[`docs/leg-phases.md`](../../docs/leg-phases.md) and the repo `CLAUDE.md`.

## Tests

- **`test/test_config_loader.cpp`** — parity: the runtime loader reproduces
  `PipelineConfig::baked()` field by field over the same YAMLs, gestures
  included, so a YAML edit can never drift from the codegen.
- The brain itself is covered off-target in
  [`shared/motion_core/test`](../../shared/motion_core/test/README.md).

```sh
./hexa sim build
./hexa sim colcon test --packages-select hexa_locomotion
```

Launched by [`hexa_bringup`](../hexa_bringup/README.md) in both sim and robot
launches.
