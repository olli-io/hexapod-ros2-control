# hexa_display

The **face**. Maps the robot's state (gait engine state, `cmd_vel`, posture,
battery) through an expression/gaze policy and rasterizes the eyes on the
Pi-attached 256×64 SH1122 OLED — all in one C++ node (`display_node`).

Pure sink in the dependency graph: it only subscribes to topics from the
existing chains; nothing imports it or subscribes to it. The policy half and the
render half share one process, so there is no intermediate `/display/*` topic
hop — the policy sets the renderer target with an in-process call. The renderer
(the vendored firmware `EyeAnim`/`EyeRaster` core) eases gaze and blinks through
expression changes autonomously, so the policy runs at a relaxed 10 Hz while the
render loop runs independently at 60 Hz and only pushes bytes over SPI when the
pixels change.

The robot comes up faceless (no crash) if `enabled: false`; in sim the node runs
headless (full pipeline, no SPI/GPIO) — attach the terminal emulator to watch the
face (see below).

## Layers

Mirrors the pure-library / ROS-node split used by the other C++ ports
(`hexa_gait_cpp`, `hexa_kinematics_cpp`): the policy is unit-testable standalone,
without rclcpp.

- **Support library** (`src/`, no rclcpp):
  - `expression_policy.{hpp,cpp}` — the pure `decide(inputs, config, prev)`
    policy, `selectFaceAnimation`, and the stateful `BatteryMonitor` debouncer.
  - `face_animation.{hpp,cpp}` — looping face-animation step sequences
    (breathing, idling) plus `stepCountAt` / `dueSteps`.
  - `face_animation_runner.{hpp,cpp}` — the face-animation clock (elapsed time,
    fired-step counter, randomized rest between bursts). RNG and the gaze/blink
    outputs are injected, so it is deterministic under test.
  - `Sh1122Panel.{h,cpp}` — self-contained SH1122 driver (spidev + kernel GPIO
    character device, no libgpiod).
- **Node** (`display_node.cpp`) — the only rclcpp component: caches latest topic
  samples, runs the policy + animation clock on a timer, drives the renderer
  target, and rasterizes frames on the render timer.
- **Terminal emulator** (`face_sim.cpp`) — a second, sim-only rclcpp executable
  that reuses the same support library. See below.

The `Expression` / `GazeDirection` enums and their canonical names come from the
vendored firmware core (`vendor/core/Expression.h` + `ExpressionController.cpp`),
the single source of truth shared with the ESP32 / Pico firmware.

## Expression policy

Precedence, highest first:

- **battery critical** — `dead`, gaze centered, unconditional.
- **battery warning** — `sleepy`, only while idle (zero `cmd_vel`, gait state
  folded/stand/paused, no animation mode) so the warning never masks the face
  mid-walk. Both battery thresholds ship at 0.0 (disabled) until the ADC voltage
  divider is calibrated.
- **animation mode** — `woozy` while a posture animation is active.
- **gait-state map** — YAML-configured expression per canonical gait state
  (defaults: `gait` → happy, `folded`/`paused`/`folding` → sleepy, everything
  else neutral).

Gaze:

- **vertical** — always follows body pitch from `/body/pose`: nose up → up.
  Driving forward or backward never moves the gaze up or down.
- **horizontal, gait-active** — follows `cmd_vel`: REP-103 left (`+vy`, `+wz`) →
  left. Each axis is normalized by a configured cap and sign-quantized with
  enter/exit hysteresis so the gaze does not chatter at the deadband.
- **horizontal, pose mode** — follows body tilt from `/body/pose`: yaw left /
  roll left → left.
- `dead` always forces gaze center.

## Face animations

Looping gaze/blink step sequences relayed by the animation clock; the renderer
(`EyeAnim`) still eases gaze and auto-blinks on top. While one is active it owns
the gaze; the policy gaze resumes when it ends. Distinct from the posture
animation stack in `hexa_posture` — these only drive the display.

- **breathing** — slow vertical gaze drift (up → center → down → center, 4.8 s
  period) while no `/gait/state` has been heard yet, i.e. the robot stack (servo
  UART, gait engine) is still initializing.
- **idling** — look-around-and-blink burst (3.04 s: left, blink, right, up,
  down, center, blink) once the robot has stood idle, level, and command-free
  for `idling_start_delay_s`. Bursts are spaced a random 5-10 s apart
  (`repeat_range_s`); the eyes rest at center in between rather than scanning
  continuously.

Battery warning/critical, a posture animation mode, any `cmd_vel`, or a tilted
body pose suppress the animations.

## Topics

Subscribes (inputs only — nothing subscribes to this node):

- `/gait/state` (`std_msgs/String`) — gait engine state name.
- `/cmd_vel` (`geometry_msgs/Twist`) — horizontal gaze source while walking.
- `/body/pose` (`hexa_interfaces/BodyPose`) — vertical gaze source (pitch);
  horizontal source in pose mode (yaw/roll).
- `/animation/mode` (`std_msgs/String`, transient_local depth 1, matching the
  teleop publisher) — posture animation selection.
- battery topic (`sensor_msgs/BatteryState`, sensor-data QoS, default
  `/hexa_hardware_aux/battery_state`) — real robot only.

## Configuration

All knobs in `config/display.yaml`: the policy update rate, the per-gait-state
expression map, animation/battery expressions, battery thresholds and debounce,
the gaze deadband/hysteresis/normalization caps, the idling start delay, and the
renderer's SH1122 SPI/GPIO pins, render rate, and headless switch. Expression
names are validated against the firmware enum at startup (a typo fails launch
fast).

Setting `enabled: false` in the same file makes the bringup launch files
(`sim.launch.py`, `robot.launch.py`) skip the face node entirely — the rest of
the stack is unaffected.

## Terminal emulator (sim)

There is no OLED in the sim container, so `display_node` runs headless and its
eye frames go nowhere. `face_sim` is a live terminal mirror of the face: a second
node that subscribes to the same robot-state topics and runs the *identical*
policy + `EyeAnim` pipeline (shared via the support library), then rasterizes the
two eyes into the terminal with Unicode quadrant blocks (Braille fallback on
short terminals) instead of driving SPI. It stays a pure sink — no `/display/*`
topic — just a second independent consumer of the shared topics.

Attach it on demand to a running sim:

```
./hexa sim up      # sim stack runs detached
./hexa sim face    # attach the emulator; press 'q' (or Ctrl-C) to detach
```

`face_sim` is a sibling of the sim launch (PID 1), so closing it leaves the stack
running. It is built in every image but launched only by hand — `robot.launch.py`
never starts it, so the robot runtime is unaffected. Its policy params default to
`display.yaml`'s shipped values; the `hexa sim face` command runs it with
`use_sim_time:=true` so the policy clock tracks Gazebo.

## Tests

`colcon test --packages-select hexa_display` (gtest, all headless): full policy
precedence and gaze quantization table (`test_expression_policy`), face-animation
selection and step timing (`test_face_animation`), the idle look-around rest-gap
timing (`test_face_animation_runner`), and the renderer's name parity + panel
dirty-flush + render settle (`test_face`).
