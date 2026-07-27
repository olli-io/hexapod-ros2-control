# hexa_display

The **face**. Maps the robot's state (gait engine state, `cmd_vel`, posture,
battery) through an expression/gaze policy and rasterizes the eyes on the
Pi-attached 256×64 SH1122 OLED — all in one C++ node (`display_node`).

Pure sink: it only subscribes to topics from the existing chains; nothing imports
it or subscribes to it. Policy and render share one process (no `/display/*`
hop). The policy runs at ~10 Hz; the renderer (vendored `EyeAnim`/`EyeRaster`)
eases gaze/blinks autonomously at 60 Hz and only pushes bytes over SPI when
pixels change. With `enabled: false` the robot comes up faceless; in sim the node
runs headless (no SPI/GPIO).

## Layout

The pure, target-agnostic policy lives in **`shared/display_core`** (namespace
`hexa::display`), compiled directly by this node, the Pi Pico firmware, and the
firmware host test — the same way `hexa_locomotion` consumes `shared/motion_core`.
This package keeps only the ROS + Linux-panel code:

- `src/display_node.cpp` — the only rclcpp component: caches topic samples, runs
  the policy + animation clock, drives the renderer, rasterizes to the panel.
- `src/face_sim.cpp` — sim-only terminal mirror (see below).
- `src/Sh1122Panel.{h,cpp}` — self-contained Linux SH1122 driver (spidev + kernel
  GPIO char device, no libgpiod).
- `hexa_display_support` (CMake) — links the `shared/display_core` sources +
  `Sh1122Panel`; the gtest suites link this directly.

## Behavior

- **Expression** — precedence: Bluetooth scanning (`scanning`) >
  battery-critical (`dead`) > battery-warning (`sleepy`, idle only) > animation
  mode (`woozy`) > per-gait-state map. Battery thresholds ship disabled (0.0)
  until the ADC divider is calibrated.
- **Gaze** — vertical follows body pitch; horizontal follows `cmd_vel` when
  walking or body tilt in pose mode, sign-quantized with hysteresis. `dead` and
  `scanning` force center.
- **`scanning`** — the one animated expression: the neutral eyes' ring with a
  gap taken out of it, turning once a second. It runs while `hexa_buttons`
  reports a pairing scan on `/bluetooth/scanning`, and it outranks everything
  else because the operator asked for it by hand and is watching the panel for
  it. `AnimFrame::phase` carries the angle; it is quantized into 30 steps a turn
  so the dirty-flush skip still holds between steps.
- **Face animations** — looping gaze/blink sequences: **breathing** until the
  first `/gait/state` arrives, **idling** look-around once idle/level/command-free.
  Suppressed by battery warning, animation mode, Bluetooth scanning, `cmd_vel`,
  or a tilted pose.
- **Text mode** — a non-empty `/display/text` message replaces the eyes with the
  message text (vendored Pixel Operator 16 px font from `shared/display_core/fonts`,
  ASCII + Latin-1): hard-breaks on `\n`, word-wraps to the panel width, centers
  the block, at most 4 lines of ~42 chars. An empty string returns to the face.
  The policy keeps ticking underneath, so the face resumes with current state;
  the panel only reflushes when the message changes. The subscription is
  transient_local, so publishers must match, e.g.:

  ```
  ros2 topic pub --once --qos-durability transient_local \
    /display/text std_msgs/msg/String "data: 'Hello hexapod'"
  ```

## Topics (subscribes only)

- `/gait/state` (`std_msgs/String`), `/cmd_vel` (`geometry_msgs/Twist`),
  `/body/pose` (`hexa_interfaces/BodyPose`), `/animation/mode` (`std_msgs/String`,
  transient_local depth 1), `/display/text` (`std_msgs/String`, transient_local
  depth 1 — non-empty shows the text screen, empty returns the face),
  `/bluetooth/scanning` (`std_msgs/Bool`, transient_local depth 1 — true wears
  the spinners), and a battery topic (`sensor_msgs/BatteryState`, default
  `/hexa_hardware_aux/battery_state`, real robot only).

`/display/text` and `/bluetooth/scanning` are both published by `hexa_buttons`
(the front-panel GPIO buttons). That package depends on this one's topic names;
nothing here depends on it, so the face stays a pure sink.

## Configuration

All knobs in `config/display.yaml`: policy rate, per-gait-state expression map,
animation/battery expressions and thresholds, gaze deadband/hysteresis/caps,
idling delay, and the SH1122 SPI/GPIO pins + render rate + headless switch.
Expression names are validated against the firmware enum at startup (a typo fails
launch fast). `enabled: false` makes the bringup launch files skip the node.

## Terminal emulator (sim)

`face_sim` mirrors the face into the terminal (Unicode blocks) using the
*identical* policy + `EyeAnim` pipeline, since the sim container has no OLED:

```
./hexa sim up      # sim stack runs detached
./hexa sim face    # attach the emulator; 'q' or Ctrl-C to detach
```

Built in every image but launched only by hand — `robot.launch.py` never starts
it, so the robot runtime is unaffected.

## Tests

`colcon test --packages-select hexa_display` (gtest, all headless):
`test_expression_policy`, `test_face_animation`, `test_face_animation_runner`,
`test_face` (name parity + panel dirty-flush + render settle + the scanning
spinner's geometry and its non-settling redraw).
