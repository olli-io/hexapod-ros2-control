# hexa_display

The **face**: one C++ node (`display_node`) that maps robot state through an
expression/gaze policy and rasterizes the eyes on the Pi's 256×64 SH1122 OLED.
Pure sink — nothing imports it or subscribes to it. Headless in sim;
`enabled: false` skips the node.

## Layout

- `shared/display_core` — the pure policy (`hexa::display`) and the vendored
  `EyeAnim`/`EyeRaster` core, shared with the Pi Pico firmware.
- `src/display_node.cpp` — rclcpp glue: caches topics, ticks the policy (~10 Hz),
  drives the renderer (60 Hz, SPI flush only on pixel change).
- `src/Sh1122Panel.{h,cpp}` — Linux SH1122 driver (spidev + GPIO char device).
- `src/face_sim.cpp` — terminal mirror for sim.

## Behavior

- **Expression precedence** — busy (`scanning`) > battery-critical (`dead`) >
  battery-warning (`sleepy`, idle only) > animation mode (`woozy`) > posture
  sticks (pose mode: tilt `happy`, shift `love`, both `angry`) > per-gait-state
  map. Battery thresholds ship disabled (0.0).
- **Gaze** — vertical follows pitch; horizontal follows `cmd_vel` when walking,
  body tilt in pose mode. Named in the robot frame; `toScreenGaze` converts to
  panel coordinates at the renderer seam.
- **`scanning`** — the only animated expression (spinners on the neutral ring).
  Worn while `/bluetooth/scanning` or `/display/busy` is true; the node ORs them
  into `PolicyInputs::busy`.
- **Face animations** — breathing until the first `/gait/state`, idling
  look-around when idle. Suppressed by any command, posed body, warning or busy.
- **Text mode** — a non-empty `/display/text` replaces the eyes with wrapped,
  centered text (Pixel Operator 16 px, max 4 lines). Empty returns the face.
  Text wins over everything, spinners included. Publishers must be
  transient_local:

  ```
  ros2 topic pub --once --qos-durability transient_local \
    /display/text std_msgs/msg/String "data: 'Hello hexapod'"
  ```

## Topics (subscribes only)

- `/gait/state` (`String`), `/cmd_vel` (`Twist`), `/body/pose`
  (`hexa_interfaces/BodyPose`), `/animation/mode` (`String`, transient_local),
  `/display/text` (`String`, transient_local), `/bluetooth/scanning` and
  `/display/busy` (`Bool`, transient_local), battery
  (`sensor_msgs/BatteryState`, real robot only).

`/display/text`, `/bluetooth/scanning` and `/display/busy` come from
`hexa_buttons`; the dependency is one-way.

## Configuration

All knobs in `config/display.yaml`: expression map (keys every `/gait/state`
value), thresholds, gaze deadband/hysteresis, idling delay, SPI/GPIO pins,
render rate, headless switch. Expression names are validated at startup.

## Sim

```
./hexa sim up
./hexa sim face    # terminal emulator; 'q' to detach
```

## Tests

`colcon test --packages-select hexa_display` — gtest, headless:
`test_expression_policy`, `test_face_animation`, `test_face_animation_runner`,
`test_face`.
