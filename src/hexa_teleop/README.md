# hexa_teleop

Gamepad teleop. Publishes the high-level command topics the rest of the
stack consumes: `/cmd_vel`, `/body/pose`, `/cmd_gait`, `/animation/mode`,
`/gait/initialize`. Deliberately thin — the single, swappable producer of
high-level commands; an autonomy node would publish the same topics and
replace it transparently.

- **`joy_publisher.py`** — reads `/dev/input/jsN` directly, publishes
  `sensor_msgs/Joy` on `/joy`. Drop-in for upstream `joy_node` with
  reliable hot-plug in the sim container (see below).
- **`teleop_joy.py`** — `/joy` → command topics. Mode-switched stick
  semantics; mapping fully configurable via YAML.
- **`joy_mapping.py`** — pure mapping library (`Joy` → commands), no
  `rclpy`, unit-testable.

Use any X-input controller (fe. 8BitDo etc. in X-input mode). The
`/dev/input` bind mount propagates hot-plugged devices into the running
container.

## Hot-plug (`joy_publisher.py`)

Replaces `joy_node` because udev events don't propagate reliably into the
container. Unplug/replug at any time: the node closes the dead fd, polls
`/dev/input/` every `scan_period_s` (1 s), and re-opens — no ROS restart.
While the device is absent it publishes empty `Joy` at `autorepeat_rate`
(mapped to the safe all-zero state). Auto-discovers the lowest-numbered
`jsN` (Linux renumbers on replug); override with
`device_path:=/dev/input/jsN` when multiple controllers are attached.

## Configuration

Controller identity, per-mode bindings, and posture scalar limits all live
in [`config/teleop_joy.yaml`](config/teleop_joy.yaml) (documented inline;
override via `joy_config_file:=...`). The node starts in **gait** mode. The
loader validates every binding at startup — unknown names/keys,
button-vs-axis class mismatches, and cross-section conflicts all raise.

## Values owned elsewhere (SSoT in `hexa_description/config/tuning.yaml`)

- **Velocity caps** — via `hexa_common.load_velocity_caps`. Both are
  per-gait and both are re-scaled the moment a gait switch is accepted.
  Linear is isotropic (`stride_length × (1−β) / (min_swing_time × β)`);
  angular is that same cap divided by the outermost standing foot's planar
  radius, the lever arm a yaw rate acts through. There is no turn-rate knob
  — to change it, change `stride_length`, `min_swing_time`, the gait's duty
  factor, or the stance width (`standing_pose.tip_radius`). Because the
  angular cap comes from the stance, the loader reads `geometry.yaml`
  alongside `tuning.yaml`. Edit those, not teleop config.
- **Animation cycler list** — via
  `hexa_posture.load_animation_mode_animations`
  (`animation_mode_animations`). Adding an entry exposes it on the
  joystick with no teleop edit.
