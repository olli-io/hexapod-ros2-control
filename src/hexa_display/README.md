# hexa_display

Face relay. Maps the robot's state (gait engine state, `cmd_vel`,
posture, battery) into expression and gaze commands for the ESP32 OLED
face and relays them over UART.

Pure sink in the dependency graph: it only subscribes to topics from
the existing chains; nothing imports it or subscribes to it. The
firmware (separate repo, `hexapod-esp32-display`) animates blinks and
gaze easing autonomously — this node only sets target state, at a
relaxed 10 Hz.

## Layers

Mirrors the library/node split used by `hexa_kinematics` and
`hexa_posture`:

- **Library** (`hexa_display/`) — pure Python, no ROS or pyserial
  imports required.
  - `protocol.py` — frame codec mirroring the firmware byte for byte:
    SOF `0xA5`, LEN u16 little-endian, CMD u8, payload, CRC-16/
    CCITT-FALSE big-endian over LEN+CMD+PAYLOAD. Encoders for
    `SET_EXPRESSION` / `SET_GAZE` / `TRIGGER_BLINK` / `PING`, and a
    stateless `decode_frames` scanner that resyncs on corruption.
  - `expression_policy.py` — pure `decide(inputs, config, prev)`
    policy, face-animation selection, plus the stateful
    `BatteryMonitor` debouncer.
  - `face_animation.py` — looping face-animation step sequences
    (breathing, idling); the node owns the clock and asks `due_steps`
    what to relay each tick.
  - `transport.py` — `Transport` ABC with `SerialTransport` (pyserial,
    imported lazily) and `StubTransport` (decodes and logs frames; used
    in sim and tests).
- **Node** (`display_node.py`) — ROS glue: caches latest topic
  samples, runs the policy on a timer, relays frames with change
  detection plus a periodic full refresh so an ESP32 reboot resyncs.

## Expression policy

Precedence, highest first:

- **battery critical** — `dead`, gaze centered, unconditional.
- **battery warning** — `sleepy`, only while idle (zero `cmd_vel`,
  gait state folded/stand/paused, no animation mode) so the warning
  never masks the face mid-walk. Both battery thresholds ship at 0.0
  (disabled) until the ADC voltage divider is calibrated.
- **animation mode** — `woozy` while a posture animation is active.
- **gait-state map** — YAML-configured expression per canonical gait
  state (defaults: `gait` → happy, `folded`/`paused`/`folding` →
  sleepy, everything else neutral).

Gaze:

- **vertical** — always follows body pitch from `/body/pose`: nose up
  → up. Driving forward or backward never moves the gaze up or down.
- **horizontal, gait-active** — follows `cmd_vel`: REP-103 left
  (`+vy`, `+wz`) → left. Each axis is normalized by a configured cap
  and sign-quantized with enter/exit hysteresis so the gaze does not
  chatter at the deadband.
- **horizontal, pose mode** — follows body tilt from `/body/pose`:
  yaw left / roll left → left.
- `dead` always forces gaze center.

## Face animations

Looping gaze/blink step sequences (`face_animation.py`) relayed by the
node; the firmware still eases gaze and auto-blinks on top. While one
is active it owns the gaze; the policy gaze resumes when it ends.
Distinct from the posture animation stack in `hexa_posture` — these
only drive the display.

- **breathing** — slow vertical gaze drift (up → center → down →
  center, 4.8 s period) while no `/gait/state` has been heard yet,
  i.e. the robot stack (servo UART, gait engine) is still
  initializing.
- **idling** — look-around-and-blink burst (3.04 s: left, blink,
  right, up, down, center, blink) once the robot has stood idle,
  level, and command-free for `idling_start_delay_s`. Bursts are
  spaced a random 5-10 s apart (`repeat_range_s`); the eyes rest at
  center in between rather than scanning continuously.

Battery warning/critical, a posture animation mode, any `cmd_vel`, or
a tilted body pose suppress the animations.

## Topics

- Subscribes:
  - `/gait/state` (`std_msgs/String`) — gait engine state name.
  - `/cmd_vel` (`geometry_msgs/Twist`) — horizontal gaze source while
    walking.
  - `/body/pose` (`hexa_interfaces/BodyPose`) — vertical gaze source
    (pitch); horizontal source in pose mode (yaw/roll).
  - `/animation/mode` (`std_msgs/String`, transient_local depth 1,
    matching the teleop publisher) — posture animation selection.
  - battery topic (`sensor_msgs/BatteryState`, sensor-data QoS,
    default `/hexa_hardware_aux/battery_state`) — real robot only.
- Publishes: nothing.

## Transport

- **serial** — `/dev/serial0` at 921600 baud (Pi PL011 UART header;
  see `docs/robot-environment.md` for the Pi config and container
  device mapping). Fire-and-forget TX, passive RX: NACK and firmware
  LOG frames are logged at warn, ACK at debug, PONG feeds the
  heartbeat monitor. On failure the
  node keeps running faceless and retries the port every
  `reconnect_period_s`, pushing full state on reconnect.
- **heartbeat** — the firmware counts any valid frame as link
  activity and falls back to a DEAD face after 3 s of silence
  (`LINK_TIMEOUT_MS` in the firmware config). Whenever nothing else
  has been written for `ping_period_s` (default 1 s) — idle, or
  steady-state walking where change detection sends nothing between
  refreshes — the node sends a PING to keep the link alive. The
  firmware echoes each PING back as a PONG; if none arrives within
  `pong_timeout_s` the node logs a ROS error (throttled) and an info
  line once the display responds again. The stub transport answers
  PINGs with PONGs and does not log them.
- **stub** — sim default: decodes outgoing frames and logs
  `display: SET_EXPRESSION HAPPY` style lines instead.

Note: `hardware.yaml` comments mention `/dev/ttyAMA0` as a possible
future servo UART transport — that is the same PL011 the display uses;
the two cannot share it.

## Configuration

All knobs in `config/display.yaml`: transport selection, serial
device/baud, update/refresh/reconnect/ping periods, the per-gait-state
expression map, animation/battery expressions, battery thresholds and
debounce, the gaze deadband/hysteresis/normalization caps, and the
idling start delay. Expression names are validated against the
protocol enum at startup.

Setting `enabled: false` in the same file makes the bringup launch
files (`sim.launch.py`, `robot.launch.py`) skip the display node
entirely — the rest of the stack is unaffected.

## Tests

`pytest src/hexa_display/test` — protocol byte-format vectors and
decoder resync, full policy precedence and gaze quantization table,
face-animation selection and step timing, battery debounce, and
transport behaviour with fakes.
