# Pico 2 W hexapod firmware

Bare-metal (Pico SDK) port of the ROS2 hexapod brain onto a single **Raspberry
Pi Pico 2 W (RP2350)**. Teleop over Bluetooth; gait, kinematics, control,
posture and battery telemetry all run onboard. The existing **Servo 2040** board
stays a dumb PWM slave over UART (Chica protocol).

See `../.tmp/plan/pi-pico-2-w/` for the full plan. This tree implements:

- **part 00** — shared project layout / conventions.
- **part 01** — skeleton & toolchain: USB-CDC stdio, `cyw43_arch` onboard-LED
  blink, boot banner + 1 Hz heartbeat, and a `time_us_64()` loop stub that later
  becomes the 200 Hz control scheduler.
- **part 02** — Bluetooth teleop (`bt_teleop.{hpp,cpp}`, `btstack_config.h`):
  Bluepad32 pairs a gamepad over the CYW43439 and adapts its state into the exact
  raw `axes[]`/`buttons[]` layout `hexa_teleop/config/teleop_joy.yaml` expects, so
  the ported `map_joy` (part 07) runs unchanged. The main loop dumps the adapted
  arrays over USB-CDC for verification.
- **part 03** — Servo 2040 slave link (`servo_out.{hpp,cpp}`): the Chica binary
  protocol over the Pico's hardware UART, driving the Servo 2040 as a dumb PWM
  slave. Forked from `hexa_hardware/src/{servo2040_protocol,joint_calibration}.cpp`
  (framing, run-grouping and pulse-width calibration reused verbatim, `double`→
  `float`); only the transport changes — the host `termios` UART becomes the Pico
  SDK UART. Carries servo SET frames, the rail relay, and the battery GET. The
  main loop drives a sine sweep on all 18 servos and polls the battery as the
  part-03 verification surface.

- **part 04** — shared foundations (`src/vec3.hpp`, `src/leg_index.hpp`,
  `tools/gen_config.py` → `src/config_generated.hpp`, `test/host/`): the float
  `Vec3` (drop-in for `Eigen::Vector3d`), the `Leg` enum + name tables, the
  build-time config generator that bakes the repo YAMLs into `constexpr` structs
  (so the filesystem-less firmware keeps `hexa_description` as the single source
  of truth), and the host-native test harness that unit-tests them off-target.

- **part 05** — kinematics (`src/kinematics/{leg_ik,body_transform}.{hpp,cpp}`):
  the FK/IK and body↔leg / `BodyPose` transforms, forked from
  `hexa_kinematics_cpp` (`double`→`float`, `namespace hexa`). Reuse
  `config::LegSpec` geometry (no duplication). The main loop composes one static
  standing stance through IK as the verification surface.

- **part 06** — gait engine (`src/gait/`): the full locomotion engine forked from
  `hexa_gait_cpp` (`double`→`float`, `namespace hexa::gait`) — phase clock,
  quartic-Bézier trajectory, the five gait strategies, the engagement / pause /
  stand-transition / reseat sub-controllers, velocity limits, and the
  FOLDED→INITIALIZE→STAND→ENGAGING→GAIT→PAUSING→PAUSED→RESUMING→RESEATING→FOLDING
  state machine. YAML loaders are replaced by `*_from_config` builders over the
  baked `config_generated.hpp`. (Part 06 drove the engine from a hardcoded demo
  cmd_vel schedule; part 07 replaces that with live teleop.)

- **part 07** — control shaping & teleop mapping (`src/joy_mapping.{hpp,cpp}`,
  `src/control.{hpp,cpp}`): closes the loop from the Bluetooth gamepad to
  walking. `joy_mapping` forks `hexa_teleop/joy_mapping.py` (`double`→`float`,
  `namespace hexa::teleop`) — the mode FSM, eased yaw/wiggle, persistent height,
  two-press init/revert, record folding, and the gait/animation cyclers — driven
  straight off the `bt_teleop` snapshot. `control` forks
  `hexa_control`'s `body_velocity_limiter.py` + `control_node.py`
  (`namespace hexa::control`): `scale_to_envelope` cuts the command to the active
  gait's per-leg foot-speed envelope, then the constant-max-accel
  `BodyVelocityLimiter` slews toward it. The per-mode function→key bindings are
  pre-resolved into `JoyKeyRef` tables in `config_generated.hpp` (no runtime
  string handling). Arbitration (`/teleop/owner`) is dropped — the firmware
  always owns the sole gamepad. The main loop now maps the pad → shapes velocity
  → drives the engine; the init button (not a schedule) triggers the stand, and
  gait-cycle / animation buttons route into the engine gated by its state. Body
  pose (posture) is wired into IK in part 08.

- **part 08** — posture (`src/posture/{pose,animations,posture}.{hpp,cpp}`): the
  body-pose chain. `pose` forks `hexa_posture/pose.py` (the `add`/`scale`/`clamp`
  small-offset algebra over the kinematics `BodyPose`); `animations` forks the
  seven pure animations + the `Stack` (`Still`, `Breathing`, `GaitSway`,
  `GaitBounce`, `VerticalBodyRoll`, `HorizontalBodyRoll`, `BodyRoll3D`);
  `posture::PostureController` forks `posture_node.py` (`namespace
  hexa::posture`) — it derives the support-centroid and swing-lift signals from
  the engine's per-leg output (no topics), low-pass-filters them, gates the
  whole stack on `POSTURE_ACTIVE_STATES`, and composes `clamp(user_pose +
  animated)` into the `body_pose_target`. The main loop now feeds map_joy's pose
  output as the user pose, routes `/animation/mode` selections into the stack,
  and passes the result into the compose step, which re-expresses each foot
  target through `apply_body_pose` before `body_to_leg` → IK (the exact
  `ik_node.cpp` ordering). This completes full parity minus the LED face.

- **part 09** — integration, telemetry & failsafes (`src/supervisor.{hpp,cpp}`):
  the safety / health coordinator that turns the working chain into a robust
  robot. `supervisor` is **pure** (no Pico SDK) so it unit-tests off-target like
  the rest of the port; each tick main feeds it the BT-link freshness, engine
  posture and a rate-limited battery sample, and applies the decisions it
  returns. It folds four ROS-side mechanisms into one onboard unit:
  - **input watchdog** — a stale/lost BT link (no fresh gamepad frame within
    `webteleop safety.input_timeout_s` = 0.5 s) zeroes `cmd_vel` before shaping,
    so the engine settles (pauses → reseats) rather than latching stale velocity;
  - **battery monitor** — a float port of `hexa_display`'s hysteresis-debounced
    `BatteryMonitor` (thresholds from `display.yaml`; shipped disabled — the
    divider scale is uncalibrated), off a Chica GET rate-limited to every
    `hardware.yaml get_period_ticks` control ticks;
  - **relay-arming discipline** — the servo rail energizes only after link-up +
    a completed stand, and drops on a clean fold or a critical battery (a stale
    link does **not** drop it — the robot holds its stand and settles);
  - **status LED** — the onboard LED stands in for the dropped `hexa_display`
    face: slow blink = idle / stand / scanning, solid = linked + walking, fast
    blink = fault (low/critical battery or a link lost mid-operation).

  The remaining two failsafes were already in place: the **IK guard** (hold
  last-good angle on `UnreachableTarget`, part 05) and the **boot-FOLDED**
  startup that runs `InitializeController` before any gait (part 06). The
  heartbeat also logs **tick jitter** (measured inter-tick spread vs the 5 ms
  budget + deadline overruns) and the **free-heap** low-water mark (the
  `std::map`-keyed engine's fragmentation drift over a soak).

- **part 10** — Linux build & simulator testing (`src/pipeline.{hpp,cpp}`,
  `test/host/test_pipeline.cpp`, and the `hexa_pico_bridge` ROS2 package). The
  desktop test loop, with **no hardware**. Two things land here:
  - **The portability seam (Tier 2).** The whole 50 Hz control brain — teleop
    mapping → velocity shaping → gait → posture → compose/IK, plus the
    supervisor — is factored out of `main.cpp` into the target-agnostic
    `hexa::pipeline::Pipeline` (no Pico SDK, no ROS, no I/O). The caller owns the
    hardware seam — input (`bt_teleop` / a scripted source / a ROS `/joy`
    bridge), servo out (the Chica UART / a sim publisher), and the clock
    (`time_us_64` / `std::chrono` / a ROS clock) — and each tick feeds a
    `TickInput` and applies the `TickResult`. `main.cpp` is now the **Pico**
    composition of that seam; the tick logic (every line that could carry a port
    bug) is shared source, a **link-time swap, not `#ifdef` soup**.
  - **Gazebo-in-the-loop (Tier 3).** `hexa_pico_bridge` (`src/hexa_pico_bridge`,
    an `ament_cmake` + `rclcpp` package) links the pipeline sources directly and
    runs the real tick against the simulated hexapod: it subscribes `/joy`,
    converts it to the firmware's raw int16 layout, and publishes the
    `JointAngles` (radians) on `/joint_group_position_controller/commands` — so
    the firmware brain **walks the Gazebo model** (sim-first). See that package's
    README.

  Tier 1 (the host golden-trace + unit suites the earlier parts grew) is
  formalized here and extended with `test_pipeline`, which drives the whole
  composed brain off-target (stand → walk → safe-stop). The optional Tier 4
  (a Wokwi / Renode firmware-binary smoke sim) is **deferred** — it needs the
  ARM build and can't simulate the CYW43 Bluetooth radio.

This completes the plan. The remaining work — cutover of the ROS2 workspace to
the C++ ports, on-target bring-up (flashing, servo wiring), and the optional
Tier 4 binary sim — is tracked outside this tree.

## Shared foundations & config codegen (part 04)

The pure port runs entirely in `float` (RP2350 single-precision FPU). `Vec3`
(`src/vec3.hpp`) replaces `Eigen::Vector3d`; `JointAngles` is
`std::array<float,3>`. `src/leg_index.hpp` fixes the six-leg order (`Leg` enum +
`LEG_NAMES`, matching the ROS2 libs).

`tools/gen_config.py` reads the repo's runtime YAMLs and emits
`src/config_generated.hpp` — `constexpr` mirrors of the leg geometry (with the
six-leg mount symmetry expansion), joint limits, standing / initial pose, the
gait `EngineConfig`, the derived per-gait velocity caps, teleop joystick config,
the posture animation stack, control ramps, and the 18 servo calibrations. Its
transforms are ports of the ROS2 loaders (`description_loader.cpp`,
`limits.cpp` + `gaits/registry.cpp`, `joint_calibration.cpp`) and must stay in
lockstep with them. CMake runs it pre-build (both the Pico build here and the
host harness); the generated header is git-ignored (regenerated, not committed).
`hexa_description` and the other config packages remain the single source of
truth for the numbers. Regenerate manually with `python3 tools/gen_config.py`.

Host-native unit tests (`vec3`, generated-config spot-checks) live in
`test/host/` — see `test/host/README.md`. They are the seed of the golden-trace
regression gate that parts 05–08 grow.

## Bluepad32 (part 02)

Uses Bluepad32's **bare Pico SDK C "platform" API** (`uni.h` — a custom
`uni_platform` with callbacks), **not** the Arduino `BP32`/`ControllerPtr` API
(that needs the arduino-pico core, incompatible with this `pico_sdk_import.cmake`
project). Vendor it as a git submodule, or point `BLUEPAD32_ROOT` at a checkout:

```sh
git submodule add https://github.com/ricardoquesada/bluepad32 external/bluepad32
# or: export BLUEPAD32_ROOT=/path/to/bluepad32
```

Needs a Bluepad32 with RP2350 / Pico 2 W support. `btstack_config.h` (firmware
root) configures the BTstack HID-host + flow-control profile; keep it in sync
with the vendored Bluepad32 / SDK BTstack version if pairing regresses. BTstack
runs in the background (`pico_cyw43_arch_threadsafe_background`); the cooperative
loop just calls `bt_teleop::read()` each tick.

## Servo 2040 link (part 03)

The Servo 2040 stays a dumb PWM slave. `servo_out` speaks the existing **Chica**
protocol over **uart0 (GP0 TX / GP1 RX) at 921600 8N1** — the pin pair CMakeLists
reserves (USB-CDC carries stdio, so UART stdio is off; no contention). Everything
rides that one UART as Chica frames:

- **SET** — servo pulse widths (µs). Consecutive pins collapse into one SET frame
  per run (`command_all`); the current wiring (pins 1..18) is a single 18-value
  frame, ~39 bytes / ~0.43 ms at 921600.
- **SET** — the servo-rail relay (Servo 2040 digital pin 24). `set_relay(true)`
  energizes the rail. Part 09 gates this on link-up + stand; part 03 energizes at
  boot so the bench sweep is observable.
- **GET** — battery voltage/current from the Servo 2040 aux ADC pins (26/27,
  scales `0.00366` V and `0.00098` A per 14-bit count).

Pin map and calibration are hardcoded from `hexa_hardware/config/hardware.yaml`
for now; **part 04** regenerates them into `config_generated.hpp` from the repo
YAMLs. No PWM code runs on the Pico.

> **Verify the Servo 2040 firmware speaks Chica over its hardware UART** (not only
> USB-CDC) and adjust `kBaud` / the GPIO in `servo_out.cpp` to match if pairing
> fails. 921600 8N1 is the assumed default.

## Toolchain

- **Pico SDK** pinned to **2.1.1** (RP2350 + Bluepad32 require >= 2.1.0).
  Provide it one of two ways:
  - export `PICO_SDK_PATH` pointing at a local checkout of the SDK, **or**
  - let CMake fetch the pinned tag from git: `-DPICO_SDK_FETCH_FROM_GIT=ON`.
- **ARM GCC** — `arm-none-eabi-gcc` (the SDK-recommended toolchain; Arch:
  `arm-none-eabi-gcc` + `arm-none-eabi-newlib`).
- **CMake** >= 3.13, plus `picotool` for flashing over USB.

## Build

```sh
cd pi-pico-2-w-firmware
cmake -B build -DPICO_BOARD=pico2_w    # add -DPICO_SDK_FETCH_FROM_GIT=ON if no PICO_SDK_PATH
cmake --build build -j
```

Produces `build/hexa_pico.uf2` (plus `.elf`/`.bin`/`.map`).

## Flash

- **BOOTSEL / UF2:** hold BOOTSEL while plugging in the Pico 2 W, then copy
  `build/hexa_pico.uf2` onto the `RPI-RP2` mass-storage volume.
- **picotool:** `picotool load -f build/hexa_pico.uf2` (auto-reboots into
  BOOTSEL if the running firmware supports it).

## Verify (part 01)

- `cmake`/build produce `hexa_pico.uf2` with no errors.
- After flashing, the onboard LED blinks at ~1 Hz.
- Open the USB-CDC serial port (`minicom -D /dev/ttyACM0`, or `screen`) and
  observe the boot banner followed by an incrementing `[heartbeat]` counter.

This confirms the toolchain, `cyw43_arch` init (prerequisite for Bluetooth in
part 02), USB stdio logging, and the loop timer all work.

## Verify (part 02)

With the USB-CDC serial port open:

- Power on and put a PS4/PS5/Xbox pad in pairing mode — the log shows
  `[bt] init complete — scanning`, then `[bt] controller ready` once it pairs.
- Move sticks / press buttons and watch the `[joy]` dump (4 Hz). Each index
  should move with the right sign and range (`±32767`): e.g. right stick Y
  (`RY`) drives `drive_x`, `start` (bit 7 of `btns`) is `init`, `y` (bit 3) is
  `posture_mode` — cross-check against `teleop_joy.yaml`'s `base` block.
- Triggers (`L2`/`R2`) rest at `+32767` and fall toward `-32767` when pressed
  (the joy_node convention `map_joy` reads as pressed below `0.5`).
- Disconnect the pad — the dump switches to `idle` with neutral axes (sticks/dpad
  `0`, triggers back at `+32767`) and no buttons, proving the failsafe idle path.

## Verify (part 03)

Wire the Pico's GP0→Servo 2040 UART RX, GP1←Servo 2040 UART TX, and share ground
(servo power stays on the Servo 2040 rail). With the USB-CDC serial port open:

- **Sweep** — the boot banner logs `servo: Chica UART up ...`, then all 18 servos
  sweep ±0.30 rad about center on a 4 s sine. Command one pin at 1500 µs (center)
  and 1000/2000 µs to confirm symmetric travel.
- **Relay** — the rail energizes at boot (`rail relay energized` in the banner);
  measure/observe the servo rail power up. Comment out `set_relay(true)` to see it
  stay dropped.
- **Battery** — the 1 Hz `[servo]` line prints decoded `batt=<V> <A>`; cross-check
  against a multimeter. `batt=-- (no reply)` means the GET timed out (link/baud).
- **Timing** — the same `[servo]` line reports `SET burst` and `GET rt` in µs;
  confirm both sit well inside the 5 ms tick budget.

Host-side, the Chica framing + float calibration + run-grouping are checked
independent of hardware (center → 1500 µs, ±π/4 → 2000/1000 µs, clamps, framing
byte layout).

## Verify (part 07)

Bench the robot on a stand (feet clear) with a paired pad and the USB-CDC serial
port open:

- **Stand** — press the **init** button (`start`); the log shows
  `[teleop] init: FOLDED -> INITIALIZE` and the legs run the placement/lift
  ladder to STAND. Press again in STAND to fold back.
- **Drive** — once standing, the sticks drive: right stick forward/back →
  `linear_x`, right stick left/right → `linear_y`, left stick left/right →
  `angular_z`, each with the sign and scaling of the active gait's cap. Release
  the sticks and the body **ramps** smoothly to a stop (constant-max-accel slew,
  not an instant cut) — the engine then debounces → PAUSING → RESEATING → STAND.
- **Gait cycle** — D-pad left/right steps tripod → tetrapod → ripple (surf/crawl
  are unstable and filtered out); the `[teleop] gait -> …` line prints the new
  stick cap. Switches are dropped (logged) when the engine can't accept one.
- **Envelope** — combined max forward + max yaw stays inside the per-leg
  foot-speed envelope (`scale_to_envelope`), so no leg saturates or jitters.
- **Failsafe** — disconnect the pad: `bt_teleop::read` returns the neutral
  snapshot, `map_joy` yields zero velocity, and the engine pauses/settles.

Host-side, `map_joy` is checked frame-for-frame against the untouched Python
`hexa_teleop.joy_mapping` reference over a scripted axes/buttons trace
(`test/host/test_joy_mapping.cpp` + `gen_joy_golden.py`), and the
`BodyVelocityLimiter` slew + `Control` scale/reset wiring have unit tests
(`test/host/test_control.cpp`).

## Verify (part 09)

Bench the robot on a stand (feet clear) with a paired pad and the USB-CDC serial
port open. The boot banner now logs `rail DROPPED (arms on link-up + stand)` and
a `safety:` line (input timeout, GET period, free heap).

- **Relay discipline** — the rail stays dropped through pairing and the
  INITIALIZE ladder; press **init** and the `[safety] relay ENERGIZED` line
  fires only once the engine reaches STAND. Press init again to fold: at FOLDED
  the log shows `[safety] relay dropped`.
- **Input watchdog** — walk the pad out of BT range (or power it off) mid-gait:
  within ~0.5 s the command zeroes, the engine pauses → reseats → settles to a
  stand (the rail stays energized so it doesn't collapse), and the status LED
  switches from **solid** to **fast blink** (link lost). Walk back into range and
  it resumes.
- **Status LED** — slow blink while idle / standing / scanning, solid while
  walking, fast blink on a fault (link lost, or a low/critical battery once real
  thresholds are set in `display.yaml`).
- **Battery** — the 1 Hz `[servo]` line still prints decoded `batt=<V> <A>`
  (GET now runs every `get_period_ticks` control ticks); cross-check against a
  multimeter across a discharge. Set `battery_warning_v`/`battery_critical_v` in
  `display.yaml` to arm the low-battery path (0 disables — shipped default).
- **Scheduler & heap** — the 1 Hz `[safety]` line reports tick `jitter`
  (`last/min/max` interval, `over/total` deadline overruns) and `heap` free +
  low-water mark. Over a multi-hour soak the overrun count and heap floor should
  hold steady; sustained overruns or a drifting heap floor are the trigger for
  the `std::map`→`std::array<T,6>` refactor flagged in the overview.

Host-side, the whole failsafe / telemetry / status-LED policy is unit-tested
off-target — the battery hysteresis debounce, the input watchdog, the
relay-arming state machine, the LED mapping and the jitter accounting
(`test/host/test_supervisor.cpp`, pure `supervisor.cpp`, no hardware).

## Verify (part 10) — no hardware

Everything below runs inside the sim container (`./hexa sim`); no ARM toolchain
or physical board is needed.

- **Host harness (Tier 1/2)** — the whole port's logic, natively:

  ```sh
  cd pi-pico-firmware/test/host
  cmake -S . -B build && cmake --build build -j
  ctest --test-dir build --output-on-failure
  ```

  All suites pass, including `test_pipeline`, which drives the composed
  `hexa::pipeline::Pipeline` (the exact source the firmware and the bridge
  compile) from FOLDED → STAND → walking → lost-link safe-stop.

- **Gazebo-in-the-loop (Tier 3)** — the firmware brain walking the sim:

  ```sh
  colcon build --packages-select hexa_pico_bridge
  ros2 launch hexa_pico_bridge bridge.launch.py
  ```

  On a paired pad: **init** (start) stands the robot up, the sticks walk it, and
  pose mode tilts the body with feet planted — motion should match the ROS2 node
  chain (`ik_node` + `gait_node` + `posture_node`) in the same world. Headless,
  the seam is checked end-to-end by feeding `/joy` and watching
  `/joint_group_position_controller/commands`: neutral input holds the folded
  pose at a steady 200 Hz, and an init press drives the 18 joints up to the
  standing pose. See `src/hexa_pico_bridge/README.md`.
