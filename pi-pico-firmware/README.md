# Pico 2 W hexapod firmware

Bare-metal (Pico SDK) port of the hexapod brain onto a single **Raspberry Pi
Pico 2 W (RP2350)**. Teleop over Bluetooth; the whole velocity → gait → posture
→ compose/IK pipeline plus failsafes run onboard at 200 Hz. The **Servo 2040**
board stays a dumb PWM slave over UART (Chica protocol).

## Flash

- **BOOTSEL / UF2:** hold BOOTSEL while plugging in the Pico 2 W, then copy
  `build/hexa_pico.uf2` onto the `RPI-RP2` mass-storage volume.
- **picotool:** `picotool load -f build/hexa_pico.uf2`.

## Build

The Pico is a deploy target (its artifact is the `.uf2`), built in the sim
container where the ARM toolchain already lives:

```sh
./hexa deploy --pico          # cmake configure + build; extra args -> cmake --build
```

Produces `pi-pico-firmware/build/hexa_pico.uf2` (plus `.elf`/`.bin`/`.map`) on
the host — the tree is bind-mounted. `hexa deploy --pico` runs a one-shot
`compose run --build` container and never launches the ROS2 sim stack, so
building firmware is fully separate from `hexa sim up`.

## Architecture

The control brain is **not** in this tree — it lives in `../shared/motion_core`
(`hexa::pipeline::Pipeline` + `hexa::gait`/`posture`/`control`/`supervisor`), the
target-agnostic `float` core shared bug-for-bug with `hexa_locomotion` and
`hexa_pico_bridge`. This tree is only the **Pico hardware seam** compiled around
it — a link-time swap, no `#ifdef`:

- **`src/main.cpp`** — the Pico composition. Owns the clock (`time_us_64`), the
  onboard LED, and USB-CDC logging. Each 200 Hz tick it samples the seam into a
  `TickInput`, runs `Pipeline::tick()`, and applies the `TickResult` (commands
  the 18 joints, drives the relay + status LED). Cold-starts FOLDED and holds
  the folded pose until the init button, so a paired pad alone never walks the
  robot; an unpaired pad leaves the rail open entirely.
- **`src/bt_teleop.{hpp,cpp}`** — Bluetooth gamepad input via Bluepad32/BTstack,
  adapted into the raw `axes[]`/`buttons[]` layout `map_joy` expects. Also owns
  the onboard-LED write (the CYW43 lives on core1 with BTstack).
- **`src/servo_out.{hpp,cpp}`** — the Servo 2040 link: Chica protocol over the
  Pico's hardware UART. Carries servo SET frames, the rail relay, and the
  battery GET.

The two RP2350 cores split: **core0** runs the cooperative 200 Hz control loop;
**core1** runs cyw43/BTstack (Bluepad32) in the background.

> There is no face on the Pico — the SH1122 OLED eyes run only on the Pi-side
> `hexa_display` node. The status LED (slow blink = idle/stand, solid = walking,
> fast blink = fault) stands in for it here.

## Config codegen

The Pico has no filesystem, so the repo's runtime YAMLs are baked into
`../shared/motion_core/config_generated.hpp` by `../shared/motion_core/tools/gen_config.py`
before the build — `constexpr` mirrors of leg geometry, joint limits, poses, the
gait/posture/control config, and the 18 servo calibrations. `hexa_description`
stays the single source of truth. CMake regenerates it whenever the generator or
a source YAML changes (the header is git-ignored). Regenerate by hand with
`python3 ../shared/motion_core/tools/gen_config.py`.

## Build

The Pico is a deploy target (its artifact is the `.uf2`), built in the sim
container where the ARM toolchain already lives:

```sh
./hexa deploy --pico          # cmake configure + build; extra args -> cmake --build
```

Produces `pi-pico-firmware/build/hexa_pico.uf2` (plus `.elf`/`.bin`/`.map`) on
the host — the tree is bind-mounted. `hexa deploy --pico` runs a one-shot
`compose run --build` container and never launches the ROS2 sim stack, so
building firmware is fully separate from `hexa sim up`.

## Config codegen

The Pico has no filesystem, so the repo's runtime YAMLs are baked into
`../shared/motion_core/config_generated.hpp` by `../shared/motion_core/tools/gen_config.py`
before the build — `constexpr` mirrors of leg geometry, joint limits, poses, the
gait/posture/control config, and the 18 servo calibrations. `hexa_description`
stays the single source of truth. CMake regenerates it whenever the generator or
a source YAML changes (the header is git-ignored). Regenerate by hand with
`python3 ../shared/motion_core/tools/gen_config.py`.

## Toolchain

The `hexa-sim` image bakes the whole firmware toolchain (see `sim.Dockerfile`) —
nothing to install on the host:

- **Pico SDK** 2.1.1 (RP2350 + Bluepad32 need ≥ 2.1.0) at `/opt/pico-sdk`
  (`PICO_SDK_PATH`).
- **Bluepad32** 4.2.0 (first release with Pico 2 W / RP2350 support) at
  `/opt/bluepad32` (`BLUEPAD32_ROOT`). The firmware's `btstack_config.h` +
  `sdkconfig.h` are the version sync point; keep them in step if pairing
  regresses.
- **ARM GCC** (`arm-none-eabi-gcc`), CMake ≥ 3.13, and **picotool**.

To build on a bare host, provide `PICO_SDK_PATH` (or
`-DPICO_SDK_FETCH_FROM_GIT=ON`) and `BLUEPAD32_ROOT` yourself.

Bluepad32 uses the bare Pico SDK C "platform" API (`uni.h`), **not** the Arduino
`BP32` API. BTstack comes from the Pico SDK (no `bluepad32_import.cmake`); the
component is pulled in with `add_subdirectory`. The firmware links
`pico_cyw43_arch_none` (BT-only, `CYW43_LWIP=0`), which still runs the
threadsafe_background async context that services the BTstack loop.

## Servo 2040 link

`servo_out` speaks the **Chica** protocol over **uart0 (GP0 TX / GP1 RX) at
921600 8N1** (USB-CDC carries stdio, so UART stdio is off — no contention):

- **SET** — servo pulse widths (µs); consecutive pins collapse into one frame
  (the 1..18 wiring is a single 18-value frame, ~0.43 ms). During the energize
  sweep below the frames are per-leg instead, one leg at a time.
- **SET** — the servo-rail relay (Servo 2040 digital pin 24). The supervisor
  energizes it on link-up in any engine state but FAULT — so normally as soon as
  a pad pairs, while the engine is still FOLDED — and drops it on a completed
  fold, a critical battery, or an over-current trip. Closing the relay leaves
  every servo **limp**; `command_legs` then drives the legs one at a time,
  `init.sweep_leg_interval_ms` apart in pin order, so the inrush arrives as six
  small steps instead of one spike (`hexa::EnergizeSweep`, shared with the ROS
  `hexa_hardware` plugin).
- **GET** — battery voltage/current from the Servo 2040 aux ADC pins (26/27).

Pin map and calibration are baked from `hexa_description/config/hardware.yaml` +
`servo_calibration.yaml`. No PWM code runs on the Pico.

> Verify the Servo 2040 firmware speaks Chica over its **hardware UART** (not
> only USB-CDC); adjust `kBaud` / GPIO in `servo_out.cpp` if the link fails.

## Test & verify

Host-native tests (no ARM toolchain, no board) — the exact `motion_core` source
the firmware compiles, plus the Pico seams' host harness:

```sh
cd ../shared/motion_core/test
cmake -S . -B build && cmake --build build -j
ctest --test-dir build --output-on-failure
```

`test_pipeline` drives the composed brain FOLDED → STAND → walking → lost-link
safe-stop; `test_joy_mapping` checks `map_joy` frame-for-frame against the Python
reference; `test_supervisor` covers the failsafes/telemetry/LED policy.

On-target, open the USB-CDC serial port (`minicom -D /dev/ttyACM0`) — the boot
banner, a 1 Hz `[heartbeat]`, the `[servo]`/`[gait]`/`[safety]` telemetry lines,
and a 4 Hz `[joy]` dump confirm the toolchain, pairing, servo link, and control
loop. Bench the robot on a stand with feet clear before flashing — the init
button (`start`) stands it up, then the sticks walk it.

USB-CDC logging is on by default; build `-DHEXA_ENABLE_USB_DEBUGGING=OFF` for
on-robot / low-jitter runs so a blocking CDC write can't stall the tick.
