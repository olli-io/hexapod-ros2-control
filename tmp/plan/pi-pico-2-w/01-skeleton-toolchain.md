# Part 01 — Skeleton & toolchain

**Goal:** a `firmware/pico/` Pico SDK project that builds for the Pico 2 W,
flashes, and gives a working debug channel. Foundation for everything else.

**Depends on:** nothing. **Blocks:** all parts.

## Scope

- Install/pin **Pico SDK ≥ 2.1** (required for RP2350 + Bluepad32) and the ARM GCC toolchain. Document the exact SDK version.
- `firmware/pico/CMakeLists.txt`: `set(PICO_BOARD pico2_w)`, C++20, `pico_sdk_init()`, `-fexceptions`, and the flash/uart2/float link libs. Add `pico_sdk_import.cmake`.
- `src/main.cpp`: init `stdio` over USB-CDC, blink the onboard LED (via `cyw43_arch` GPIO, since on Pico W-family the LED is on the wireless chip), and print a boot banner + a 1 Hz heartbeat counter.
- Wire a `time_us_64()`-based loop stub that will later become the 50 Hz scheduler (for now just drives the heartbeat).
- README snippet: build command (`cmake -DPICO_BOARD=pico2_w ..`), and flash steps (BOOTSEL → UF2, or `picotool load`).

## Done when / verification

- `cmake` + `make` produce a `.uf2` with no errors.
- Flashing it: onboard LED blinks at ~1 Hz.
- `minicom`/`screen` on the USB-CDC port shows the boot banner and incrementing heartbeat.
- Confirms toolchain, `cyw43_arch` init (needed for BT in part 02), USB stdio logging, and the loop timer all work.
