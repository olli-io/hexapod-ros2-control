# Part 02 — Bluetooth teleop (Bluepad32)

**Goal:** pair a gamepad over Bluetooth and produce the raw `axes[]`/`buttons[]`
arrays in the exact layout `teleop_joy.yaml` expects — the drop-in replacement
for `joy_publisher.py`. Verified standalone by dumping input; consumed by part 07.

**Depends on:** 01. **Blocks:** 07 (and live-input testing of 05/06).

## Scope

- Vendor **Bluepad32** into the build (`PICO_BOARD=pico2_w`, its BTstack + `cyw43_arch` deps). Use the **bare Pico SDK C "platform" API** (`uni.h` — a custom `uni_platform` with callbacks), **not** the Arduino `BP32`/`ControllerPtr` API (that one needs the arduino-pico core, incompatible with part 01's `pico_sdk_import.cmake` scaffold). Supply `btstack_config.h`. Link `pico_cyw43_arch_threadsafe_background` so BTstack runs in the cyw43 background context and the cooperative loop stays clean.
- `src/bt_teleop.{hpp,cpp}`:
  - Init `cyw43_arch`, then `uni_platform_set_custom(...)` + `uni_init(...)`; register a `uni_platform` whose `on_device_connected` / `on_device_disconnected` / `on_device_ready` / `on_controller_data` callbacks receive a `uni_hid_device_t*` / `uni_controller_t*`.
  - Expose `void pump()` (service hook; a no-op in background mode, kept for symmetry / a future poll-mode build) and `bool read(int16_t axes[], uint32_t& buttons)` that copies the latest adapted snapshot (buttons as a bitmask, bit *i* = button index *i*).
  - **Adapter layer:** map Bluepad32 `uni_gamepad_t` state (`axis_x/y/rx/ry ≈ −512..512`, `buttons` bitmask, `dpad` bitmask, `brake`/`throttle 0..1023`) into the **X-input index layout + normalization** assumed by `hexa_teleop/config/teleop_joy.yaml` (`base.buttons{a,b,x,y,l1,r1,select,start}`, `base.axes{left/right stick x/y, l2, r2, dpad_x, dpad_y}`, axis scale `1/32767`, `axis_signs`). Reproduce that mapping so `map_joy` runs unchanged in part 07.
  - On no controller / disconnect: emit neutral axes + released buttons (safe idle). Note triggers rest at **+32767** (released), not 0 — `map_joy` reads `value < 0.5` as pressed.
- Reference for the target layout: `src/hexa_teleop/joy_publisher.py` (event struct, axis scaling) and `teleop_joy.yaml` (indices/signs).

## Done when / verification

- Power on, put a PS4/PS5 (or Xbox) controller in pairing mode; USB-CDC log shows "controller connected".
- Move sticks / press buttons → live dump of the adapted `axes[]`/`buttons[]` shows the correct index moving with the right sign and range.
- Cross-check a few bindings against `teleop_joy.yaml` (e.g. `start`=init, `y`=posture_mode, right stick Y = drive_x) so part 07 inherits a correct mapping.
- Disconnect the controller → arrays go neutral (proves the idle/failsafe input path).
