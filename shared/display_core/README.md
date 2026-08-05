# `shared/display_core` — vendored sources

These are the third-party / firmware pieces of the **shared display core**
(`shared/display_core`): the platform-free eye animation + rasterizer and the
u8g2 C core. They are compiled directly by every consumer of the display core —
the `hexa_display` ROS node, the Pi Pico firmware, and the firmware host test —
so the eyes rasterize bit-identically across targets (the same relationship
`shared/motion_core` has for the locomotion brain). Nothing here is original to
this repo — it is copied verbatim from upstream. Do not edit these files by hand;
fix the upstream source and re-run `./sync.sh`.

## `core/` — firmware pieces (from `hexapod-esp32-display`)

Platform-free code shared with the ESP32 firmware. Kept in sync so the eye
animation and the on-wire expression/gaze enum names never diverge between the
Pi node and the microcontroller.

- `EyeAnim.{h,cpp}` — `components/renderer/`
- `EyeRaster.{h,cpp}` — `components/renderer/`
- `Expression.h` — `components/expression/include/`
- `ExpressionController.{h,cpp}` — `components/expression/`
- `IRenderer.h` — `components/expression/include/`
- `Config.h` — `components/config/include/`

Vendored at repo commit `803dce8`, with one local divergence: `EyeAnim`'s first
frame adopts the target expression outright instead of blinking through to it.
Upstream boots NEUTRAL and so never noticed; the Pi boots SLEEPY (no
`/gait/state` heard yet) and would otherwise flash NEUTRAL through the opening
blink. Port it upstream and re-sync to drop this note.

## `u8g2/` — u8g2 C core (from https://github.com/olikraus/u8g2)

The complete `csrc/` of the u8g2 submodule **except** the two pure font-data
blobs (`u8g2_fonts.c`, `u8x8_fonts.c`, ~40 MB combined) — the face draws no
text, so no font symbol is referenced and `--gc-sections` drops the unused
device drivers at link time.

Vendored at u8g2 commit `cbceaa1`.

## `fonts/` — text-mode font (generated here, not synced)

The display's text mode draws with **Pixel Operator**
(https://www.dafont.com/pixel-operator.font, CC0), a true pixel font with a
16 px native em. Unlike `core/` and `u8g2/` this directory is owned by this
repo, not `sync.sh`:

- `PixelOperator.ttf` — vendored source font (CC0, so redistribution is fine).
- `hexa_text_font.{c,h}` — the generated u8g2 font array
  (`u8g2_font_hexa_text_16`, ASCII + Latin-1), checked in.

Regenerate with `tools/gen_font.py` (host: needs Pillow + u8g2's `bdfconv`;
build recipe in the script docstring) after changing the TTF, pixel size, or
character set.

## Re-syncing

Run `./sync.sh` from a full repo checkout (with the u8g2 submodule
initialized). It re-copies every file above and prints the current provenance
commits so this README can be updated.
