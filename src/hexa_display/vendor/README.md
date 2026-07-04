# Vendored sources

This directory makes the `hexa_display` package **self-contained**: it can be
built and shipped without a checkout of the surrounding firmware repo. Nothing
here is original to the package — it is copied verbatim from upstream. Do not
edit these files by hand; fix the upstream source and re-run `./sync.sh`.

## `core/` — firmware pieces (from `hexapod-esp32-display`)

Platform-free code shared with the ESP32 firmware. Kept in sync so the eye
animation and the on-wire expression/gaze enum names never diverge between the
Pi node and the microcontroller.

| Vendored file              | Upstream path                                       |
| -------------------------- | --------------------------------------------------- |
| `EyeAnim.{h,cpp}`          | `components/renderer/`                               |
| `EyeRaster.{h,cpp}`        | `components/renderer/`                               |
| `Expression.h`             | `components/expression/include/`                    |
| `ExpressionController.{h,cpp}` | `components/expression/`                         |
| `IRenderer.h`              | `components/expression/include/`                    |
| `Config.h`                 | `components/config/include/`                         |

Vendored at repo commit `803dce8`.

## `u8g2/` — u8g2 C core (from https://github.com/olikraus/u8g2)

The complete `csrc/` of the u8g2 submodule **except** the two pure font-data
blobs (`u8g2_fonts.c`, `u8x8_fonts.c`, ~40 MB combined) — the face draws no
text, so no font symbol is referenced and `--gc-sections` drops the unused
device drivers at link time.

Vendored at u8g2 commit `cbceaa1`.

## Re-syncing

Run `./sync.sh` from a full repo checkout (with the u8g2 submodule
initialized). It re-copies every file above and prints the current provenance
commits so this README can be updated.
