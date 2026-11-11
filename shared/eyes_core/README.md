# `shared/eyes_core` — vendored sources

The greyscale eye renderer and SH1122 driver of **eye-animation-cpp**, copied
verbatim. They are compiled directly by the `hexa_eyes` package. Nothing here is
original to this repo: fix the upstream source and re-run `./sync.sh`. Do not
edit these files by hand.

This is a separate renderer from `shared/display_core`. That one is 1-bit and is
shared with the Pi Pico firmware; this one draws at 4x size into 8-bit greys and
shrinks with a Lanczos filter, so it runs on the Pi only. Which of the two drives
the panel is chosen by `backend` in `hexa_bringup/config/face.yaml`; never both.

## Files

- **`image.{hpp,cpp}`** — 8-bit image, shape fills, thick lines, Lanczos
  downscale, rotate and masked paste.
- **`eyes.{hpp,cpp}`** — `draw_eyes()` and every mood. Moods: normal, happy,
  angry, sleepy, dead, woozy, love, thug-life and the scanning family (scanning,
  scanning-ccw, scanning-counter, snake-chase, hop-chase, snake-infinity,
  hop-infinity).
- **`sh1122.{hpp,cpp}`** — spidev + GPIO character device driver; sends only the
  rows that changed since the last frame.

## Provenance

Vendored from `eye-animation-cpp` at commit `31e3d46`, unmodified.

## Re-syncing

Run `./sync.sh [path]` with a checkout of eye-animation-cpp (default
`../../../eye-animation-cpp`, next to this repo). It re-copies the six files and
prints the commit; update the line above.
