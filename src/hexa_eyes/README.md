# hexa_eyes

An optional add-on **face**: an alternative to `hexa_display` that draws the eyes
in 16 greys instead of 1 bit. One C++ node (`eyes_node`) reads the same robot
state, runs the same expression/gaze policy and drives the same 256×64 SH1122
OLED. Pure sink — nothing imports it or subscribes to it.

It is never run beside `hexa_display`: the two share the panel's SPI bus and
GPIO lines. `hexa_bringup/config/face.yaml` has one setting, `backend`, with
three values — `none` (the default), `hexa_display` and `eyes_cpp` — and the
robot launch starts exactly the node it names. To use this face, set
`backend: eyes_cpp`.

## Layout

- `shared/display_core` — the policy (`hexa::display`), the face animations, the
  text screen and `eyes::EyeAnim` (blinks, gaze easing), compiled directly here
  exactly as `hexa_display` compiles them. So expression precedence, gaze and
  idling behave the same in both faces.
- `shared/eyes_core` — the vendored greyscale renderer and SH1122 driver from
  eye-animation-cpp. See its README for provenance and re-syncing.
- `src/eyes_face.{hpp,cpp}` — pure: maps each expression to a mood and draws one
  animation frame; converts the text screen's buffer to an image.
- `src/eyes_node.cpp` — rclcpp glue: caches topics, ticks the policy (~10 Hz),
  draws at `render_hz`. Its input handling is a copy of `display_node.cpp`'s, so
  keep the two in step when either changes.
- `hexa_display/src/{Sh1122Panel,gesture_expressions}.cpp` — compiled here, not
  copied: the panel class only as a headless u8g2 buffer for the text screen, and
  the gesture-expression loader for the policy.

## Behavior

- **Expressions** — chosen by the shared policy, with the same precedence as
  `hexa_display` (busy, battery critical, gesture, battery warning, animation
  mode, posture sticks, gait state). Each expression then draws a **mood** from
  `expression_mood.*` in `eyes.yaml`.
- **Moods** — normal, happy, angry, sleepy, dead, woozy, love, thug-life, and the
  scanning family: scanning, scanning-ccw, scanning-counter, snake-chase,
  hop-chase, snake-infinity, hop-infinity. Any expression can be given any mood.
  `shared/eyes_core/README.md` lists what each one looks like.
- **Greedy** — the 1-bit face has art for it and this one does not, so it shows
  thug-life by default. Change `expression_mood.greedy` to taste.
- **Animated moods** — thug-life, the scanning family, love and woozy move on
  their own clock, which restarts each time the mood shown changes, so thug-life
  always starts with bare eyes and then drops the glasses.
- **Text mode** — a non-empty `/display/text` replaces the eyes with the text,
  drawn by the same text screen `hexa_display` uses, so `hexa_buttons` works with
  either face. Empty returns the face.
- **Headless** — `headless: true` runs the whole pipeline without opening SPI or
  GPIO (CI).

## Topics (subscribes only)

The same as `hexa_display`: `/gait/state` and `/gait/gesture`, `/cmd_vel`,
`/body/pose`, `/animation/mode`, `/display/text`, `/bluetooth/scanning`,
`/display/busy` and the battery topic. See `hexa_display`'s README.

## Configuration

`config/eyes.yaml` holds only what is specific to this renderer: `render_hz`,
`headless`, panel `contrast`, `hflip` and `vflip`, and the `expression_mood.*`
map. The policy and gaze knobs and the SPI/GPIO pins are read from
`hexa_display/config/display.yaml`, so the two faces never disagree about them.
The greyscale driver leaves chip select to the SPI controller, so `cs_line` must
stay `-1`.

Mood names are validated at startup, so a typo stops the node with the list of
valid moods.

## Notes

- **Orientation** — `hflip: false`, `vflip: true` matches the panel setup
  `hexa_display` leaves in place. If the face is mirrored or upside down, flip the
  matching switch in `eyes.yaml`.
- **Cost** — drawing takes about 1.5 to 4 ms per frame on a Raspberry Pi 5 and the
  driver sends only the rows that changed. This face is Pi-only: it needs an 8-bit
  frame buffer, which the Pico firmware does not have.
- **Verified so far** — `eyes_face` and its tests build and pass on a plain host.
  The node itself has not been built in the ROS container or run on the robot.

## Tests

`colcon test --packages-select hexa_eyes` — gtest, headless: `test_eyes_face`.
