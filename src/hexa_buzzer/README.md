# hexa_buzzer

The robot's only audible channel: a passive buzzer on the Raspberry Pi's
hardware PWM (GPIO12). Four things it says — "the Pi is alive", "the stack is
up", "I tripped", and "you can cut the power now" — plus an undervoltage
warning.

Real robot only. There is no buzzer in sim, and the node runs inert without a
PWM block, so nothing here has to be simulated.

## The one thing to know first

The buzzer is driven from **two places that cannot be one place**:

- **`buzzer_node`, in the container** — plays `up`, `fault` and `undervolt`,
  the tunes that mean something about a running stack.
- **two host systemd units** — play `boot` and `shutdown`. Neither can be the
  node's: the boot chirp lands seconds after the kernel, long before Docker,
  and the shutdown chirp fires at `final.target`, after the container is
  already gone. That is the whole reason they exist separately.

Both run **the same Python**. `tunes.py`, `catalog.py`, `pwm.py` and `player.py`
are stdlib-only and import no rclpy, so `hexa deploy` ships those five files
(`__init__.py` included) plus `config/` to `~/hexa-robot/hexa_buzzer/`, and the
units run `python3 -m hexa_buzzer.player boot` against them. One tune table, one
event map, one player — an event sounds identical whichever side plays it.

`test_package_purity.py` enforces the no-rclpy rule in a fresh interpreter,
because breaking it is a one-line convenience import and the symptom is a
shutdown that silently stops chirping. The same rule is why nothing here parses
YAML with PyYAML: `python3` is guaranteed on a Pi OS Lite host, `python3-yaml`
is not.

## Layout

Pure modules carry the tests; the device and ROS layers stay thin and separate.
`__init__.py` re-exports **only** the pure module, so importing the package
pulls in neither rclpy nor the filesystem — the same rule as `hexa_buttons`.

- **`tunes.py`** — pure. The RTTTL parser. Note frequencies are computed
  (equal temperament, A4 = 440 Hz), not tabulated.
- **`catalog.py`** — reads `config/tunes.yaml` and `config/buzzer.yaml`, and
  resolves a name against them. Stdlib only, so it carries a small YAML subset
  reader instead of PyYAML: nested mappings of plain scalars, which is all
  either file is, and a loud error on anything else.
- **`pwm.py`** — the sysfs seam: find the chip, claim a channel, put a square
  wave on it, let it go. Stdlib only. Every failure is one
  `BuzzerHardwareError`.
- **`player.py`** — sequences the notes, and the CLI the host units run.
  Stdlib only.
- **`buzzer_node.py`** — the only rclpy module. Subscribes `/buzzer/play`.

## Events and tunes

Two files, because an occasion and a noise are different things:

- **`config/tunes.yaml`** — named tones. Named for what they sound like
  (`chirp`, `klaxon`, `groan`), not for when they play.
- **`config/buzzer.yaml`** — the node's parameters, plus an `events:` map from
  the word that arrives on `/buzzer/play` to a tune above.

So `up: ready` is what "the stack is live" currently sounds like. Change the
right side to give an event a different voice; add an event with a line in each
file and no code either side. An event naming a tune that does not exist is
refused loudly when it fires, rather than played as silence.

The name on `/buzzer/play` — and on `play-tune` — resolves as an event first
and a tune second, which is what lets you audition a tone no event points at
(`./hexa robot play-tune coin`).

## Tunes are RTTTL

`tunes.yaml` holds Nokia ringtone strings, `name:defaults:notes`:

    coin:d=4,o=6,b=200:16b5,2e

- **defaults** — `b` beats per minute, `d` the note value a note gets when it
  gives none, `o` its octave. Any subset, any order; the spec's fallbacks are
  `d=4,o=6,b=63`.
- **a note** — `[duration]letter[#][octave][.]`. `16b5` is a sixteenth-note B5,
  `2e` a half note in the default octave, `1g.` a dotted whole note, `8p` an
  eighth rest.
- **duration** — a note *value*, the denominator of a whole note (`4` = quarter,
  `8` = eighth), not a count of ticks. A whole note is four beats at `b`.

The point of borrowing the format rather than inventing one: a ringtone found
anywhere pastes into `tunes.yaml` verbatim, and each tune carries its own tempo,
so there is no global tempo knob that would distort a tune written elsewhere.
Octave numbering is the one part of RTTTL players disagree on (some are off by
one) — ours is scientific pitch, A4 = 440 Hz. Quote the strings in the YAML: an
unquoted `#` would start a comment instead of sharpening a note.

Notation has no gap between notes and a passive buzzer needs one, or two notes
at the same pitch run together. `player.NOTE_GAP_S` takes 15 ms out of the end
of every note rather than adding it after, so a tune plays at the tempo it was
written at.

## Reaching the PWM

The container gets the PWM block through a bind mount, `/pwm`, added by
`docker-compose.buzzer.yaml`. `scripts/robot.sh` adds that overlay only when the
host actually has the tree, because a bind whose source is missing stops the
whole stack from starting — and plenty of builds have no buzzer fitted.

This is what replaced the old arrangement, where the container wrote a tune name
into a file on the log volume and a host `systemd .path` unit relayed it to a
shell script, because `/sys` is mounted read-only and an export from inside
failed with `EROFS`. A writable bind of just that one directory is enough; the
container already runs as root, so no udev rule or supplementary group is
involved.

Wiring, the `config.txt` overlay, and the Pi 4 differences are in
`docs/robot-environment.md` §15.

## Topic

- **`/buzzer/play`** (`std_msgs/String`, transient_local, depth 1) — an event
  name: `up`, `fault`, `undervolt`, published by `hexa_hardware` on activation,
  on an over-current trip, and on the undervoltage warning rung. `boot` and
  `shutdown` are the same map's events, asked for by the host units instead.

Latched, and that is load-bearing rather than symmetry with the other topics:
`up` goes out from `on_activate()`, which can beat this node's subscription
matching, and a volatile reader would drop the one tune that says the robot is
ready. The price is that restarting `hexa_buzzer` alone replays the last tune
once.

A tune is one to two seconds of sleeps, so it is played on a worker thread and
never on the executor. One pending tune, not a queue — a tune arriving while one
is playing is dropped, because a backlog of stale beeps helps nobody.

The channel is claimed for the length of a tune and released after. Holding it
for the node's lifetime would break `./hexa robot play-tune`, which is the
buzzer bring-up tool and has to keep working while the stack is up.

## Failure is never fatal

No buzzer fitted, no overlay in `config.txt`, no `/pwm` mount, a busy channel:
all of them mean silence and nothing else. The node logs one error at startup
and stays alive but inert; the CLI prints a line and exits 0. A beep must never
hold up a boot, a shutdown, or the fault path that asked for it.

## Tests

    ./hexa sim python3 -m pytest src/hexa_buzzer/test -q

- **`test_tunes.py`** — the note table and the RTTTL parser.
- **`test_catalog.py`** — the YAML subset reader, and the shipped tunes and
  events: that the five the rest of the system names still resolve, and that
  they kept the pitches they had when the table was baked into `tunes.py`.
- **`test_pwm_channel.py`** — the sysfs seam against a tmp directory. The one
  suite that imports a submodule directly: sysfs is just files, so a tmp_path
  reproduces it faithfully, and claiming/releasing the channel is the part with
  real consequences.
- **`test_package_purity.py`** — the no-rclpy rule above.

Not tested here, verified on the robot: that a real RP1 PWM channel makes an
audible noise, and the launch wiring.
