# hexa_buzzer

Passive buzzer on the Raspberry Pi's hardware PWM (GPIO12). Real robot only;
the node runs inert without a PWM block.

## Two players, one code

- **`buzzer_node`** (container) — plays `up`, `fault`, `undervolt` from
  `/buzzer/play`.
- **host systemd units** — play `boot` and `shutdown`, before Docker starts and
  after the container is gone. They run `python3 -m hexa_buzzer.player <event>`
  against the files `hexa deploy` ships to `~/hexa-robot/hexa_buzzer/`.

`tunes.py`, `catalog.py`, `pwm.py`, `player.py` are stdlib-only (no rclpy, no
PyYAML). `test_package_purity.py` enforces this.

## Layout

- **`tunes.py`** — RTTTL parser. Pure.
- **`catalog.py`** — reads `config/tunes.yaml` and `config/buzzer.yaml` with a
  small YAML-subset reader, resolves a name (event first, tune second).
- **`pwm.py`** — sysfs seam: find chip, claim channel, square wave, release.
  Failures raise `BuzzerHardwareError`.
- **`player.py`** — note sequencer and the host CLI.
- **`buzzer_node.py`** — the only rclpy module.

`__init__.py` re-exports only the pure module.

## Config

- **`config/tunes.yaml`** — named RTTTL strings (`name:defaults:notes`, quoted).
  Octaves are scientific pitch, A4 = 440 Hz.
- **`config/buzzer.yaml`** — node parameters plus an `events:` map from the
  word on `/buzzer/play` to a tune. An event naming a missing tune fails loudly.

Adding an event is one line in each file, no code.

## PWM access

The container gets the PWM tree as a writable bind mount at `/pwm`
(`docker-compose.buzzer.yaml`). `scripts/robot.sh` adds the overlay only when
the host has the tree. Wiring and `config.txt` overlay: `docs/robot-environment.md`
§15.

## Topic

- **`/buzzer/play`** (`std_msgs/String`, transient_local, depth 1) — event name.
  Published by `hexa_hardware`. Latched because `up` can beat subscription
  matching.

Tunes play on a worker thread. One pending tune, no queue. The channel is
claimed per tune and released after, so `./hexa robot play-tune` works while the
stack is up.

## Failure is never fatal

No buzzer, no overlay, no mount, busy channel: silence, one logged error, node
stays alive. CLI exits 0.

## Tests

    ./hexa sim python3 -m pytest src/hexa_buzzer/test -q

- **`test_tunes.py`** — RTTTL parser and note table.
- **`test_catalog.py`** — YAML reader, shipped tunes and events.
- **`test_pwm_channel.py`** — sysfs seam against a tmp directory.
- **`test_package_purity.py`** — no-rclpy rule.
