"""Sequences a tune's notes, and the CLI the host's systemd units run.

    python3 -m hexa_buzzer.player up
    python3 -m hexa_buzzer.player --rtttl "coin:d=4,o=6,b=200:16b5,2e"

The name is an event from buzzer.yaml, or a tune from tunes.yaml.

Stdlib only, no rclpy: the boot and shutdown units run this without a ROS
environment. Failure is never fatal — every hardware error prints a line and
exits 0, so a beep cannot hold up a boot, a shutdown, or a fault.
"""
from __future__ import annotations

import argparse
import sys
import time

from . import catalog, pwm, tunes


# Taken out of each note rather than added after it, so a tune keeps its written
# tempo. Without a gap, two notes at the same pitch run together.
NOTE_GAP_S = 0.015


def play_notes(
    notes: list[tuple[int, float]],
    *,
    pwm_dev: str = pwm.DEFAULT_PWM_DEV,
    channel: int = pwm.DEFAULT_CHANNEL,
    wait_s: float = pwm.DEFAULT_WAIT_S,
) -> None:
    """Sound `(hz, seconds)` pairs in order. Raises BuzzerHardwareError."""
    with pwm.Channel(pwm_dev, channel, wait_s) as line:
        for hz, seconds in notes:
            gap = min(NOTE_GAP_S, seconds / 2.0)
            line.tone(hz)
            time.sleep(seconds - gap)
            line.tone(0)
            time.sleep(gap)


def play(
    name: str,
    *,
    pwm_dev: str = pwm.DEFAULT_PWM_DEV,
    channel: int = pwm.DEFAULT_CHANNEL,
    wait_s: float = pwm.DEFAULT_WAIT_S,
    rtttl: str = "",
    config_dir: str = "",
) -> None:
    """Sound an event or tune name, or `rtttl` if one is given. Raises on failure."""
    notes = tunes.parse_rtttl(rtttl or catalog.lookup(name, config_dir))
    play_notes(notes, pwm_dev=pwm_dev, channel=channel, wait_s=wait_s)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="hexa_buzzer.player", description="Play a tune on the hexapod's buzzer."
    )
    parser.add_argument(
        "tune", nargs="?", default="boot",
        help="event from buzzer.yaml or tune from tunes.yaml (default: boot)",
    )
    parser.add_argument(
        "--rtttl", default="",
        help="play this RTTTL string instead of a named tune, "
             "e.g. 'coin:d=4,o=6,b=200:16b5,2e'",
    )
    parser.add_argument(
        "--config-dir", default="",
        help="where tunes.yaml and buzzer.yaml live (default: beside the package)",
    )
    parser.add_argument("--pwm-dev", default=pwm.DEFAULT_PWM_DEV)
    parser.add_argument("--channel", type=int, default=pwm.DEFAULT_CHANNEL)
    parser.add_argument(
        "--wait", type=float, default=pwm.DEFAULT_WAIT_S,
        help="seconds to wait for the PWM driver to probe (boot only)",
    )
    args = parser.parse_args(argv)

    try:
        play(
            args.tune,
            pwm_dev=args.pwm_dev,
            channel=args.channel,
            wait_s=args.wait,
            rtttl=args.rtttl,
            config_dir=args.config_dir,
        )
    except (pwm.BuzzerHardwareError, OSError, ValueError) as exc:
        print(f"buzzer: {exc} — staying silent.", file=sys.stderr)

    # Always 0: a unit that failed here would make a missing buzzer a failed boot.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
