"""The strings the front-panel buttons put on the display, and the pack
voltage -> percentage map behind the battery one.

Pure: no rclpy, no I/O. The node supplies the readings, this decides what they
read as. Output goes out verbatim on ``/display/text``, which word-wraps and
centers it (see ``shared/display_core/text_screen.hpp``) — the newlines here are
the hard line breaks.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .screen_logic import Screen


@dataclass(frozen=True)
class InfoConfig:
    #: Linear voltage -> percentage map for the pack. Defaults are a 2S LiPo:
    #: 3.3 V/cell empty, 4.2 V/cell full.
    battery_empty_v: float = 6.6
    battery_full_v: float = 8.4
    #: Port the web teleop UI listens on. Loaded from hexa_webteleop's
    #: webteleop.yaml at launch so the two can never disagree.
    control_port: int = 8080
    #: Separator between a label and its value. ASCII by default, and it has to
    #: be: the bundled Pixel Operator font covers ASCII + Latin-1 only, so a
    #: U+2192 arrow would come out blank. Widening the character set means
    #: adding the codepoint to ``codepoints()`` in
    #: ``shared/display_core/tools/gen_font.py`` and regenerating
    #: ``hexa_text_font.c``.
    arrow: str = "->"


def _round_half_away(value: float) -> int:
    """Round half away from zero, as C's lround does.

    Not ``round()``: that is banker's rounding, so ``round(0.5) == 0`` and
    ``round(2.5) == 2``. These numbers are read off a panel by a human, and a
    50.5 % pack must show 51 %, not 50 %.
    """
    return int(math.floor(value + 0.5)) if value >= 0 else -int(math.floor(-value + 0.5))


def battery_percent(voltage: float, config: InfoConfig) -> int:
    """Pack percentage, 0-100, clamped.

    A non-positive span (empty >= full, i.e. a mis-set config) reads 0 rather
    than dividing by zero.
    """
    span = config.battery_full_v - config.battery_empty_v
    if span <= 0.0:
        return 0
    ratio = (voltage - config.battery_empty_v) / span
    return _round_half_away(min(max(ratio, 0.0), 1.0) * 100.0)


def battery_screen(voltage: float | None, ip: str, config: InfoConfig) -> str:
    """``Battery -> 50 %  ( 7.4 V )\\nControl -> 192.168.1.42:8080``."""
    if voltage is None:
        # No aux poll yet (or none at all — aux_period_ms: 0). Dashes, not a
        # fabricated 0 %, which would read as a dead pack.
        battery = f"Battery {config.arrow} -- %  ( --.- V )"
    else:
        pct = battery_percent(voltage, config)
        battery = f"Battery {config.arrow} {pct} %  ( {voltage:.1f} V )"
    if ip:
        control = f"Control {config.arrow} {ip}:{config.control_port}"
    else:
        control = f"Control {config.arrow} no network"
    return f"{battery}\n{control}"


def bluetooth_screen(controller: str, hold_s: float) -> str:
    """The controller-status screen.

    ``hold_s`` is the same value the buttons wait for, so the panel can never
    advertise a hold they do not honour.
    """
    pair = f"Hold {_round_half_away(hold_s)} seconds to pair"
    if not controller:
        return f"No connected controllers\n{pair}"
    return f"Connected to:\n{controller}\n{pair}"


def screen_text(
    screen: Screen,
    *,
    voltage: float | None,
    ip: str,
    controller: str,
    config: InfoConfig,
    hold_s: float,
) -> str:
    """Render a screen. Empty means "hand the panel back to the face".

    SCANNING renders empty on purpose: its spinners *are* the face, so text
    mode has to be off for them to show.
    """
    if screen is Screen.BATTERY:
        return battery_screen(voltage, ip, config)
    if screen is Screen.BLUETOOTH:
        return bluetooth_screen(controller, hold_s)
    return ""
