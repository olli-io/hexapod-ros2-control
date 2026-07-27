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

from .network_state import NetworkState
from .screen_logic import Screen

#: Characters that fit one panel line of ordinary mixed text. The panel wraps
#: at 252 px (256 wide, 2 px margins) and the bundled PixelOperator 16 px font
#: is proportional, averaging ~8 px per glyph, so 30 is the working figure —
#: not the "~42" a naive character count suggests.
#:
#: It is a *text* budget, not a pixel guarantee: the widest glyphs advance 9 px,
#: so 28 is the floor that holds whatever the characters are. Both numbers are
#: measured against the real font in hexa_display's test_text_screen.cpp
#: (ThirtyCharactersOfRealTextFitOneLine, TwentyEightWideGlyphsAreTheGuaranteedFloor).
#: Worth remembering before putting a configurable SSID on a line already at 30.
#:
#: Overflow does not truncate, it *wraps*, and a wrap can push the fourth line
#: off a four-line panel (``text_screen.cpp``: max_lines = height /
#: line_spacing). A silently dropped line is why this is a guarded budget.
LINE_BUDGET = 30


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


def battery_screen(
    voltage: float | None,
    ip: str,
    config: InfoConfig,
    network: NetworkState | None = None,
) -> str:
    """``Battery -> 50 %  ( 7.4 V )\\nControl -> 192.168.1.42:8080``.

    In hotspot mode a third line carries the credentials, so the network a
    phone has to join is always one tap away rather than only visible in the
    seconds after a switch. In station mode the screen is unchanged — there is
    no hotspot to name.
    """
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
    lines = [battery, control]
    if network is not None and network.is_hotspot and network.ssid:
        lines.append(f"WiFi {config.arrow} {network.ssid} / {network.psk}")
    return "\n".join(lines)


def bluetooth_screen(controller: str, hold_s: float) -> str:
    """The controller-status screen.

    ``hold_s`` is the same value the buttons wait for, so the panel can never
    advertise a hold they do not honour.
    """
    pair = f"Hold {_round_half_away(hold_s)} seconds to pair"
    if not controller:
        return f"No connected controllers\n{pair}"
    return f"Connected to:\n{controller}\n{pair}"


#: Host error tokens -> something an operator can act on. Kept here, in the
#: pure module, so the shell script only ever has to emit a short stable word.
#: Every value is inside LINE_BUDGET.
_ERROR_REASONS = {
    "no-helper": "No network helper installed",
    "timeout": "The switch timed out",
    "nmcli-missing": "NetworkManager is missing",
    "no-wifi": "No wlan0 device found",
    "unmanaged": "wlan0 is not managed by NM",
    "no-country": "No wifi country is set",
    "ap-failed": "Could not start the hotspot",
    "station-failed": "Could not rejoin the wifi",
}


def network_error_reason(reason: str) -> str:
    """Explain a host error token. Unknown ones pass through, clipped to fit.

    Passing an unknown token through rather than swallowing it means a host
    script that grows a new failure mode still says *something* useful on the
    panel before anyone updates this table.
    """
    if not reason:
        return "The switch did not finish"
    return _ERROR_REASONS.get(reason, reason[:LINE_BUDGET])


def network_screen(
    state: NetworkState, ip: str, config: InfoConfig, hold_s: float
) -> str:
    """How the network switch turned out.

    Hotspot: the credentials and where to point a browser. Station: just the
    address, since the network it rejoined is one the operator already knows.
    Failure: what went wrong and that holding again retries.
    """
    if state.failed:
        return (
            f"Network switch failed\n"
            f"{network_error_reason(state.reason)}\n"
            f"Hold {_round_half_away(hold_s)} seconds to retry"
        )
    if state.is_hotspot:
        lines = [f"Hotspot {config.arrow} {state.ssid}"]
        if state.psk:
            lines.append(f"Password {config.arrow} {state.psk}")
        lines.append(
            f"Control {config.arrow} {ip}:{config.control_port}"
            if ip
            else f"Control {config.arrow} no network"
        )
        return "\n".join(lines)
    control = (
        f"Control {config.arrow} {ip}:{config.control_port}"
        if ip
        else f"Control {config.arrow} no network"
    )
    return f"Hotspot off\n{control}"


def screen_text(
    screen: Screen,
    *,
    voltage: float | None,
    ip: str,
    controller: str,
    config: InfoConfig,
    hold_s: float,
    network: NetworkState | None = None,
) -> str:
    """Render a screen. Empty means "hand the panel back to the face".

    SCANNING and NETWORK_SWITCHING render empty on purpose: their spinners
    *are* the face, so text mode has to be off for them to show.
    """
    if screen is Screen.BATTERY:
        return battery_screen(voltage, ip, config, network)
    if screen is Screen.BLUETOOTH:
        return bluetooth_screen(controller, hold_s)
    if screen is Screen.NETWORK_INFO:
        return network_screen(network or NetworkState(), ip, config, hold_s)
    return ""
