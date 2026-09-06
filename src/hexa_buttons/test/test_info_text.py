"""Unit tests for the strings the front-panel buttons put on the display."""
import dataclasses

import pytest

from hexa_buttons import (
    LINE_BUDGET,
    MODE_HOTSPOT,
    MODE_STATION,
    RESULT_ERROR,
    RESULT_OK,
    InfoConfig,
    NetworkState,
    Screen,
    battery_percent,
    battery_screen,
    bluetooth_screen,
    network_error_reason,
    network_screen,
    screen_text,
)

HOTSPOT = NetworkState(
    mode=MODE_HOTSPOT, result=RESULT_OK, ssid="hexapod", psk="hexahexa"
)
#: What a current host reports: the same hotspot, plus the name its DNS answers.
NAMED_HOTSPOT = dataclasses.replace(HOTSPOT, portal="control.hexa")
STATION = NetworkState(mode=MODE_STATION, result=RESULT_OK)


def test_battery_percent_maps_the_configured_span_linearly():
    config = InfoConfig()  # 2S LiPo: 6.6 V empty, 8.4 V full
    assert battery_percent(8.4, config) == 100
    assert battery_percent(7.5, config) == 50
    assert battery_percent(6.6, config) == 0


def test_battery_percent_clamps_outside_the_span():
    config = InfoConfig()
    assert battery_percent(9.9, config) == 100
    assert battery_percent(0.0, config) == 0


def test_battery_percent_degenerate_span_reads_zero():
    """A mis-set config must not divide by zero or emit nonsense."""
    config = dataclasses.replace(InfoConfig(), battery_empty_v=8.4, battery_full_v=8.4)
    assert battery_percent(8.4, config) == 0


def test_battery_percent_rounds_half_away_from_zero():
    """Python's round() is banker's rounding — round(0.5) == 0 — which would
    show a 50.5 % pack as 50 %. C's lround, which this ported from, rounds half
    away from zero."""
    config = dataclasses.replace(InfoConfig(), battery_empty_v=0.0, battery_full_v=200.0)
    assert battery_percent(1.0, config) == 1  # 0.5 % -> 1
    assert battery_percent(5.0, config) == 3  # 2.5 % -> 3


def test_battery_screen_shows_percentage_voltage_and_address():
    config = InfoConfig(control_port=8080)
    assert battery_screen(7.5, "192.168.172.42", config) == (
        "Battery -> 50 %  ( 7.5 V )\nControl -> 192.168.172.42:8080"
    )


def test_battery_screen_no_voltage_reads_as_dashes():
    """No aux poll yet: dashes, never a fabricated 0 % that reads as a dead
    pack."""
    assert battery_screen(None, "10.0.0.5", InfoConfig()) == (
        "Battery -> -- %  ( --.- V )\nControl -> 10.0.0.5:8080"
    )


def test_battery_screen_no_address_says_so():
    assert battery_screen(8.4, "", InfoConfig()) == (
        "Battery -> 100 %  ( 8.4 V )\nControl -> no network"
    )


def test_battery_screen_honours_the_configured_separator():
    config = dataclasses.replace(InfoConfig(), arrow=":")
    assert battery_screen(8.4, "10.0.0.5", config) == (
        "Battery : 100 %  ( 8.4 V )\nControl : 10.0.0.5:8080"
    )


def test_bluetooth_screen_names_the_connected_controller():
    assert bluetooth_screen("8 Bit Do Pro 2", 3.0) == (
        "Connected to:\n8 Bit Do Pro 2\nHold 3 seconds to pair"
    )


def test_bluetooth_screen_says_nothing_is_connected():
    assert bluetooth_screen("", 3.0) == (
        "No connected controllers\nHold 3 seconds to pair"
    )


def test_bluetooth_screen_advertises_the_configured_hold():
    """The advertised hold comes from the same config the buttons wait on, so
    the panel can never promise a hold they do not honour."""
    assert bluetooth_screen("", 5.0) == (
        "No connected controllers\nHold 5 seconds to pair"
    )


def test_screen_text_dispatches_to_each_screen():
    kwargs = dict(
        voltage=7.5,
        ip="10.0.0.5",
        controller="8 Bit Do Pro 2",
        config=InfoConfig(),
        hold_s=3.0,
    )
    assert screen_text(Screen.BATTERY, **kwargs).startswith("Battery ->")
    assert screen_text(Screen.BLUETOOTH, **kwargs).startswith("Connected to:")


def test_screen_text_renders_nothing_for_none_and_the_busy_screens():
    """The spinners are the face, so text mode has to be off for them to show —
    same as NONE."""
    kwargs = dict(
        voltage=7.5, ip="10.0.0.5", controller="", config=InfoConfig(), hold_s=3.0
    )
    assert screen_text(Screen.NONE, **kwargs) == ""
    assert screen_text(Screen.SCANNING, **kwargs) == ""
    assert screen_text(Screen.NETWORK_SWITCHING, **kwargs) == ""


# --- network mode ---------------------------------------------------------


def test_the_battery_screen_carries_the_credentials_in_hotspot_mode():
    """So the network a phone has to join is always one tap away, not only
    visible in the seconds after a switch."""
    assert battery_screen(7.5, "192.168.4.1", InfoConfig(), HOTSPOT) == (
        "Battery -> 50 %  ( 7.5 V )\n"
        "Control -> 192.168.4.1:8080\n"
        "WiFi -> hexapod / hexahexa"
    )


def test_the_battery_screen_is_unchanged_in_station_mode():
    plain = "Battery -> 50 %  ( 7.5 V )\nControl -> 10.0.0.5:8080"
    assert battery_screen(7.5, "10.0.0.5", InfoConfig(), STATION) == plain
    assert battery_screen(7.5, "10.0.0.5", InfoConfig(), None) == plain


def test_the_hotspot_result_screen_names_the_network_and_where_to_point():
    assert network_screen(HOTSPOT, "192.168.4.1", InfoConfig(), 3.0) == (
        "Hotspot -> hexapod\n"
        "Password -> hexahexa\n"
        "Control -> 192.168.4.1:8080"
    )


def test_the_hotspot_screens_show_the_name_over_the_address():
    """The AP resolves control.hexa itself and serves port 80, so that is what a
    person should be reading off the panel and typing."""
    assert battery_screen(7.5, "192.168.4.1", InfoConfig(), NAMED_HOTSPOT) == (
        "Battery -> 50 %  ( 7.5 V )\n"
        "Control -> control.hexa\n"
        "WiFi -> hexapod / hexahexa"
    )
    assert network_screen(NAMED_HOTSPOT, "192.168.4.1", InfoConfig(), 3.0) == (
        "Hotspot -> hexapod\nPassword -> hexahexa\nControl -> control.hexa"
    )


def test_the_name_is_only_used_on_the_hotspot():
    """Off the AP nothing resolves it — the robot is a guest on somebody else's
    network, so the routable address is the only useful thing to show."""
    named_station = dataclasses.replace(STATION, portal="control.hexa")
    assert network_screen(named_station, "10.0.0.5", InfoConfig(), 3.0) == (
        "Hotspot off\nControl -> 10.0.0.5:8080"
    )


def test_the_mdns_name_goes_first_on_a_station_network_with_the_address_under_it():
    """avahi answers <hostname>.local over multicast on whatever network the
    robot joins, so unlike control.hexa this name needs no DNS the robot
    controls. It leads because it survives a phone locking and nobody has to
    read four numbers off a 64-pixel panel.

    The address stays, on a line of its own: ".local" is not answered on every
    network or by every phone, and a name that silently does not resolve with no
    address beside it is worse than the address alone. Station mode uses two of
    the panel's four lines, so the third is free.
    """
    config = dataclasses.replace(InfoConfig(), mdns_name="hexa.local")
    assert battery_screen(7.5, "192.168.1.42", config, STATION) == (
        "Battery -> 50 %  ( 7.5 V )\n"
        "Control -> hexa.local:8080\n"
        "   or  -> 192.168.1.42:8080"
    )
    assert network_screen(STATION, "192.168.1.42", config, 3.0) == (
        "Hotspot off\n"
        "Control -> hexa.local:8080\n"
        "   or  -> 192.168.1.42:8080"
    )


def test_no_mdns_name_leaves_the_station_screens_exactly_as_they_were():
    """The default is empty, and it has to stay inert: only the host knows
    whether `hexa robot install-mdns` was ever run."""
    assert InfoConfig().mdns_name == ""
    assert battery_screen(7.5, "10.0.0.5", InfoConfig(), STATION) == (
        "Battery -> 50 %  ( 7.5 V )\nControl -> 10.0.0.5:8080"
    )


def test_the_mdns_name_is_ignored_on_the_hotspot():
    """The AP answers control.hexa from its own DNS and serves port 80, so the
    portal name still wins and no fallback line is spent — the credentials need
    that room, and every name resolves there anyway."""
    config = dataclasses.replace(InfoConfig(), mdns_name="hexa.local")
    assert battery_screen(7.5, "192.168.4.1", config, NAMED_HOTSPOT) == (
        "Battery -> 50 %  ( 7.5 V )\n"
        "Control -> control.hexa\n"
        "WiFi -> hexapod / hexahexa"
    )


def test_the_mdns_lines_fit_the_panel():
    """Two lines where there was one, on a screen that wraps rather than
    truncates — and a wrap can push the last line off a four-line panel."""
    config = dataclasses.replace(InfoConfig(), mdns_name="hexa.local")
    lines = battery_screen(7.5, "192.168.100.200", config, STATION).split("\n")
    assert len(lines) == 3
    for line in lines:
        assert len(line) <= LINE_BUDGET, line


def test_the_station_result_screen_says_the_hotspot_is_off():
    assert network_screen(STATION, "10.0.0.5", InfoConfig(), 3.0) == (
        "Hotspot off\nControl -> 10.0.0.5:8080"
    )


def test_a_result_screen_with_no_address_says_so():
    assert network_screen(STATION, "", InfoConfig(), 3.0).endswith("no network")


def test_a_failure_says_what_went_wrong_and_how_to_retry():
    failed = NetworkState(mode=MODE_STATION, result=RESULT_ERROR, reason="ap-failed")
    assert network_screen(failed, "10.0.0.5", InfoConfig(), 3.0) == (
        "Network switch failed\n"
        "Could not start the hotspot\n"
        "Hold 3 seconds to retry"
    )


@pytest.mark.parametrize(
    "reason",
    [
        "no-helper",
        "timeout",
        "nmcli-missing",
        "no-wifi",
        "unmanaged",
        "no-country",
        "ap-failed",
        "station-failed",
    ],
)
def test_every_host_error_token_reads_as_english(reason):
    text = network_error_reason(reason)
    assert text != reason
    assert len(text) <= LINE_BUDGET


def test_an_unknown_error_token_still_says_something():
    """A host script that grows a failure mode must not render a blank line
    just because this table has not caught up."""
    assert network_error_reason("wpa-supplicant-exploded") == "wpa-supplicant-exploded"
    assert network_error_reason("") == "The switch did not finish"


def test_screen_text_dispatches_the_result_screen():
    assert screen_text(
        Screen.NETWORK_INFO,
        voltage=7.5,
        ip="192.168.4.1",
        controller="",
        config=InfoConfig(),
        hold_s=3.0,
        network=HOTSPOT,
    ).startswith("Hotspot -> hexapod")


# --- panel budget ---------------------------------------------------------


def _every_screen():
    """Every string this module can put on the panel, worst case each."""
    config = InfoConfig()
    long_ip = "192.168.172.42"
    failures = [
        NetworkState(mode=MODE_STATION, result=RESULT_ERROR, reason=reason)
        for reason in (
            "no-helper",
            "timeout",
            "nmcli-missing",
            "no-wifi",
            "unmanaged",
            "no-country",
            "ap-failed",
            "station-failed",
            "",
        )
    ]
    yield battery_screen(None, long_ip, config, HOTSPOT)
    yield battery_screen(7.55, long_ip, config, HOTSPOT)
    yield battery_screen(8.4, "", config, STATION)
    yield bluetooth_screen("", 3.0)
    yield bluetooth_screen("8 Bit Do Pro 2", 3.0)
    yield network_screen(HOTSPOT, long_ip, config, 3.0)
    yield network_screen(STATION, long_ip, config, 3.0)
    yield network_screen(STATION, "", config, 3.0)
    for state in failures:
        yield network_screen(state, long_ip, config, 3.0)


@pytest.mark.parametrize("text", list(_every_screen()))
def test_every_screen_fits_the_panel(text):
    """The panel drops the fifth line silently, and overflow *wraps* rather
    than truncating — so a line over budget does not look wrong, it makes an
    unrelated line disappear. Hence a hard guard rather than eyeballing."""
    lines = text.split("\n")
    assert len(lines) <= 4
    for line in lines:
        assert len(line) <= LINE_BUDGET, line


@pytest.mark.parametrize("text", list(_every_screen()))
def test_every_screen_is_ascii(text):
    """The bundled PixelOperator font covers ASCII + Latin-1 only; anything
    outside it renders blank. See gen_font.py's codepoints()."""
    assert text.isascii()
