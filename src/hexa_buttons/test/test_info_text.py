"""Unit tests for the strings the front-panel buttons put on the display."""
import dataclasses

from hexa_buttons import (
    InfoConfig,
    Screen,
    battery_percent,
    battery_screen,
    bluetooth_screen,
    screen_text,
)


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


def test_screen_text_renders_nothing_for_none_and_scanning():
    """SCANNING's spinners are the face, so text mode has to be off for them to
    show — same as NONE."""
    kwargs = dict(
        voltage=7.5, ip="10.0.0.5", controller="", config=InfoConfig(), hold_s=3.0
    )
    assert screen_text(Screen.NONE, **kwargs) == ""
    assert screen_text(Screen.SCANNING, **kwargs) == ""
