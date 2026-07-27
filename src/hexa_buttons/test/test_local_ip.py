"""Unit tests for picking the address the battery screen advertises.

Only the ranking is tested — the ioctl enumeration underneath it depends on
kernel behaviour and is verified on hardware, the same policy
hexa_teleop/test_joy_publisher.py applies to its /dev/input reads.
"""
from hexa_buttons import rank_interfaces

PREFERRED = ["wlan0", "eth0"]


def test_prefers_the_first_listed_interface():
    addresses = {"eth0": "10.0.0.5", "wlan0": "192.168.1.42"}
    assert rank_interfaces(addresses, PREFERRED) == "192.168.1.42"


def test_falls_back_to_the_second_listed():
    assert rank_interfaces({"eth0": "10.0.0.5"}, PREFERRED) == "10.0.0.5"


def test_falls_back_to_an_unlisted_interface():
    """A robot on a USB tether or a bridge still shows something."""
    assert rank_interfaces({"usb0": "172.16.0.2"}, PREFERRED) == "172.16.0.2"


def test_returns_empty_when_nothing_has_an_address():
    assert rank_interfaces({}, PREFERRED) == ""


def test_preference_order_beats_enumeration_order():
    """dict order here is eth0 first; the config still wins."""
    addresses = {"eth0": "10.0.0.5", "usb0": "172.16.0.2", "wlan0": "192.168.1.42"}
    assert rank_interfaces(addresses, PREFERRED) == "192.168.1.42"
