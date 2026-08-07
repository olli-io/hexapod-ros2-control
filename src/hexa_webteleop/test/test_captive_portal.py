"""What the teleop server does with a request that is not the webapp's.

Pure string work — no aiohttp, no sockets. These pin the rule that makes a
phone joining the robot's hotspot end up holding the controller without anyone
telling it an address: every path that is not a webapp file is somebody who
wants the app, so it goes to the app rather than to a 404.
"""
from pathlib import Path

import pytest

from hexa_webteleop.captive_portal import request_path, static_filename


# --- request paths --------------------------------------------------------


def test_the_decoration_a_client_adds_does_not_change_a_request():
    """Connectivity probes append cache-busting queries, and clients add
    trailing slashes; neither may make a known path read as an unknown one."""
    assert request_path("/generate_204?ts=1712345") == "/generate_204"
    assert request_path("/main.js#frag") == "/main.js"
    assert request_path("/ncsi.txt/") == "/ncsi.txt"


def test_the_root_survives_normalisation():
    """Stripping trailing slashes must not turn "/" into an empty path."""
    assert request_path("/") == "/"
    assert request_path("/?x=1") == "/"


# --- static files ---------------------------------------------------------


def test_a_flat_asset_name_is_served_from_the_web_directory():
    assert static_filename("/main.js") == "main.js"
    assert static_filename("/logs.html") == "logs.html"


@pytest.mark.parametrize(
    "target",
    [
        "/",  # the app itself, served by its own route
        "/some/other/site",  # a stale tab, via the hotspot's wildcard DNS
        "/library/test/success.html",  # macOS' probe, nested besides
    ],
)
def test_nothing_nested_is_an_asset(target):
    """None means "redirect to the controller" — the webapp is flat, so a path
    with a directory in it belongs to somebody else entirely."""
    assert static_filename(request_path(target)) is None


#: The flat paths an OS asks for to decide whether a network has internet.
#: Nothing here decides behaviour — a probe is served like any other name, which
#: is to say it is redirected, because no such file exists. That is the point of
#: the test below.
_PROBE_NAMES = (
    "generate_204",  # Android, ChromeOS
    "gen_204",  # Android (older)
    "hotspot-detect.html",  # iOS, macOS — captive.apple.com
    "success.html",  # macOS
    "connecttest.txt",  # Windows 10/11
    "ncsi.txt",  # Windows (legacy NCSI)
    "redirect",  # Windows — msftconnecttest.com
    "canonical.html",  # Firefox
    "success.txt",  # Firefox — detectportal
    "check_network_status.txt",  # GNOME
    "nm-check.txt",  # NetworkManager
)


@pytest.mark.parametrize("name", _PROBE_NAMES)
def test_the_webapp_ships_no_file_an_os_probes_for(name):
    """A phone opens its sign-in browser on the controller *because* its probe
    goes unanswered. An asset added under one of these names would answer it,
    the phone would call the hotspot an ordinary network, and the popup that
    saves anyone having to be told an address would quietly stop happening.
    """
    assert not (Path(__file__).resolve().parents[1] / "web" / name).exists()


@pytest.mark.parametrize(
    "target", ["/../config/webteleop.yaml", "/..", "/.env", "/foo/../../etc/passwd"]
)
def test_no_path_can_escape_the_web_directory(target):
    assert static_filename(request_path(target)) is None
