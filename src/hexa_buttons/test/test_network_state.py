"""The container/host spool wire format.

Pure string work — no filesystem, no host, no rclpy. The point of these is that
the two sides of a bind mount agree on what the bytes mean, and that a
half-written file reads as "no news" rather than raising inside a 20 Hz tick.
"""
import pytest

from hexa_buttons import (
    MODE_HOTSPOT,
    MODE_STATION,
    REQUEST_TOGGLE,
    RESULT_ERROR,
    RESULT_OK,
    RESULT_SWITCHING,
    NetworkState,
    format_request,
    is_terminal,
    new_token,
    parse_request,
    parse_state,
)

# --- requests -------------------------------------------------------------


def test_request_round_trips():
    line = format_request(REQUEST_TOGGLE, "3f9a2c01-2")
    assert line.endswith("\n")  # the host reads it with `head -n 1`
    assert parse_request(line) == (REQUEST_TOGGLE, "3f9a2c01-2")


def test_request_without_a_token_parses():
    assert parse_request("toggle\n") == ("toggle", "")


@pytest.mark.parametrize("junk", ["", "   ", "\n"])
def test_empty_request_is_not_an_action(junk):
    assert parse_request(junk) == ("", "")


def test_token_is_unique_per_request_and_per_process():
    assert new_token("aaaa", 1) != new_token("aaaa", 2)  # successive requests
    assert new_token("aaaa", 1) != new_token("bbbb", 1)  # a restarted node


# --- state ----------------------------------------------------------------


def test_parses_a_full_hotspot_report():
    state = parse_state(
        "mode=hotspot\ntoken=3f9a2c01-2\nresult=ok\nssid=hexapod\npsk=hexahexa\n"
        "portal=control.hexa\n"
    )
    assert state == NetworkState(
        mode=MODE_HOTSPOT,
        token="3f9a2c01-2",
        result=RESULT_OK,
        ssid="hexapod",
        psk="hexahexa",
        portal="control.hexa",
    )


def test_a_report_from_a_host_that_names_no_portal_still_parses():
    """An older network-mode.sh on the Pi predates the key; the panel falls back
    to the address rather than the node failing to read the file."""
    state = parse_state("mode=hotspot\nresult=ok\nssid=hexapod\n")
    assert state is not None and state.portal == ""
    assert state.is_hotspot
    assert not state.failed


def test_missing_keys_default_to_empty():
    state = parse_state("mode=station\n")
    assert state == NetworkState(mode=MODE_STATION)
    assert not state.is_hotspot


def test_an_error_carries_its_reason():
    state = parse_state("mode=station\ntoken=a-1\nresult=error\nreason=ap-failed\n")
    assert state.failed
    assert state.reason == "ap-failed"


def test_a_password_may_contain_an_equals_sign():
    # Split on the FIRST '=' only, or a perfectly legal WPA passphrase is
    # silently mangled into something nobody can type in.
    state = parse_state("mode=hotspot\npsk=he=xa=hexa\n")
    assert state.psk == "he=xa=hexa"


def test_unknown_keys_are_ignored():
    # The host may grow a field before this side learns about it.
    assert parse_state("mode=hotspot\nchannel=6\n") == NetworkState(mode=MODE_HOTSPOT)


def test_comments_and_blank_lines_are_ignored():
    assert parse_state("# written by network-mode.sh\n\nmode=station\n") == NetworkState(
        mode=MODE_STATION
    )


def test_crlf_and_stray_whitespace_survive():
    state = parse_state("  mode = hotspot \r\n  ssid = hexapod \r\n")
    assert state == NetworkState(mode=MODE_HOTSPOT, ssid="hexapod")


@pytest.mark.parametrize("junk", ["", "   \n\n", "not a state file at all", "#\n"])
def test_unusable_input_reads_as_no_news(junk):
    # None, never an exception and never a fabricated NetworkState() that would
    # read as "the host says station mode".
    assert parse_state(junk) is None


def test_a_truncated_write_reads_as_no_news_or_partial_truth():
    # tmp+rename on the host makes this unlikely, but a partial line must never
    # produce a wrong answer: either the key parsed whole, or it is absent.
    assert parse_state("mode=hots") == NetworkState(mode="hots")  # whole line, junk value
    assert parse_state("mo") is None  # no '=' yet


# --- result classification ------------------------------------------------


def test_only_ok_and_error_end_a_request():
    assert is_terminal(RESULT_OK)
    assert is_terminal(RESULT_ERROR)
    # SWITCHING is the host's ack that it exists and is working — the node must
    # keep spinning on it, not treat it as a result.
    assert not is_terminal(RESULT_SWITCHING)
    assert not is_terminal("")
