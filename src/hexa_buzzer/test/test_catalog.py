"""Tests for the YAML subset reader and the two tables it reads."""
import os

import pytest

from hexa_buzzer import parse_rtttl
from hexa_buzzer.catalog import _parse_mapping, load_events, load_tunes, resolve

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config")

TUNES = load_tunes(CONFIG_DIR)
EVENTS = load_events(CONFIG_DIR)


def test_a_nested_mapping_comes_back_nested():
    assert _parse_mapping(
        "buzzer_node:\n  ros__parameters:\n    events:\n      up: ready\n"
    ) == {"buzzer_node": {"ros__parameters": {"events": {"up": "ready"}}}}


def test_a_quoted_value_keeps_its_hashes():
    """The whole reason RTTTL strings are quoted: a sharp is not a comment."""
    parsed = _parse_mapping('tunes:\n  # a tune\n  ready: "ready:d=4,o=6,b=200:4g#"\n')
    assert parsed["tunes"]["ready"] == "ready:d=4,o=6,b=200:4g#"


@pytest.mark.parametrize("junk", ["- a\n", "a: [1, 2]\n", "a: 1\n  b: 2\n"])
def test_what_the_reader_does_not_understand_is_refused(junk):
    """Loudly: a config it misreads is a beep that never happens."""
    with pytest.raises(ValueError):
        _parse_mapping(junk)


@pytest.mark.parametrize("event", ["boot", "up", "shutdown", "fault", "undervolt"])
def test_the_five_events_the_rest_of_the_system_asks_for_by_name(event):
    """hexa_hardware publishes three of these and the systemd units run two.
    Renaming one silently is a beep that stops happening, so pin the names."""
    assert EVENTS[event] in TUNES


def test_every_tune_plays_and_none_outlasts_the_units_that_wait_for_it():
    """The shutdown unit gives the player 10 s and a beep must never be the
    reason a power-off feels stuck."""
    for name, rtttl in TUNES.items():
        assert 0 < sum(seconds for _, seconds in parse_rtttl(rtttl)) < 3.0, name


def test_the_events_kept_their_pitches_across_the_yaml_move():
    """Re-pointed, not rewritten: what they sounded like when the table was
    still baked into tunes.py, note for note."""
    pitches = {
        event: [hz for hz, _ in parse_rtttl(resolve(event, TUNES, EVENTS))]
        for event in ("boot", "up", "shutdown", "fault", "undervolt")
    }
    assert pitches == {
        "boot": [988, 0, 1319],
        "up": [1319, 0, 1661, 0, 1976],
        "shutdown": [1319, 0, 988],
        "fault": [1047, 784, 1047, 784, 1047, 784],
        "undervolt": [784],
    }


def test_a_tune_can_be_named_directly():
    """What makes `play-tune coin` work on a tune no event points at."""
    assert resolve("coin", TUNES, EVENTS) == TUNES["coin"]


def test_an_unknown_name_lists_both_tables():
    with pytest.raises(ValueError, match="undervolt.*coin"):
        resolve("nope", TUNES, EVENTS)
