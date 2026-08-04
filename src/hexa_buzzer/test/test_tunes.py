"""Tests for the RTTTL parser. The tunes themselves are test_catalog.py's.

Imports from the package root, never a submodule, so a test can only reach pure
code — the same guard hexa_buttons' suites use.
"""
import pytest

from hexa_buzzer import note_hz, parse_rtttl

# At 200 bpm a whole note is 1.2 s, a quarter 0.3 s, a sixteenth 0.075 s.
HEAD = "test:d=4,o=6,b=200:"


# --- Note names -----------------------------------------------------------


@pytest.mark.parametrize(
    "note,hz",
    [
        ("A4", 440),  # the definition of equal temperament, A4 = 440 Hz
        ("C4", 262),
        ("G5", 784),
        ("B5", 988),
        ("C6", 1047),
        ("E6", 1319),
        ("G#6", 1661),
        ("B6", 1976),
        ("C7", 2093),
        ("G7", 3136),
    ],
)
def test_the_note_table_the_shell_player_shipped(note, hz):
    """The exact integers the sh `note_hz` case statement carried, now computed
    instead. E6 pins the rounding: it lands on 1318.5 and has to go up."""
    assert note_hz(note) == hz


def test_a_flat_is_its_sharp():
    assert note_hz("Ab4") == note_hz("G#4")
    assert note_hz("Bb4") == note_hz("A#4")


def test_an_octave_is_a_doubling():
    assert note_hz("A5") == 2 * note_hz("A4")


@pytest.mark.parametrize("junk", ["", "H5", "C", "5", "C#", "Cx4", "c4", "C##4"])
def test_a_name_that_is_not_a_note_is_refused(junk):
    """Loudly, because a melody is either ours or a deliberate override — a
    typo in one is a mistake to report, not a note to skip past."""
    with pytest.raises(ValueError):
        note_hz(junk)


# --- RTTTL ----------------------------------------------------------------


def test_the_mario_coin():
    """The tune that named the format for us: a sixteenth-note B5 into a long
    E6 a perfect fourth up, at 200 bpm."""
    assert parse_rtttl("coin:d=4,o=6,b=200:16b5,2e") == [
        (988, pytest.approx(0.075)),
        (1319, pytest.approx(0.6)),
    ]


def test_a_note_value_is_a_fraction_of_a_whole_note():
    """Not a count of ticks: `4` is a quarter of the 1.2 s whole note 200 bpm
    gives, whatever the other notes in the tune are doing."""
    assert parse_rtttl(HEAD + "1e,2e,4e,8e,16e") == [
        (1319, pytest.approx(1.2)),
        (1319, pytest.approx(0.6)),
        (1319, pytest.approx(0.3)),
        (1319, pytest.approx(0.15)),
        (1319, pytest.approx(0.075)),
    ]


def test_the_tempo_is_the_tunes_own():
    """Halving bpm doubles every note. Nothing outside the string says how fast
    a tune goes, which is what lets a pasted ringtone keep its own tempo."""
    slow = parse_rtttl("test:d=4,o=6,b=100:4e")
    assert slow == [(1319, pytest.approx(0.6))]


def test_a_bare_note_takes_the_default_duration_and_octave():
    assert parse_rtttl(HEAD + "e") == parse_rtttl(HEAD + "4e6")


def test_an_octave_on_the_note_overrides_the_default():
    assert [hz for hz, _ in parse_rtttl(HEAD + "e,e5,e7")] == [1319, 659, 2637]


def test_a_dot_is_half_as_long_again():
    dotted = parse_rtttl(HEAD + "4e.")
    assert dotted == [(1319, pytest.approx(0.45))]


def test_a_dot_before_the_octave_is_the_same_dot():
    """The spec puts it last; ringtones in the wild put it either side of the
    octave digit."""
    assert parse_rtttl(HEAD + "4e.5") == parse_rtttl(HEAD + "4e5.")


def test_a_sharp_is_a_semitone_up():
    assert parse_rtttl(HEAD + "4g#")[0][0] == note_hz("G#6")


def test_a_pause_is_silence_that_still_takes_time():
    hz, seconds = parse_rtttl(HEAD + "8p")[0]
    assert hz == 0
    assert seconds == pytest.approx(0.15)


def test_notes_keep_their_order():
    assert [hz for hz, _ in parse_rtttl(HEAD + "4c,4g5,4c")] == [1047, 784, 1047]


def test_case_and_whitespace_are_not_load_bearing():
    """A downloaded ringtone arrives however its author left it."""
    assert parse_rtttl("test:D=4, O=6, B=200: 16B5, 2E") == parse_rtttl(HEAD + "16b5,2e")


def test_a_trailing_comma_is_tolerated():
    """The commonest artefact of a pasted ringtone, and an empty token cannot be
    hiding a typo."""
    assert parse_rtttl(HEAD + "4e,") == parse_rtttl(HEAD + "4e")


def test_an_empty_defaults_section_falls_back_to_the_spec():
    """d=4, o=6, b=63 — a whole note is nearly four seconds at that tempo."""
    assert parse_rtttl("test::c") == [(1047, pytest.approx(60.0 / 63.0))]


@pytest.mark.parametrize(
    "junk",
    [
        "",
        "test:d=4,o=6,b=200",  # two sections, no notes
        "test:d=4,o=6,b=200:16b5:2e",  # four
        "B5:2 REST:1 E6:4",  # the format this replaced, and it must not parse
        "test:d=4,o=6,b=200:",  # header only
        "test:d=4,o=6,b=200:16h5",  # not a note letter
        "test:d=4,o=6,b=200:12e",  # not a note value
        "test:d=4,o=6,b=200:4eb",  # RTTTL has no flats
        "test:d=4,o=6,b=0:4e",  # a tune that would never finish
        "test:d=3,o=6,b=200:4e",  # not a note value, in the defaults
        "test:d=4,o=99,b=200:4e",  # not an octave
        "test:d=4,o=6,x=200:4e",  # not a default
    ],
)
def test_an_unplayable_string_is_refused(junk):
    with pytest.raises(ValueError):
        parse_rtttl(junk)
