"""RTTTL (`name:defaults:notes`) parsed to (hz, seconds).

    coin:d=4,o=6,b=200:16b5,2e

Defaults are `b` beats per minute, `d` note value, `o` octave. A note is
`[duration]letter[#][octave][.]`, where the duration is a fraction of a whole
note and a whole note is four beats — `16b5` is a sixteenth-note B5, `8p` an
eighth rest, `1g.` a dotted whole note.

Borrowed rather than invented so any ringtone in the format pastes in verbatim
and carries its own tempo. Octave numbering, the part RTTTL players disagree on,
is scientific pitch here: A4 = 440 Hz.

The tunes themselves live in `config/tunes.yaml`, read by `catalog`.

Pure — no I/O, no clock, no hardware.
"""
from __future__ import annotations

import math
import re

# Spec fallbacks, for what a tune leaves unsaid.
DEFAULT_DURATION = 4
DEFAULT_OCTAVE = 6
DEFAULT_BPM = 63

PAUSE = "p"

# Checked against rather than just divided by, so `12b` — a typo for `2b` or
# `16b` — is refused instead of sounding like neither.
_DURATIONS = frozenset({1, 2, 4, 8, 16, 32, 64})

_SEMITONES = {"C": 0, "D": 2, "E": 4, "F": 5, "G": 7, "A": 9, "B": 11}
_NOTE = re.compile(r"([A-G])([#b]?)(-?\d)$")

_DEFAULT_TOKEN = re.compile(r"([dob])=(\d+)", re.IGNORECASE)
# The dot is accepted on either side of the octave; the spec's place is last,
# but ringtones in the wild use both.
_NOTE_TOKEN = re.compile(r"(\d{1,2})?([a-gp])(#?)(\.?)(\d?)(\.?)", re.IGNORECASE)


def note_hz(note: str) -> int:
    """Frequency of a scientific-pitch note name like `C5`, `G#6` or `Bb4`.

    Equal temperament, A4 = 440 Hz. Rounding is half-up, not Python's half-even,
    so the values match the integer table this replaced note for note.
    """
    match = _NOTE.fullmatch(note)
    if match is None:
        raise ValueError(f"{note!r} is not a note name like 'C5', 'G#6' or 'Bb4'")
    letter, accidental, octave = match.groups()

    midi = (int(octave) + 1) * 12 + _SEMITONES[letter]
    if accidental == "#":
        midi += 1
    elif accidental == "b":
        midi -= 1

    return math.floor(440.0 * 2.0 ** ((midi - 69) / 12.0) + 0.5)


def _checked_duration(value: int, token: str) -> int:
    if value not in _DURATIONS:
        raise ValueError(
            f"{token!r} has note value {value} — RTTTL durations are "
            f"{', '.join(str(d) for d in sorted(_DURATIONS))}"
        )
    return value


def _parse_defaults(defaults: str) -> tuple[int, int, int]:
    """The `d=4,o=6,b=200` section as `(duration, octave, bpm)`."""
    duration, octave, bpm = DEFAULT_DURATION, DEFAULT_OCTAVE, DEFAULT_BPM

    for token in defaults.split(","):
        token = token.strip()
        if not token:
            continue
        match = _DEFAULT_TOKEN.fullmatch(token)
        if match is None:
            raise ValueError(
                f"{token!r} is not an RTTTL default — 'd=' duration, "
                f"'o=' octave or 'b=' beats per minute"
            )
        key, value = match.group(1).lower(), int(match.group(2))
        if key == "d":
            duration = _checked_duration(value, token)
        elif key == "o":
            if not 0 <= value <= 9:
                raise ValueError(f"{token!r} is outside octave 0-9")
            octave = value
        else:
            if value <= 0:
                raise ValueError(f"tempo must be positive, got {token!r}")
            bpm = value

    return duration, octave, bpm


def parse_rtttl(rtttl: str) -> list[tuple[int, float]]:
    """An RTTTL string as `(hz, seconds)` pairs. A rest is `hz == 0`.

    Raises ValueError on anything unplayable: a tune is either ours or an
    operator's override, so a typo is a mistake to report, not a note to skip.
    """
    sections = rtttl.split(":")
    if len(sections) != 3:
        raise ValueError(
            f"{rtttl!r} is not RTTTL — expected 'name:defaults:notes', "
            f"like 'coin:d=4,o=6,b=200:16b5,2e'"
        )
    _, defaults, notes = sections
    duration, octave, bpm = _parse_defaults(defaults)
    whole_s = 4.0 * 60.0 / bpm

    played: list[tuple[int, float]] = []
    for token in notes.split(","):
        token = token.strip()
        if not token:
            continue  # trailing comma, the commonest artefact of a paste
        match = _NOTE_TOKEN.fullmatch(token)
        if match is None:
            raise ValueError(
                f"{token!r} is not an RTTTL note — [duration]letter[#][octave][.]"
            )
        value, letter, sharp, dot_before, note_octave, dot_after = match.groups()

        seconds = whole_s / (
            _checked_duration(int(value), token) if value else duration
        )
        if dot_before or dot_after:
            seconds *= 1.5

        if letter.lower() == PAUSE:
            played.append((0, seconds))
        else:
            pitch = f"{letter.upper()}{sharp}{note_octave or octave}"
            played.append((note_hz(pitch), seconds))

    if not played:
        raise ValueError(f"{rtttl!r} has no notes")
    return played
