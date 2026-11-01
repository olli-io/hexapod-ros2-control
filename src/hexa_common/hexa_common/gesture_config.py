"""Pure-Python loader for the gesture ids in ``gestures.yaml``."""

from __future__ import annotations

from pathlib import Path

import yaml


def load_gesture_ids(gestures_yaml: str | Path) -> tuple[str, ...]:
    """The gesture ids ``gestures.yaml`` declares, in declaration order.

    The web Gesture view is one tile per id, and both teleops check a request
    against the same tuple. Ids only — the keyframes are the engine's
    business, and it validates them itself when it loads the file.
    """
    with Path(gestures_yaml).open() as f:
        raw = yaml.safe_load(f) or {}
    entries = raw.get("gestures")
    if not isinstance(entries, list):
        raise ValueError("gestures.yaml: `gestures` must be a list")
    ids = tuple(str(entry["id"]) for entry in entries)
    if len(set(ids)) != len(ids):
        raise ValueError("gestures.yaml: duplicate gesture id")
    return ids
