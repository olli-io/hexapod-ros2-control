"""The two tables the buzzer plays from, and the small YAML reader that gets them.

`tunes.yaml` holds named RTTTL tones; `buzzer.yaml` maps an event — the word
that arrives on /buzzer/play — to one of them. A name resolves as an event
first, a tune second, so `play-tune fault` and `play-tune coin` both work.

Stdlib only, no PyYAML: the boot and shutdown units run this on a Pi OS Lite
host, where `python3` is guaranteed and `python3-yaml` is not. `_parse_mapping`
takes the subset both files are — nested mappings of plain scalars — and refuses
the rest rather than guessing. They stay ordinary YAML, so `robot.launch.py`
reads buzzer.yaml with PyYAML and gets the same answer.

Touches the filesystem, so it stays out of the package root's re-exports.
"""
from __future__ import annotations

import os
import re

TUNES_FILE = "tunes.yaml"
BUZZER_FILE = "buzzer.yaml"

_HERE = os.path.dirname(os.path.abspath(__file__))
# realpath too: under colcon --symlink-install the importable package is a
# symlink into the source tree, where config/ sits a level up.
_REAL = os.path.dirname(os.path.realpath(__file__))

# Deployed layout (~/hexa-robot/hexa_buzzer/config/) and source layout. The node
# passes its ament share directory instead of searching.
SEARCH_DIRS = tuple(
    dict.fromkeys(
        os.path.join(base, "config")
        for base in (_HERE, os.path.dirname(_HERE), _REAL, os.path.dirname(_REAL))
    )
)

_KEY_LINE = re.compile(r"(?P<indent> *)(?P<key>[A-Za-z_][\w.-]*) *:(?P<rest>.*)")


def _scalar(rest: str, where: str, line_no: int) -> str | None:
    """The value after `key:`, or None when the line opens a nested mapping."""
    text = rest.strip()
    if not text or text.startswith("#"):
        return None

    if text[0] in "\"'":
        quote = text[0]
        end = text.find(quote, 1)
        if end < 0:
            raise ValueError(f"{where}:{line_no}: unterminated {quote} quote")
        trailing = text[end + 1 :].strip()
        if trailing and not trailing.startswith("#"):
            raise ValueError(f"{where}:{line_no}: {trailing!r} follows the value")
        return text[1:end]

    if text[0] in "[{|>&*!":
        raise ValueError(
            f"{where}:{line_no}: {text!r} — plain scalars only, no flow "
            f"collections, block scalars, anchors or tags"
        )

    # The space before a comment is load-bearing: `4g#` is a sharp.
    cut = text.find(" #")
    return (text[:cut] if cut >= 0 else text).strip()


def _parse_mapping(text: str, where: str = "<string>") -> dict:
    """Nested mappings of plain scalars, as nested dicts of strings.

    Nothing is coerced to bool or number — the only things read through here
    are names.
    """
    root: dict = {}
    stack: list[tuple[int, dict]] = []
    opened: dict | None = None  # mapping the previous `key:` line started

    for line_no, raw in enumerate(text.splitlines(), start=1):
        line = raw.rstrip()
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if "\t" in line[: len(line) - len(line.lstrip())]:
            raise ValueError(f"{where}:{line_no}: indent with spaces, not tabs")

        match = _KEY_LINE.fullmatch(line)
        if match is None:
            raise ValueError(
                f"{where}:{line_no}: {stripped!r} is not 'key:' or 'key: value'"
            )
        indent = len(match.group("indent"))

        if not stack:
            if indent:
                raise ValueError(f"{where}:{line_no}: the first key is indented")
            stack.append((0, root))
        elif opened is not None and indent > stack[-1][0]:
            stack.append((indent, opened))
        else:
            while len(stack) > 1 and indent < stack[-1][0]:
                stack.pop()
            if indent != stack[-1][0]:
                raise ValueError(
                    f"{where}:{line_no}: indent {indent} opens no block"
                )
        opened = None

        key, parent = match.group("key"), stack[-1][1]
        if key in parent:
            raise ValueError(f"{where}:{line_no}: {key!r} is set twice")

        value = _scalar(match.group("rest"), where, line_no)
        if value is None:
            opened = {}
            parent[key] = opened
        else:
            parent[key] = value

    return root


def find_config(name: str, config_dir: str = "") -> str:
    """Path to a config file, searched for unless `config_dir` says where."""
    tried = [config_dir] if config_dir else SEARCH_DIRS
    for directory in tried:
        path = os.path.join(directory, name)
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(
        f"no {name} in {', '.join(tried)} — `hexa deploy push` ships the config "
        f"beside the player"
    )


def _names_at(data: dict, keys: tuple[str, ...], where: str) -> dict[str, str]:
    node: object = data
    for key in keys:
        node = node.get(key) if isinstance(node, dict) else None
        if not isinstance(node, dict):
            raise ValueError(f"{where}: expected a '{':'.join(keys)}:' mapping")
    if not node:
        raise ValueError(f"{where}: '{':'.join(keys)}:' is empty")
    for key, value in node.items():
        if not isinstance(value, str):
            raise ValueError(f"{where}: {key!r} is a block, not a name")
    return node


def _read(path: str) -> str:
    with open(path) as handle:
        return handle.read()


def load_tunes(config_dir: str = "") -> dict[str, str]:
    """tunes.yaml's `tunes:` mapping — tune name to RTTTL string."""
    path = find_config(TUNES_FILE, config_dir)
    return _names_at(_parse_mapping(_read(path), path), ("tunes",), path)


def load_events(config_dir: str = "") -> dict[str, str]:
    """buzzer.yaml's `events:` mapping — event name to tune name."""
    path = find_config(BUZZER_FILE, config_dir)
    return _names_at(
        _parse_mapping(_read(path), path),
        ("buzzer_node", "ros__parameters", "events"),
        path,
    )


def resolve(name: str, tunes: dict[str, str], events: dict[str, str]) -> str:
    """The RTTTL for an event name, or for a tune named directly.

    Raises ValueError on a name neither table knows: a request is either the
    system's or an operator's, so a typo is a mistake to report, not silence.
    """
    if name in events:
        tune = events[name]
        if tune not in tunes:
            raise ValueError(
                f"event '{name}' plays tune '{tune}', which tunes.yaml does not "
                f"define — it has {', '.join(sorted(tunes))}"
            )
        return tunes[tune]
    if name in tunes:
        return tunes[name]
    raise ValueError(
        f"unknown event or tune '{name}' — events are "
        f"{', '.join(sorted(events))}; tunes are {', '.join(sorted(tunes))}"
    )


def lookup(name: str, config_dir: str = "") -> str:
    """The RTTTL for an event or tune name, both tables read fresh."""
    return resolve(name, load_tunes(config_dir), load_events(config_dir))
