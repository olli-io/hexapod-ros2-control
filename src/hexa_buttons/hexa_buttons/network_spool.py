"""The two file touches that carry a network-mode switch across the container
boundary. ``network_state`` decides what the bytes mean; this moves them.

Impure, and therefore not re-exported from ``__init__`` and not unit-tested —
same policy as ``gpio_buttons``. There is no decision in here to test: every
function is one syscall plus the house rule that a filesystem that will not
cooperate is a feature that quietly does not work, never a node that dies.

Note the deliberate asymmetry in how the two files are written:

- the **state** file (host -> container) is written by the host with
  tmp + ``rename(2)``, so the node can never read a half-written line.
- the **request** file (container -> host) is written in place, truncate + one
  write + close. It must NOT be renamed into
  place: systemd's ``PathModified`` watches that path's inode once it exists,
  and swapping a new inode underneath delivers ``IN_MOVE_SELF`` instead of a
  modify, so the unit would stop firing. One short line written in one call is
  atomic enough in practice, and the host reads it with ``head -n 1``.
"""
from __future__ import annotations

import os


def request(path: str, action: str, token: str) -> bool:
    """Ask the host for a switch. False if the spool could not be written.

    A False here is what the caller turns into "No network helper installed" on
    the panel — on a robot with no bind-mounted log volume this is the first
    thing that fails, and it fails immediately rather than after a timeout.
    """
    from .network_state import format_request

    if not path:
        return False
    try:
        with open(path, "w") as spool:
            spool.write(format_request(action, token))
    except OSError:
        return False
    return True


def state_mtime(path: str) -> float:
    """Modification time of the state file, or 0.0 if it is not there yet.

    Cheaper than a read, so the tick can check this every time and only parse
    when the host has actually said something new.
    """
    if not path:
        return 0.0
    try:
        return os.stat(path).st_mtime
    except OSError:
        return 0.0


def read_state(path: str) -> str:
    """The state file's contents, or "" if it cannot be read."""
    if not path:
        return ""
    try:
        with open(path) as state:
            return state.read()
    except OSError:
        return ""
