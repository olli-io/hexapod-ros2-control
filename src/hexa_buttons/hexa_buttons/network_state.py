"""The wire format the container and the host swap network-mode over.

Pure: no rclpy, no I/O, no clocks. ``network_spool`` does the reads and writes;
this decides what the bytes mean.

Why a file protocol at all: the ROS stack runs in an unprivileged container
(``docker-compose.robot.yaml`` — host network, non-root user, no D-Bus socket,
no NET_ADMIN), so it cannot run ``nmcli``. It asks the host instead, exactly the
way ``hexa_hardware`` asks for a buzzer tune: write a word into the bind-mounted
log volume and let a host ``systemd .path`` unit relay it out. See
``systemd/network-mode.sh``.

Two files, deliberately not one:

- the **request** (``log/network``) goes container -> host. One line, one word,
  plus a token.
- the **state** (``log/network.state``) comes host -> container. Several
  ``key=value`` lines.

The host must never write the request file: that is the file its own ``.path``
unit watches, so a write there would re-trigger it forever.

The **token** is what separates "the host answered *me*" from "the state file
still holds the answer to a switch from ten minutes ago". The node mints a fresh
one per request; the host echoes it back. ``mode`` is trusted regardless of the
token — it is the host's standing report of where the radio actually is, and a
node that just restarted needs it to render a correct screen.
"""
from __future__ import annotations

from dataclasses import dataclass

#: What to ask the host for. TOGGLE is what the button sends: the host owns the
#: radio, so it is also the only thing that reliably knows which way to go.
REQUEST_TOGGLE = "toggle"
REQUEST_HOTSPOT = "hotspot"
REQUEST_STATION = "station"

#: Where the radio is. Empty means "no state file yet" — not an error, just a
#: robot whose host helper has never run.
MODE_UNKNOWN = ""
MODE_STATION = "station"
MODE_HOTSPOT = "hotspot"

#: How a request is going. SWITCHING is the host's ack, written before it starts
#: the slow part, which is what lets the node tell "no helper installed" from
#: "helper working on it".
RESULT_SWITCHING = "switching"
RESULT_OK = "ok"
RESULT_ERROR = "error"

_KEYS = ("mode", "token", "result", "reason", "ssid", "psk")


@dataclass(frozen=True)
class NetworkState:
    """The host's last word. All-empty is the "never heard from it" default."""

    mode: str = MODE_UNKNOWN
    token: str = ""
    result: str = ""
    #: Short machine token on an error (``ap-failed``, ``nmcli-missing``, ...).
    #: ``info_text.network_error_reason`` turns it into something readable.
    reason: str = ""
    ssid: str = ""
    psk: str = ""

    @property
    def is_hotspot(self) -> bool:
        return self.mode == MODE_HOTSPOT

    @property
    def failed(self) -> bool:
        return self.result == RESULT_ERROR


def new_token(nonce: str, seq: int) -> str:
    """A request token.

    The nonce is per-node-process, so a node that restarts mid-switch cannot
    mistake the in-flight answer to its *previous* life's request for an ack of
    a new one. The sequence number separates successive requests within one life.
    """
    return f"{nonce}-{seq}"


def format_request(action: str, token: str) -> str:
    """One line, trailing newline. ``head -n 1`` on the host reads it."""
    return f"{action} {token}\n"


def parse_request(text: str) -> tuple[str, str]:
    """``(action, token)`` from a request line. Junk yields ``("", "")``.

    Here rather than only in the shell so the format has one definition and the
    round-trip is testable.
    """
    parts = text.strip().split()
    if not parts:
        return "", ""
    return parts[0], parts[1] if len(parts) > 1 else ""


def parse_state(text: str) -> NetworkState | None:
    """Parse the state file. ``None`` if there is nothing usable in it.

    Deliberately tolerant. The file is written by a shell script on the other
    side of a bind mount, and although the host writes it atomically
    (tmp + rename), a truncated or half-written file must read as "no news"
    rather than raise inside a 20 Hz tick.

    Splits each line on the **first** ``=`` only, so a password containing one
    survives. Unknown keys are ignored, missing keys default to empty.
    """
    fields: dict[str, str] = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        key, sep, value = line.partition("=")
        if not sep:
            continue
        key = key.strip()
        if key in _KEYS:
            fields[key] = value.strip()
    if not fields:
        return None
    return NetworkState(**fields)


def is_terminal(result: str) -> bool:
    """True once the host has stopped working on a request, either way."""
    return result in (RESULT_OK, RESULT_ERROR)
