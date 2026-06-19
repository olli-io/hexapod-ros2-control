"""Pure arbitration for shared /cmd_vel ownership between teleop sources.

Two teleop heads — the gamepad node (``hexa_teleop.teleop_joy``) and the
web node (``hexa_webteleop.webteleop_node``) — can run simultaneously.
Only one publishes ``/cmd_vel`` + ``/body/pose`` at a time. A single
latched ``/teleop/owner`` topic (``std_msgs/String``) carries the
current owner:

- **``GAMEPAD``** (default) — the gamepad node publishes.
- **``WEB``** — the web node publishes; the gamepad node goes dormant.

Only the web node writes ``/teleop/owner`` — the gamepad node is a
passive reader — so there is no write-write race. The default owner is
``GAMEPAD``; the gamepad node treats "no owner message seen yet" as
owning, so a standalone sim/dev session (no web node running) is
behaviourally unchanged.

Pure-python; rclpy-free so the claim/release logic is unit-testable
standalone and importable from both teleop heads without ROS overhead.
"""

from __future__ import annotations

from dataclasses import dataclass

GAMEPAD = "gamepad"
WEB = "web"
_VALID_OWNERS = frozenset({GAMEPAD, WEB})


@dataclass
class ArbitrationState:
    """Current ownership state, mirrored from ``/teleop/owner``.

    ``owner`` starts at ``GAMEPAD`` (the default) so a gamepad node that
    never sees an owner message publishes immediately. ``seen_owner_msg``
    flips to True on the first ``/teleop/owner`` message and lets a node
    distinguish "no arbitrator running" from "arbitrator says gamepad".
    """

    owner: str = GAMEPAD
    seen_owner_msg: bool = False


def on_owner_msg(state: ArbitrationState, owner: str) -> None:
    """Update state from an incoming ``/teleop/owner`` message.

    Silently ignores unknown values (treats them as no change) rather
    than raising — a garbled message should never take down a teleop
    node. The ROS layer logs the drop.
    """
    if owner in _VALID_OWNERS:
        state.owner = owner
        state.seen_owner_msg = True


def should_publish(state: ArbitrationState, who: str) -> bool:
    """True if ``who`` (``GAMEPAD`` or ``WEB``) is the current owner.

    The default (no owner message yet) is ``GAMEPAD``, so a gamepad node
    always publishes until told otherwise.
    """
    return state.owner == who


def web_claim(state: ArbitrationState) -> str:
    """Web app requested control. Returns the owner string to publish.

    Idempotent: claiming when web already owns is a no-op (still returns
    ``WEB`` — the ROS layer may re-publish to re-latch).
    """
    state.owner = WEB
    state.seen_owner_msg = True
    return WEB


def web_release(state: ArbitrationState) -> str:
    """Web app released control (or disconnected). Returns owner to publish.

    Idempotent: releasing when gamepad already owns is a no-op (still
    returns ``GAMEPAD``).
    """
    state.owner = GAMEPAD
    return GAMEPAD
