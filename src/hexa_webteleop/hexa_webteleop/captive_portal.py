"""What the teleop server does with a request that is not the webapp's.

Pure: no aiohttp, no rclpy, no filesystem. ``webteleop_node`` does the I/O;
this decides what a path means, so the rule is unit-testable without a server.

On the robot's own hotspot (``systemd/network-mode.sh``) NetworkManager's
dnsmasq answers *every* hostname with the robot, so requests that were never
meant for us arrive anyway: the connectivity probe a phone fires the moment it
joins a network, and whatever a stale tab or a mistyped name asks for. Both are
somebody who needs the controller, so neither is a 404 — anything that is not a
webapp file is sent to the root of the same host.

The redirect is doing two jobs. It answers the probe with something other than
the "there is internet here" the OS hoped for, which is what makes the phone
declare a captive portal and open its sign-in browser on the controller by
itself; and it puts the browser at ``/``, where the webapp's relative
``main.js`` and ``styles.css`` resolve. Serving the page inline under some other
path would load neither.
"""
from __future__ import annotations


def request_path(target: str) -> str:
    """The comparable path of a request target.

    Drops the query and fragment and strips a trailing slash, so a client's
    cache-busting decoration cannot change what a request means. The root stays
    ``/`` rather than becoming empty.
    """
    path = target.split("#", 1)[0].split("?", 1)[0]
    if len(path) > 1:
        path = path.rstrip("/")
    return path or "/"


def static_filename(path: str) -> str | None:
    """The file under the web directory this path asks for, or ``None``.

    ``None`` means "not a webapp asset" — the caller redirects to the app. The
    webapp is flat, so anything nested is somebody else's URL arriving through
    the hotspot's wildcard DNS, and refusing both separators and leading dots
    keeps a request from naming a path outside the directory.
    """
    name = path.lstrip("/")
    if not name or "/" in name or "\\" in name or name.startswith("."):
        return None
    return name
