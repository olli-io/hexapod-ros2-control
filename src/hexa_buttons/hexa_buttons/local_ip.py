"""The address a phone would point at the web teleop UI.

The robot container runs with ``network_mode: host``, so the interfaces walked
here are the Pi's own — there is no container bridge address to filter out.

``getifaddrs`` has no stdlib binding, so this goes through the same raw-ioctl
route ``hexa_teleop/joy_publisher.py`` uses for the joystick. The ranking half
is pure and carries the tests; the ioctl half is a dozen lines and is verified
on hardware.
"""
from __future__ import annotations

import fcntl
import socket
import struct
from typing import Mapping, Sequence

# <bits/ioctls.h>
SIOCGIFFLAGS = 0x8913
SIOCGIFADDR = 0x8915

# <net/if.h>
IFF_UP = 0x1
IFF_LOOPBACK = 0x8

# struct ifreq is 16 bytes of name plus a 24-byte union on 64-bit Linux.
_IFREQ = "16s24x"
_IFNAMSIZ = 16


def rank_interfaces(addresses: Mapping[str, str], preferred: Sequence[str]) -> str:
    """Pick the address to show.

    The first address found on a ``preferred`` interface, in the order given;
    failing that, any other interface's; failing that, the empty string.
    Preference beats enumeration order — a machine that happens to list `eth0`
    first still shows `wlan0` if that is what the config asked for.
    """
    for name in preferred:
        address = addresses.get(name)
        if address:
            return address
    for name, address in addresses.items():
        if name not in preferred and address:
            return address
    return ""


def enumerate_ipv4() -> dict[str, str]:
    """``{interface: dotted quad}`` for every up, non-loopback IPv4 interface."""
    found: dict[str, str] = {}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for _index, name in socket.if_nameindex():
            try:
                flags = struct.unpack_from("H", _ioctl(sock, SIOCGIFFLAGS, name), 16)[0]
                if not flags & IFF_UP or flags & IFF_LOOPBACK:
                    continue
                # sockaddr_in sits at offset 16: family (2) + port (2) + addr.
                found[name] = socket.inet_ntoa(_ioctl(sock, SIOCGIFADDR, name)[20:24])
            except OSError:
                # EADDRNOTAVAIL for an interface with no IPv4 lease (Wi-Fi still
                # associating), ENODEV if it vanished mid-walk. Neither is an
                # error here — it just has no address to show.
                continue
    return found


def local_ipv4(preferred: Sequence[str]) -> str:
    return rank_interfaces(enumerate_ipv4(), preferred)


def _ioctl(sock: socket.socket, request: int, name: str) -> bytes:
    return fcntl.ioctl(
        sock.fileno(), request, struct.pack(_IFREQ, name.encode()[: _IFNAMSIZ - 1])
    )
