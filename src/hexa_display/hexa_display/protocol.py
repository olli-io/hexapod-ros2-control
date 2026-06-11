"""Framed binary UART protocol for the ESP32 OLED face.

Pure module: no rclpy, no pyserial. Mirrors the firmware protocol in
``hexapod-esp32-display`` byte for byte:

- **frame** — ``SOF(0xA5) | LEN u16 LE | CMD u8 | PAYLOAD[LEN] | CRC u16 BE``.
- **CRC** — CRC-16/CCITT-FALSE (poly ``0x1021``, init ``0xFFFF``, no
  reflection, no xor-out) computed over ``LEN + CMD + PAYLOAD``.
  ``crc16(b"123456789") == 0x29B1``.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

SOF = 0xA5
MAX_PAYLOAD = 1024
# SOF + LEN(2) + CMD(1) + CRC(2)
_OVERHEAD = 6


class Cmd(IntEnum):
    # Pi → ESP32
    PING = 0x01
    SET_EXPRESSION = 0x10
    SET_GAZE = 0x11
    TRIGGER_BLINK = 0x12
    QUERY_STATUS = 0x20
    # ESP32 → Pi
    ACK = 0x80
    NACK = 0x81
    STATUS = 0x82
    PONG = 0x83
    LOG = 0x8F


class Expression(IntEnum):
    NEUTRAL = 0
    HAPPY = 1
    SLEEPY = 2
    DEAD = 3
    GREEDY = 4
    WOOZY = 5
    ANGRY = 6
    LOVE = 7


class Gaze(IntEnum):
    CENTER = 0
    UP = 1
    DOWN = 2
    LEFT = 3
    RIGHT = 4
    UP_LEFT = 5
    UP_RIGHT = 6
    DOWN_LEFT = 7
    DOWN_RIGHT = 8


class NackReason(IntEnum):
    BAD_CRC = 0
    BAD_LEN = 1
    UNKNOWN_CMD = 2
    BAD_PAYLOAD = 3
    BUSY = 4


def crc16(data: bytes, init: int = 0xFFFF, poly: int = 0x1021) -> int:
    c = init
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def encode_frame(cmd: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} > {MAX_PAYLOAD}")
    body = struct.pack("<H", len(payload)) + bytes([cmd]) + payload
    c = crc16(body)
    return bytes([SOF]) + body + bytes([(c >> 8) & 0xFF, c & 0xFF])


def set_expression(expression: Expression) -> bytes:
    return encode_frame(Cmd.SET_EXPRESSION, bytes([Expression(expression)]))


def set_gaze(gaze: Gaze) -> bytes:
    return encode_frame(Cmd.SET_GAZE, bytes([Gaze(gaze)]))


def trigger_blink() -> bytes:
    return encode_frame(Cmd.TRIGGER_BLINK)


def ping(payload: bytes = b"") -> bytes:
    return encode_frame(Cmd.PING, payload)


@dataclass(frozen=True)
class Frame:
    cmd: int
    payload: bytes


def decode_frames(buf: bytes) -> tuple[list[Frame], bytes]:
    """Scan ``buf`` for complete frames; return (frames, leftover).

    Stateless: callers keep the leftover and prepend the next read.
    Resyncs by skipping to the next ``SOF`` byte; frames with a bad CRC
    or an oversized LEN are dropped and the scan resumes one byte past
    the bogus SOF (the dropped bytes may contain a real frame start).
    """
    frames: list[Frame] = []
    i = 0
    n = len(buf)
    while i < n:
        if buf[i] != SOF:
            i += 1
            continue
        if n - i < _OVERHEAD:
            break  # partial header — wait for more bytes
        length = buf[i + 1] | (buf[i + 2] << 8)
        if length > MAX_PAYLOAD:
            i += 1  # bogus header, resync
            continue
        end = i + _OVERHEAD + length
        if end > n:
            break  # partial frame — wait for more bytes
        body = buf[i + 1 : i + 4 + length]
        got = (buf[end - 2] << 8) | buf[end - 1]
        if got != crc16(body):
            i += 1  # corrupt frame, resync
            continue
        frames.append(Frame(cmd=buf[i + 3], payload=bytes(buf[i + 4 : i + 4 + length])))
        i = end
    return frames, bytes(buf[i:])
