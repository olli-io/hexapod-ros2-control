"""Byte transports for the ESP32 face link.

Pure module: importable without pyserial (``SerialTransport`` imports
it lazily inside ``open()``) and without rclpy. The node picks the
transport from the ``transport`` parameter: ``serial`` on the robot,
``stub`` in sim, where it decodes outgoing frames and logs them so
``pod sim`` shows the face transitions.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Callable

from .protocol import Cmd, Expression, Frame, Gaze, decode_frames


class TransportError(Exception):
    """Raised by transports on open/read/write failure."""


class Transport(ABC):
    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def write(self, data: bytes) -> None: ...

    @abstractmethod
    def read(self) -> bytes:
        """Non-blocking: return whatever bytes are pending (b"" if none)."""

    @property
    @abstractmethod
    def is_open(self) -> bool: ...


class SerialTransport(Transport):
    def __init__(self, device: str, baud: int) -> None:
        self._device = device
        self._baud = baud
        self._serial = None

    def open(self) -> None:
        try:
            import serial  # lazy: module stays importable without pyserial
        except ImportError as e:
            raise TransportError(f"pyserial not installed: {e}") from e

        try:
            self._serial = serial.Serial(
                self._device, self._baud, timeout=0, write_timeout=0.5
            )
        except (serial.SerialException, OSError, ValueError) as e:
            self._serial = None
            raise TransportError(f"open {self._device}: {e}") from e

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def write(self, data: bytes) -> None:
        if self._serial is None:
            raise TransportError("write on closed transport")
        try:
            self._serial.write(data)
        except Exception as e:
            self.close()
            raise TransportError(f"write {self._device}: {e}") from e

    def read(self) -> bytes:
        if self._serial is None:
            raise TransportError("read on closed transport")
        try:
            pending = self._serial.in_waiting
            return self._serial.read(pending) if pending else b""
        except Exception as e:
            self.close()
            raise TransportError(f"read {self._device}: {e}") from e

    @property
    def is_open(self) -> bool:
        return self._serial is not None


def _describe(frame: Frame) -> str:
    try:
        name = Cmd(frame.cmd).name
    except ValueError:
        return f"CMD 0x{frame.cmd:02X} payload={frame.payload.hex()}"
    if frame.cmd == Cmd.SET_EXPRESSION and len(frame.payload) == 1:
        try:
            return f"{name} {Expression(frame.payload[0]).name}"
        except ValueError:
            pass
    if frame.cmd == Cmd.SET_GAZE and len(frame.payload) == 1:
        try:
            return f"{name} {Gaze(frame.payload[0]).name}"
        except ValueError:
            pass
    return name if not frame.payload else f"{name} payload={frame.payload.hex()}"


class StubTransport(Transport):
    """Decodes written frames and reports them via ``log_fn``.

    Used in sim and in tests: ``frames`` records every decoded frame
    so assertions can check exactly what the node sent.
    """

    def __init__(self, log_fn: Callable[[str], None] | None = None) -> None:
        self._log_fn = log_fn or (lambda _msg: None)
        self._open = False
        self._leftover = b""
        self.frames: list[Frame] = []

    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    def write(self, data: bytes) -> None:
        if not self._open:
            raise TransportError("write on closed transport")
        frames, self._leftover = decode_frames(self._leftover + data)
        for frame in frames:
            self.frames.append(frame)
            self._log_fn(_describe(frame))

    def read(self) -> bytes:
        if not self._open:
            raise TransportError("read on closed transport")
        return b""

    @property
    def is_open(self) -> bool:
        return self._open
