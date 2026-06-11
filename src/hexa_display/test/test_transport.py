import pytest

from hexa_display.protocol import Cmd, Expression, Gaze, set_expression, set_gaze
from hexa_display.transport import (
    SerialTransport,
    StubTransport,
    TransportError,
)


def test_stub_decodes_and_logs_written_frames():
    lines: list[str] = []
    stub = StubTransport(log_fn=lines.append)
    stub.open()
    stub.write(set_expression(Expression.HAPPY))
    stub.write(set_gaze(Gaze.UP_LEFT))
    assert [f.cmd for f in stub.frames] == [Cmd.SET_EXPRESSION, Cmd.SET_GAZE]
    assert lines == ["SET_EXPRESSION HAPPY", "SET_GAZE UP_LEFT"]


def test_stub_handles_split_writes():
    stub = StubTransport()
    stub.open()
    frame = set_expression(Expression.SLEEPY)
    stub.write(frame[:3])
    assert stub.frames == []
    stub.write(frame[3:])
    assert len(stub.frames) == 1
    assert stub.frames[0].payload == bytes([Expression.SLEEPY])


def test_stub_rejects_io_when_closed():
    stub = StubTransport()
    with pytest.raises(TransportError):
        stub.write(b"\x00")
    with pytest.raises(TransportError):
        stub.read()
    stub.open()
    assert stub.is_open
    assert stub.read() == b""
    stub.close()
    assert not stub.is_open


class FakeSerial:
    """Minimal stand-in injected in place of a pyserial handle."""

    def __init__(self) -> None:
        self.written = b""
        self.rx = b""
        self.closed = False
        self.fail_write = False
        self.fail_read = False

    @property
    def in_waiting(self) -> int:
        if self.fail_read:
            raise OSError("device gone")
        return len(self.rx)

    def write(self, data: bytes) -> None:
        if self.fail_write:
            raise OSError("device gone")
        self.written += data

    def read(self, n: int) -> bytes:
        out, self.rx = self.rx[:n], self.rx[n:]
        return out

    def close(self) -> None:
        self.closed = True


def make_open_serial_transport() -> tuple[SerialTransport, FakeSerial]:
    transport = SerialTransport("/dev/fake", 921600)
    fake = FakeSerial()
    transport._serial = fake
    return transport, fake


def test_serial_write_and_read_pass_through():
    transport, fake = make_open_serial_transport()
    transport.write(b"\xa5\x01")
    assert fake.written == b"\xa5\x01"
    fake.rx = b"\x80\x10"
    assert transport.read() == b"\x80\x10"
    assert transport.read() == b""


def test_serial_errors_wrap_and_close():
    transport, fake = make_open_serial_transport()
    fake.fail_write = True
    with pytest.raises(TransportError):
        transport.write(b"\x00")
    assert not transport.is_open
    assert fake.closed

    transport, fake = make_open_serial_transport()
    fake.fail_read = True
    with pytest.raises(TransportError):
        transport.read()
    assert not transport.is_open


def test_serial_io_on_closed_transport_raises():
    transport = SerialTransport("/dev/fake", 921600)
    assert not transport.is_open
    with pytest.raises(TransportError):
        transport.write(b"\x00")
    with pytest.raises(TransportError):
        transport.read()


def test_serial_open_failure_raises_transport_error():
    # Holds with or without pyserial installed: a missing module and a
    # missing device must both surface as TransportError, so the node
    # can come up faceless and keep retrying.
    transport = SerialTransport("/dev/nonexistent-display-uart", 921600)
    with pytest.raises(TransportError):
        transport.open()
    assert not transport.is_open
