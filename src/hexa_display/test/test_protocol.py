import pytest

from hexa_display.protocol import (
    MAX_PAYLOAD,
    SOF,
    Cmd,
    Expression,
    Frame,
    Gaze,
    crc16,
    decode_frames,
    encode_frame,
    ping,
    set_expression,
    set_gaze,
    trigger_blink,
)


def test_crc16_check_value():
    # CRC-16/CCITT-FALSE reference vector.
    assert crc16(b"123456789") == 0x29B1


def test_set_expression_happy_is_byte_exact():
    frame = set_expression(Expression.HAPPY)
    # SOF, LEN=1 LE, CMD=0x10, payload=0x01, then CRC big-endian.
    assert frame[:5] == bytes([0xA5, 0x01, 0x00, 0x10, 0x01])
    crc = crc16(bytes([0x01, 0x00, 0x10, 0x01]))
    assert frame[5:] == bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    assert len(frame) == 7


def test_empty_payload_frame_layout():
    frame = trigger_blink()
    assert frame[:4] == bytes([SOF, 0x00, 0x00, Cmd.TRIGGER_BLINK])
    assert len(frame) == 6


def test_payload_too_long_rejected():
    with pytest.raises(ValueError):
        encode_frame(Cmd.PING, b"\x00" * (MAX_PAYLOAD + 1))


@pytest.mark.parametrize(
    "frame,cmd,payload",
    [
        (ping(b"hi"), Cmd.PING, b"hi"),
        (set_expression(Expression.DEAD), Cmd.SET_EXPRESSION, bytes([3])),
        (set_gaze(Gaze.DOWN_RIGHT), Cmd.SET_GAZE, bytes([8])),
        (trigger_blink(), Cmd.TRIGGER_BLINK, b""),
    ],
)
def test_encode_decode_round_trip(frame, cmd, payload):
    frames, leftover = decode_frames(frame)
    assert leftover == b""
    assert frames == [Frame(cmd=cmd, payload=payload)]


def test_decode_multiple_frames_in_one_buffer():
    buf = set_expression(Expression.NEUTRAL) + set_gaze(Gaze.UP) + ping()
    frames, leftover = decode_frames(buf)
    assert [f.cmd for f in frames] == [Cmd.SET_EXPRESSION, Cmd.SET_GAZE, Cmd.PING]
    assert leftover == b""


def test_decode_skips_garbage_before_sof():
    buf = b"\x00\xffjunk" + set_gaze(Gaze.LEFT)
    frames, leftover = decode_frames(buf)
    assert frames == [Frame(cmd=Cmd.SET_GAZE, payload=bytes([Gaze.LEFT]))]
    assert leftover == b""


def test_decode_split_frame_keeps_leftover():
    frame = set_expression(Expression.WOOZY)
    frames, leftover = decode_frames(frame[:4])
    assert frames == []
    assert leftover == frame[:4]
    frames, leftover = decode_frames(leftover + frame[4:])
    assert frames == [
        Frame(cmd=Cmd.SET_EXPRESSION, payload=bytes([Expression.WOOZY]))
    ]
    assert leftover == b""


def test_decode_drops_corrupt_crc_and_resyncs():
    bad = bytearray(set_expression(Expression.HAPPY))
    bad[-1] ^= 0xFF
    buf = bytes(bad) + set_expression(Expression.SLEEPY)
    frames, leftover = decode_frames(buf)
    assert frames == [
        Frame(cmd=Cmd.SET_EXPRESSION, payload=bytes([Expression.SLEEPY]))
    ]
    assert leftover == b""


def test_decode_resyncs_on_bogus_length():
    # SOF followed by an impossible LEN, then a real frame.
    buf = bytes([SOF, 0xFF, 0xFF]) + trigger_blink()
    frames, leftover = decode_frames(buf)
    assert frames == [Frame(cmd=Cmd.TRIGGER_BLINK, payload=b"")]
    assert leftover == b""


def test_decode_partial_header_waits():
    frames, leftover = decode_frames(bytes([SOF, 0x01]))
    assert frames == []
    assert leftover == bytes([SOF, 0x01])
