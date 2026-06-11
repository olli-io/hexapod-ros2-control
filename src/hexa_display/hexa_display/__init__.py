"""Face relay: robot state → ESP32 OLED expression/gaze commands.

Pure modules (importable without rclpy):

- :mod:`hexa_display.protocol` — frame codec mirroring the firmware.
- :mod:`hexa_display.expression_policy` — state → (expression, gaze).
- :mod:`hexa_display.transport` — serial / stub byte transports.

ROS glue lives in :mod:`hexa_display.display_node`.
"""

from .expression_policy import (
    BatteryMonitor,
    DisplayTarget,
    PolicyConfig,
    PolicyInputs,
    decide,
)
from .protocol import Cmd, Expression, Gaze, crc16, decode_frames, encode_frame
from .transport import SerialTransport, StubTransport, Transport, TransportError

__all__ = [
    "BatteryMonitor",
    "Cmd",
    "decide",
    "decode_frames",
    "DisplayTarget",
    "encode_frame",
    "Expression",
    "Gaze",
    "crc16",
    "PolicyConfig",
    "PolicyInputs",
    "SerialTransport",
    "StubTransport",
    "Transport",
    "TransportError",
]
