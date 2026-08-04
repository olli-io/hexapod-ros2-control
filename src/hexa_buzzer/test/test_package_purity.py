"""The import seam that lets one player serve two very different callers.

The host's boot and shutdown units run `player` on a Pi with no ROS environment,
which holds only while neither it, `pwm`, nor the package root reaches for
rclpy. Easy to break with one convenience import, and invisible until a shutdown
goes silent.

Checked in a fresh interpreter: by the time a suite runs, anything may already
have imported rclpy, and the question is what *this* import pulls in.
"""
import subprocess
import sys

import pytest

_PROBE = (
    "import sys; import {module}; "
    "sys.exit(1 if [m for m in sys.modules if m == 'rclpy' "
    "or m.startswith('rclpy.')] else 0)"
)


@pytest.mark.parametrize(
    "module",
    [
        "hexa_buzzer",  # also what lets these suites run in the sim container
        "hexa_buzzer.catalog",
        "hexa_buzzer.pwm",
        "hexa_buzzer.player",
    ],
)
def test_importing_it_does_not_pull_in_rclpy(module):
    probe = subprocess.run(
        [sys.executable, "-c", _PROBE.format(module=module)],
        capture_output=True,
        text=True,
    )
    assert probe.returncode == 0, (
        f"importing {module} pulled in rclpy, which breaks the host systemd "
        f"units.\n{probe.stderr}"
    )
