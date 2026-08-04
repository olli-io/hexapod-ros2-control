"""The sysfs PWM seam: claim a channel, put a square wave on it, let it go.

Impure but stdlib-only, which is what lets the same code drive the buzzer from
inside the container (PWM tree bind-mounted at /pwm) and from the host's boot
and shutdown units.

Not re-exported from the package root, so importing `hexa_buzzer` stays free of
anything that touches the filesystem.
"""
from __future__ import annotations

import glob
import os
import time

# The Pi 5's RP1 PWM0 block, by platform address rather than pwmchipN number:
# that number is kernel probe order and has moved between releases.
DEFAULT_PWM_DEV = "/sys/bus/platform/devices/1f00098000.pwm/pwm"

# GPIO12 is channel 0 of that block on a Pi 5, and of pwmchip0 on a Pi 4.
DEFAULT_CHANNEL = 0

# Only a cold boot can outrun the RP1 PWM driver's probe; anything later passes
# 0 and fails fast instead of sitting in a retry loop.
DEFAULT_WAIT_S = 0.0

# udev applies the exported directory's permissions asynchronously, so it can
# exist a moment before it is usable.
_EXPORT_TIMEOUT_S = 2.0


class BuzzerHardwareError(RuntimeError):
    """Anything that stops a tune being played. Never fatal — the buzzer is an
    optional fitting, so every failure here means silence and nothing else."""


def _write(path: str, value: object) -> None:
    try:
        with open(path, "w") as sysfs:
            sysfs.write(str(value))
    except OSError as exc:
        raise BuzzerHardwareError(f"cannot write {path}: {exc}") from exc


def _hush(path: str, value: object) -> None:
    try:
        with open(path, "w") as sysfs:
            sysfs.write(str(value))
    except OSError:
        pass


def find_chip(pwm_dev: str = DEFAULT_PWM_DEV, wait_s: float = DEFAULT_WAIT_S) -> str:
    """The single pwmchip directory under `pwm_dev`, waiting up to `wait_s`."""
    deadline = time.monotonic() + wait_s
    while True:
        for chip in sorted(glob.glob(os.path.join(pwm_dev, "pwmchip*"))):
            if os.path.isdir(chip):
                return chip
        if time.monotonic() >= deadline:
            raise BuzzerHardwareError(
                f"no pwmchip under {pwm_dev} — is dtoverlay=pwm-2chan in "
                f"/boot/firmware/config.txt, and is the tree mounted into this "
                f"container?"
            )
        time.sleep(0.5)


class Channel:
    """One exported PWM channel, silent and released when the block exits.

    Held only for the length of a tune: the container, the host units and
    `hexa robot play-tune` share this one channel, so nobody may own it for a
    lifetime.
    """

    def __init__(
        self,
        pwm_dev: str = DEFAULT_PWM_DEV,
        channel: int = DEFAULT_CHANNEL,
        wait_s: float = DEFAULT_WAIT_S,
    ) -> None:
        self.chip = find_chip(pwm_dev, wait_s)
        self.channel = channel
        self.path = os.path.join(self.chip, f"pwm{channel}")
        self._exported = False

    def __enter__(self) -> "Channel":
        if os.path.isdir(self.path):
            return self

        _write(os.path.join(self.chip, "export"), self.channel)
        self._exported = True
        # Python does not call __exit__ when __enter__ raises, so without this
        # an export that never materialised would leave the channel claimed.
        try:
            deadline = time.monotonic() + _EXPORT_TIMEOUT_S
            while not os.path.isdir(self.path):
                if time.monotonic() >= deadline:
                    raise BuzzerHardwareError(
                        f"{self.path} never appeared after exporting channel "
                        f"{self.channel}"
                    )
                time.sleep(0.05)
        except BaseException:
            self.close()
            raise
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def close(self) -> None:
        """Silence the buzzer and release the channel. Errors are swallowed:
        this runs on the way out of failures too."""
        _hush(os.path.join(self.path, "enable"), 0)
        if self._exported:
            # Only if we exported it — otherwise the channel is someone else's.
            _hush(os.path.join(self.chip, "unexport"), self.channel)
            self._exported = False

    def tone(self, hz: int) -> None:
        """Start a square wave, or silence for `hz == 0`."""
        if hz <= 0:
            _write(os.path.join(self.path, "enable"), 0)
            return

        period_ns = 1_000_000_000 // hz
        # Disable first: the kernel rejects a duty_cycle exceeding the period
        # still in force, so a descending run of notes fails without this.
        _write(os.path.join(self.path, "enable"), 0)
        _write(os.path.join(self.path, "period"), period_ns)
        # 50% duty = square wave, the loudest a passive buzzer gets.
        _write(os.path.join(self.path, "duty_cycle"), period_ns // 2)
        _write(os.path.join(self.path, "enable"), 1)
