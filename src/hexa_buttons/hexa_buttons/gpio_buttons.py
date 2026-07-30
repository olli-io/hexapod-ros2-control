"""The two front-panel buttons, as gpiozero devices.

The only module in this package that touches hardware, and the only one that
imports gpiozero — lazily, inside ``open_buttons``, so that:

- the pure modules (and therefore the pytest suites) import cleanly in the sim
  container, which has no gpiozero installed; and
- a missing library lands on the same "log it and stay inert" path as a missing
  chip, rather than killing the process at import.

gpiozero is doing real work here: edge delivery, debounce (``bounce_time``) and
the hold clock (``hold_time`` + ``when_held``) all come from it, which is why
this package has no polling loop and no debounce code of its own. Both buttons
carry a hold: the info button's switches network mode, the bluetooth button's
starts a pairing scan.
"""
from __future__ import annotations

import re
from typing import Callable

from .screen_logic import Event, pull_kwargs


class ButtonHardwareError(RuntimeError):
    """Anything that stops the lines being claimed.

    Never fatal to bringup: the buttons are a convenience fitting, so the node
    catches this, logs it, and runs on inert.
    """


def _lgpio_factory(chip_path: str):
    """An LGPIOFactory pinned to the chip buttons.yaml names.

    Deterministic on purpose. gpiozero's default factory search is
    lgpio -> rpigpio -> pigpio -> native, and the tail of that list is dangerous
    on this robot: ``rpigpio`` does not work on a Pi 5 at all, ``pigpio`` needs a
    daemon the container does not run, and ``native`` maps BCM283x registers
    directly — on a Pi 5's RP1 that does not fail, it reads garbage. Naming the
    factory makes a wrong environment an error at startup instead of a button
    that silently never responds.

    The container is handed exactly one chip node (``${GPIO_CHIP}`` mapped to
    /dev/gpiochip0 by docker-compose.robot.yaml), so N is always 0 here. Parsing
    it from the configured path keeps ``gpio_chip`` meaningful rather than
    decorative — and note it is the *container-side* path, which is fixed
    regardless of what the host calls its chip.

    ``LGPIOFactory(chip=N)`` cannot be used to do that, because gpiozero 2.0
    (Ubuntu 24.04's python3-gpiozero, what the robot image installs) throws its
    own argument away::

        def __init__(self, chip=None):
            super().__init__()
            chip = 4 if (self._get_revision() & 0xff0) >> 4 == 0x17 else 0
            self._handle = lgpio.gpiochip_open(chip)

    0x17 is a BCM2712, so on a Pi 5 it always opens /dev/gpiochip**4** whatever
    it was asked for. On Raspberry Pi OS that happens to work — udev leaves a
    gpiochip4 -> gpiochip0 compatibility symlink from the 6.1 kernel, where the
    RP1 really was chip 4. Inside the container there is no such symlink, only
    the one device node compose maps in, so the open fails with 'can not open
    gpiochip' and every press is silently dropped (the node logs the error and
    runs on inert).

    Hence constructing the factory the long way round: everything
    ``LGPIOFactory.__init__`` does, with the chip number we were actually given.
    ``LocalPiFactory.__init__`` is called directly to skip the overriding line —
    the base initialisation still runs, just not the part that ignores us.
    """
    import lgpio
    from gpiozero.pins.lgpio import LGPIOFactory, LGPIOPin
    from gpiozero.pins.local import LocalPiFactory

    match = re.fullmatch(r"/dev/gpiochip(\d+)", chip_path)
    if match is None:
        raise ButtonHardwareError(f"gpio_chip {chip_path!r} is not /dev/gpiochipN")
    chip = int(match.group(1))

    class PinnedLGPIOFactory(LGPIOFactory):
        """An LGPIOFactory that honours the chip it is handed."""

        def __init__(self) -> None:
            LocalPiFactory.__init__(self)
            self._handle = lgpio.gpiochip_open(chip)
            self._chip = chip
            self.pin_class = LGPIOPin

    factory = PinnedLGPIOFactory()
    # Cheap insurance against a future gpiozero reshuffling __init__ under us:
    # the whole point of this class is the chip number, so assert we got it.
    if factory.chip != chip:
        factory.close()
        raise ButtonHardwareError(
            f"pinned factory opened chip {factory.chip}, not {chip}"
        )
    return factory


class Buttons:
    """Owns the two devices and the pin factory, so they can all be released."""

    def __init__(self, factory, buttons) -> None:
        self._factory = factory
        self._buttons = buttons

    def close(self) -> None:
        """Release the lines and stop gpiozero's hold threads.

        Called from main()'s finally: a leaked line request would make the next
        `hexa robot restart` fail to claim them.
        """
        for button in self._buttons:
            button.close()
        self._factory.close()


def open_buttons(
    *,
    chip_path: str,
    info_line: int,
    bluetooth_line: int,
    active_low: bool,
    bias_pull_up: bool,
    debounce_s: float,
    hold_s: float,
    on_event: Callable[[Event], None],
) -> Buttons:
    """Claim both lines and route their edges into ``on_event``.

    ``on_event`` is called from gpiozero's own threads — see button_node's
    ``_enqueue``, which is all it is allowed to do.
    """
    try:
        from gpiozero import Button

        pull = pull_kwargs(active_low=active_low, bias_pull_up=bias_pull_up)
        factory = _lgpio_factory(chip_path)
        info = Button(
            info_line,
            bounce_time=debounce_s,
            hold_time=hold_s,
            hold_repeat=False,
            pin_factory=factory,
            **pull,
        )
        bluetooth = Button(
            bluetooth_line,
            bounce_time=debounce_s,
            hold_time=hold_s,
            hold_repeat=False,
            pin_factory=factory,
            **pull,
        )
    except ImportError as exc:
        raise ButtonHardwareError(f"gpiozero/lgpio not installed: {exc}") from exc
    except ButtonHardwareError:
        raise
    except Exception as exc:
        # Deliberately broad. gpiozero raises BadPinFactory / PinInvalidPin /
        # GPIOPinInUse (all GPIOZeroError), pull_kwargs raises ValueError, and
        # lgpio surfaces a bare lgpio.error or an OSError from gpiochip_open.
        # Every one of them means "no buttons", never "no robot".
        raise ButtonHardwareError(f"{type(exc).__name__}: {exc}") from exc

    # Handlers are attached AFTER construction on purpose: gpiozero does not
    # replay a device's initial state into a handler that did not exist yet, so
    # a button already held at startup produces no press. Its hold thread is
    # already running, though — ScreenSequencer's press-seen guard covers that.
    info.when_pressed = lambda: on_event(Event.INFO_PRESS)
    info.when_released = lambda: on_event(Event.INFO_RELEASE)
    info.when_held = lambda: on_event(Event.INFO_HOLD)
    bluetooth.when_pressed = lambda: on_event(Event.BT_PRESS)
    bluetooth.when_released = lambda: on_event(Event.BT_RELEASE)
    bluetooth.when_held = lambda: on_event(Event.BT_HOLD)
    return Buttons(factory, (info, bluetooth))
