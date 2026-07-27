"""Front-panel button sequencer — which info screen is on the face's panel.

Pure: no rclpy, no gpiozero, no clocks, no I/O. The node hands this one
``(event, t)`` per real button event plus a periodic ``TICK`` for the timeout
clocks, and publishes whatever screen comes back. Every timestamp is monotonic
seconds; this module never reads a clock itself.

Debounce and hold *detection* are not here — gpiozero's ``Button`` owns them
(``bounce_time`` / ``hold_time``), below userspace. What is here is everything
that decides what the operator sees:

- info button — press toggles the battery/address screen; a hold asks the host
  to switch the Pi between wifi station and hotspot mode.
- bluetooth button — press toggles the controller-status screen; a hold starts
  a pairing scan instead, and the release ending that hold is swallowed so it
  cannot toggle a screen back on.
- while a scan runs, either button cancels it, as does ``scan_timeout_s``, as
  does a controller actually connecting.
- while a network switch runs, neither button cancels it. The host is already
  committed to an ``nmcli`` call by then, so there is nothing to call off, and
  a panel handed back early would be showing a mode the radio has already left.

Toggling happens on **release**, not press, so a hold has its full ``hold_time``
to declare itself before anything appears on the panel.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class Screen(str, Enum):
    """What the panel should show. NONE hands it back to the face."""

    NONE = "none"
    BATTERY = "battery"
    BLUETOOTH = "bluetooth"
    #: The spinners are part of the face, so this renders no text either — but
    #: it is a distinct state because it latches /bluetooth/scanning and obeys
    #: its own, much longer, timeout.
    SCANNING = "scanning"
    #: The host is switching the radio between station and hotspot mode. Wears
    #: the same spinners as SCANNING (and renders no text, for the same reason),
    #: but it is the one state nothing can cancel.
    NETWORK_SWITCHING = "network_switching"
    #: How the switch turned out — the hotspot's credentials, or why it failed.
    NETWORK_INFO = "network_info"


class Event(Enum):
    """One thing that happened. TICK carries no edge, only the current time."""

    INFO_PRESS = "info_press"
    INFO_RELEASE = "info_release"
    INFO_HOLD = "info_hold"
    BT_PRESS = "bt_press"
    BT_RELEASE = "bt_release"
    BT_HOLD = "bt_hold"
    TICK = "tick"


def wants_spinners(screen: Screen) -> bool:
    """Does this screen want the face wearing its scanning animation?

    A function rather than the node testing ``is Screen.SCANNING`` inline, so
    that "which states spin" has one definition and adding a third cannot leave
    the publisher behind.
    """
    return screen in (Screen.SCANNING, Screen.NETWORK_SWITCHING)


@dataclass(frozen=True)
class ScreenConfig:
    #: Only the number the bluetooth screen advertises. gpiozero owns the clock
    #: that actually measures the hold; this is passed through so the panel can
    #: never promise a hold the buttons do not honour.
    hold_s: float = 3.0
    #: An info screen hands the panel back to the face this long after it
    #: went up.
    screen_timeout_s: float = 6.0
    #: Backstop for a pairing scan that finds nothing.
    scan_timeout_s: float = 30.0
    #: Backstop for a network switch the host never reports the end of. Falls
    #: to NETWORK_INFO rather than NONE, so the panel says what went wrong
    #: instead of quietly going back to the face.
    network_timeout_s: float = 60.0
    #: The result screen holds longer than an ordinary one — it carries a
    #: password somebody has to type into a phone.
    network_info_timeout_s: float = 20.0


def pull_kwargs(*, active_low: bool, bias_pull_up: bool) -> dict:
    """Map buttons.yaml's wiring keys onto ``gpiozero.Button``'s pull arguments.

    gpiozero splits what the config states as two independent booleans across
    two mutually-constrained arguments: ``active_state`` is rejected unless
    ``pull_up is None``, and required when it is. Hence the table rather than
    two separate translations.

    Lives in the pure module, not in the device layer, so it is testable in a
    container with no gpiozero installed.
    """
    if bias_pull_up and not active_low:
        raise ValueError(
            "active_low: false with bias_pull_up: true is not a wiring — an "
            "internal pull-up holds the line high, so an active-high button "
            "would read as permanently pressed. Use active_low: true (switch "
            "to ground, the documented build) or bias_pull_up: false."
        )
    if bias_pull_up:
        return {"pull_up": True}  # switch to ground, SoC pull-up
    if not active_low:
        return {"pull_up": False}  # switch to 3V3, SoC pull-down
    return {"pull_up": None, "active_state": False}  # external pull-up


class ScreenSequencer:
    def __init__(self, config: ScreenConfig | None = None) -> None:
        self._config = config or ScreenConfig()
        self._screen = Screen.NONE
        self._entered_at = 0.0
        self._changed = False
        # Swallow the next release of this button (its press has already been
        # spent on starting or cancelling a scan).
        self._suppress_info_release = False
        self._suppress_bt_release = False
        # Latched by the first press of each button, never cleared. Guards
        # against a button physically jammed down when the node starts:
        # gpiozero does not replay initial state into a handler attached after
        # construction, but the hold thread is already running, so `when_held`
        # can fire hold_s later for a press nobody saw. Once a real press has
        # been seen the guard has done its job — it must NOT re-arm on release,
        # or it would drop the hold in the release-before-hold race below.
        self._info_press_seen = False
        self._bt_press_seen = False

    @property
    def screen(self) -> Screen:
        return self._screen

    @property
    def changed(self) -> bool:
        """True iff the most recent ``apply`` moved the screen."""
        return self._changed

    def apply(self, event: Event, t: float) -> Screen:
        self._changed = False
        handler = {
            Event.INFO_PRESS: self._on_info_press,
            Event.INFO_RELEASE: self._on_info_release,
            Event.INFO_HOLD: self._on_info_hold,
            Event.BT_PRESS: self._on_bt_press,
            Event.BT_RELEASE: self._on_bt_release,
            Event.BT_HOLD: self._on_bt_hold,
            Event.TICK: self._on_tick,
        }[event]
        handler(t)
        return self._screen

    def on_controller_seen(self, t: float) -> Screen:
        """A controller connected — a running scan has succeeded."""
        self._changed = False
        if self._screen is Screen.SCANNING:
            self._set(Screen.NONE, t)
        return self._screen

    def on_network_result(self, t: float) -> Screen:
        """The host finished a switch, one way or the other.

        Only acts from NETWORK_SWITCHING: a result that arrives after
        ``network_timeout_s`` has already given up must not seize a panel the
        operator has moved on from. The screen it lands on renders from the
        node's *current* cached state, so a late result still corrects the text
        while NETWORK_INFO is up.
        """
        self._changed = False
        if self._screen is Screen.NETWORK_SWITCHING:
            self._set(Screen.NETWORK_INFO, t)
        return self._screen

    # --- handlers ---------------------------------------------------------

    def _on_info_press(self, t: float) -> None:
        self._info_press_seen = True
        self._suppress_info_release = False  # a fresh press clears a stale flag
        if self._screen is Screen.NETWORK_SWITCHING:
            # Swallowed, but NOT a cancel — see the module docstring.
            self._suppress_info_release = True
            return
        if self._screen is Screen.SCANNING:
            # The press *is* the cancel, so its release must not go on to open
            # this button's screen.
            self._suppress_info_release = True
            self._set(Screen.NONE, t)

    def _on_info_release(self, t: float) -> None:
        if not self._info_press_seen:
            return
        if self._suppress_info_release:
            self._suppress_info_release = False
            return
        self._toggle(Screen.BATTERY, t)

    def _on_info_hold(self, t: float) -> None:
        """Ask the host to switch network mode.

        Same shape as ``_on_bt_hold``, including the suppress-release flag that
        makes both gpiozero orderings land here, and the press-seen guard —
        which matters more on this button than on the other one. gpiozero's hold
        thread runs from construction, so a button physically jammed down at
        startup would otherwise fire a hold nobody asked for, and this hold
        takes the robot off the network.
        """
        if not self._info_press_seen:
            return
        if self._screen is Screen.NETWORK_SWITCHING:
            # Already in flight. Returning without _set also means the timeout
            # clock is not restarted by leaning on the button.
            return
        self._suppress_info_release = True
        self._set(Screen.NETWORK_SWITCHING, t)

    def _on_bt_press(self, t: float) -> None:
        self._bt_press_seen = True
        self._suppress_bt_release = False
        if self._screen is Screen.NETWORK_SWITCHING:
            self._suppress_bt_release = True
            return
        if self._screen is Screen.SCANNING:
            self._suppress_bt_release = True
            self._set(Screen.NONE, t)

    def _on_bt_release(self, t: float) -> None:
        if not self._bt_press_seen:
            return
        if self._suppress_bt_release:
            self._suppress_bt_release = False
            return
        self._toggle(Screen.BLUETOOTH, t)

    def _on_bt_hold(self, t: float) -> None:
        if not self._bt_press_seen:
            return
        if self._screen is Screen.SCANNING:
            return
        if self._screen is Screen.NETWORK_SWITCHING:
            # No pairing scan on top of a switch: the radio is about to flap,
            # and a scan started across it would find nothing anyway.
            return
        # gpiozero fires this from the hold thread and `when_released` from the
        # pin thread, so within a hair of hold_time the two can arrive in either
        # order. Arming the flag either way makes both orders end in SCANNING:
        # hold-then-release swallows the release here, release-then-hold has
        # already toggled the bluetooth screen and this overrides it. The
        # left-over flag is cleared by the next press.
        self._suppress_bt_release = True
        self._set(Screen.SCANNING, t)

    def _on_tick(self, t: float) -> None:
        if self._screen is Screen.SCANNING:
            # A scan deliberately ignores screen_timeout_s: it is a job in
            # progress, not something the operator is reading.
            if t - self._entered_at >= self._config.scan_timeout_s:
                self._set(Screen.NONE, t)
        elif self._screen is Screen.NETWORK_SWITCHING:
            # A job in progress too, but it ends on the result screen: this
            # backstop only ever fires when the host never answered, and that
            # is exactly the case the operator most needs told about.
            if t - self._entered_at >= self._config.network_timeout_s:
                self._set(Screen.NETWORK_INFO, t)
        elif self._screen is Screen.NETWORK_INFO:
            if t - self._entered_at >= self._config.network_info_timeout_s:
                self._set(Screen.NONE, t)
        elif self._screen is not Screen.NONE:
            if t - self._entered_at >= self._config.screen_timeout_s:
                self._set(Screen.NONE, t)

    # --- helpers ----------------------------------------------------------

    def _toggle(self, screen: Screen, t: float) -> None:
        self._set(Screen.NONE if self._screen is screen else screen, t)

    def _set(self, screen: Screen, t: float) -> None:
        if screen is self._screen:
            return
        self._screen = screen
        self._entered_at = t  # every transition restarts the timeout clock
        self._changed = True
