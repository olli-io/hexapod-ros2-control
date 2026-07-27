"""Unit tests for the front-panel screen sequencer.

No GPIO, no ROS graph, no gpiozero: the state machine is fed (event, time)
pairs directly. Debounce and hold detection are gpiozero's job now, so there is
nothing here to step at a sample rate — which is most of the readability win
over the polled version this replaced.
"""
import pytest

from hexa_buttons import Event, Screen, ScreenConfig, ScreenSequencer, pull_kwargs


def sequencer(**overrides):
    return ScreenSequencer(ScreenConfig(**overrides))


def tap(seq, press, release, t):
    """A press/release pair 0.1 s apart, as a real tap arrives."""
    seq.apply(press, t)
    return seq.apply(release, t + 0.1)


def info_tap(seq, t):
    return tap(seq, Event.INFO_PRESS, Event.INFO_RELEASE, t)


def bt_tap(seq, t):
    return tap(seq, Event.BT_PRESS, Event.BT_RELEASE, t)


def bt_hold(seq, t, hold_s=3.0):
    """Press, hold past hold_time, then let go — the real event order."""
    seq.apply(Event.BT_PRESS, t)
    seq.apply(Event.BT_HOLD, t + hold_s)
    return seq.apply(Event.BT_RELEASE, t + hold_s + 0.1)


# --- toggling ---------------------------------------------------------------


def test_info_tap_opens_the_battery_screen():
    seq = sequencer()
    assert info_tap(seq, 1.0) is Screen.BATTERY


def test_second_info_tap_closes_it():
    seq = sequencer()
    info_tap(seq, 1.0)
    assert info_tap(seq, 2.0) is Screen.NONE


def test_bluetooth_tap_toggles_its_screen():
    seq = sequencer()
    assert bt_tap(seq, 1.0) is Screen.BLUETOOTH
    assert bt_tap(seq, 2.0) is Screen.NONE


def test_other_button_swaps_screens():
    seq = sequencer()
    assert info_tap(seq, 1.0) is Screen.BATTERY
    assert bt_tap(seq, 2.0) is Screen.BLUETOOTH
    assert info_tap(seq, 3.0) is Screen.BATTERY


def test_press_alone_does_not_open_a_screen():
    """Toggling is on release, so a hold has its full hold_time to declare
    itself before anything hits the panel."""
    seq = sequencer()
    assert seq.apply(Event.INFO_PRESS, 1.0) is Screen.NONE


# --- timeouts ---------------------------------------------------------------


def test_screen_times_out_back_to_the_face():
    seq = sequencer(screen_timeout_s=6.0)
    info_tap(seq, 1.0)  # screen entered at t=1.1
    assert seq.apply(Event.TICK, 7.0) is Screen.BATTERY
    assert seq.apply(Event.TICK, 7.2) is Screen.NONE


def test_swap_restarts_the_timeout():
    seq = sequencer(screen_timeout_s=6.0)
    info_tap(seq, 1.0)
    seq.apply(Event.TICK, 6.0)
    bt_tap(seq, 6.5)  # entered at 6.6
    assert seq.apply(Event.TICK, 12.0) is Screen.BLUETOOTH


# --- pairing scan -----------------------------------------------------------


def test_holding_bluetooth_starts_a_scan():
    seq = sequencer()
    assert bt_hold(seq, 1.0) is Screen.SCANNING


def test_the_release_after_a_hold_is_swallowed():
    """Letting go of a hold must not toggle the bluetooth screen back on."""
    seq = sequencer()
    seq.apply(Event.BT_PRESS, 1.0)
    seq.apply(Event.BT_HOLD, 4.0)
    assert seq.apply(Event.BT_RELEASE, 4.1) is Screen.SCANNING


def test_release_before_hold_still_ends_in_scanning():
    """gpiozero fires when_held from the hold thread and when_released from the
    pin thread, so within a hair of hold_time they can arrive in either order.
    Both must end up scanning."""
    seq = sequencer()
    seq.apply(Event.BT_PRESS, 1.0)
    assert seq.apply(Event.BT_RELEASE, 4.0) is Screen.BLUETOOTH
    assert seq.apply(Event.BT_HOLD, 4.0) is Screen.SCANNING


def test_a_stale_suppress_flag_does_not_swallow_the_next_tap():
    """The inverted order above arms the suppress flag with no release left to
    spend it on. It must not eat a later, legitimate tap."""
    seq = sequencer()
    seq.apply(Event.BT_PRESS, 1.0)
    seq.apply(Event.BT_RELEASE, 4.0)
    seq.apply(Event.BT_HOLD, 4.0)  # -> SCANNING, flag armed and orphaned
    assert bt_tap(seq, 5.0) is Screen.NONE  # this press cancels the scan
    assert bt_tap(seq, 6.0) is Screen.BLUETOOTH  # and this one still works


def test_a_short_press_is_a_tap_not_a_scan():
    seq = sequencer()
    assert bt_tap(seq, 1.0) is Screen.BLUETOOTH


def test_info_press_cancels_a_scan_and_its_release_is_swallowed():
    seq = sequencer()
    bt_hold(seq, 1.0)
    assert seq.apply(Event.INFO_PRESS, 6.0) is Screen.NONE
    assert seq.apply(Event.INFO_RELEASE, 6.1) is Screen.NONE


def test_bluetooth_press_cancels_a_scan():
    seq = sequencer()
    bt_hold(seq, 1.0)
    assert seq.apply(Event.BT_PRESS, 6.0) is Screen.NONE
    assert seq.apply(Event.BT_RELEASE, 6.1) is Screen.NONE


def test_a_scan_times_out_after_scan_timeout_s():
    seq = sequencer(scan_timeout_s=30.0)
    bt_hold(seq, 1.0)  # scanning entered at t=4.0
    assert seq.apply(Event.TICK, 33.0) is Screen.SCANNING
    assert seq.apply(Event.TICK, 34.5) is Screen.NONE


def test_a_scan_ignores_the_screen_timeout():
    """A scan is a job in progress, not something the operator is reading."""
    seq = sequencer(screen_timeout_s=6.0, scan_timeout_s=30.0)
    bt_hold(seq, 1.0)
    assert seq.apply(Event.TICK, 20.0) is Screen.SCANNING


def test_a_reported_controller_ends_a_scan():
    seq = sequencer()
    bt_hold(seq, 1.0)
    assert seq.on_controller_seen(6.0) is Screen.NONE
    assert seq.changed


def test_a_reported_controller_does_not_disturb_other_screens():
    seq = sequencer()
    info_tap(seq, 1.0)
    assert seq.on_controller_seen(2.0) is Screen.BATTERY
    assert not seq.changed


# --- startup guard ----------------------------------------------------------


def test_a_hold_without_a_press_is_ignored():
    """A button jammed down when the node starts produces no press (gpiozero
    does not replay initial state into a later-attached handler) but its hold
    thread is already running and can still fire when_held."""
    seq = sequencer()
    assert seq.apply(Event.BT_HOLD, 3.0) is Screen.NONE


def test_a_release_without_a_press_is_ignored():
    seq = sequencer()
    assert seq.apply(Event.BT_RELEASE, 3.0) is Screen.NONE
    assert seq.apply(Event.INFO_RELEASE, 3.0) is Screen.NONE


def test_the_guard_lifts_after_the_first_real_press():
    seq = sequencer()
    seq.apply(Event.BT_HOLD, 1.0)  # jammed at startup, ignored
    seq.apply(Event.BT_RELEASE, 2.0)  # ditto
    assert bt_tap(seq, 3.0) is Screen.BLUETOOTH


# --- change flag ------------------------------------------------------------


def test_changed_is_true_only_on_the_transition():
    seq = sequencer()
    seq.apply(Event.INFO_PRESS, 1.0)
    assert not seq.changed
    seq.apply(Event.INFO_RELEASE, 1.1)
    assert seq.changed
    seq.apply(Event.TICK, 1.2)
    assert not seq.changed


# --- wiring translation -----------------------------------------------------


@pytest.mark.parametrize(
    "active_low, bias_pull_up, expected",
    [
        (True, True, {"pull_up": True}),  # switch to ground, SoC pull-up
        (False, False, {"pull_up": False}),  # switch to 3V3, SoC pull-down
        (True, False, {"pull_up": None, "active_state": False}),  # external pull-up
    ],
)
def test_pull_kwargs_maps_every_documented_wiring(active_low, bias_pull_up, expected):
    assert pull_kwargs(active_low=active_low, bias_pull_up=bias_pull_up) == expected


def test_pull_kwargs_rejects_pull_up_with_active_high():
    """An internal pull-up holds the line high, so an active-high button would
    read as permanently pressed."""
    with pytest.raises(ValueError):
        pull_kwargs(active_low=False, bias_pull_up=True)
