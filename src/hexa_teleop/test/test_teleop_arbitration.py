from hexa_teleop.teleop_arbitration import (
    GAMEPAD,
    WEB,
    ArbitrationState,
    on_owner_msg,
    should_publish,
    web_claim,
    web_release,
)


def test_default_owner_is_gamepad():
    state = ArbitrationState()
    assert state.owner == GAMEPAD
    assert state.seen_owner_msg is False


def test_gamepad_publishes_by_default_no_msg_seen():
    state = ArbitrationState()
    assert should_publish(state, GAMEPAD) is True
    assert should_publish(state, WEB) is False


def test_on_owner_msg_web_switches_owner():
    state = ArbitrationState()
    on_owner_msg(state, WEB)
    assert state.owner == WEB
    assert state.seen_owner_msg is True
    assert should_publish(state, GAMEPAD) is False
    assert should_publish(state, WEB) is True


def test_on_owner_msg_gamepad_switches_back():
    state = ArbitrationState()
    on_owner_msg(state, WEB)
    on_owner_msg(state, GAMEPAD)
    assert state.owner == GAMEPAD
    assert should_publish(state, GAMEPAD) is True
    assert should_publish(state, WEB) is False


def test_on_owner_msg_unknown_value_ignored():
    state = ArbitrationState()
    on_owner_msg(state, WEB)
    on_owner_msg(state, "nonsense")
    assert state.owner == WEB
    assert state.seen_owner_msg is True


def test_web_claim_sets_web_and_returns_web():
    state = ArbitrationState()
    result = web_claim(state)
    assert result == WEB
    assert state.owner == WEB
    assert state.seen_owner_msg is True


def test_web_claim_idempotent():
    state = ArbitrationState()
    web_claim(state)
    result = web_claim(state)
    assert result == WEB
    assert state.owner == WEB


def test_web_release_sets_gamepad_and_returns_gamepad():
    state = ArbitrationState()
    web_claim(state)
    result = web_release(state)
    assert result == GAMEPAD
    assert state.owner == GAMEPAD


def test_web_release_idempotent():
    state = ArbitrationState()
    result = web_release(state)
    assert result == GAMEPAD
    assert state.owner == GAMEPAD


def test_claim_then_release_round_trip():
    state = ArbitrationState()
    assert should_publish(state, GAMEPAD) is True
    web_claim(state)
    assert should_publish(state, GAMEPAD) is False
    assert should_publish(state, WEB) is True
    web_release(state)
    assert should_publish(state, GAMEPAD) is True
    assert should_publish(state, WEB) is False
