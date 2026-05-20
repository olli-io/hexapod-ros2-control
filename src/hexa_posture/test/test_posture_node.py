"""Regression tests for posture_node gating against /gait/state."""

from hexa_posture.posture_node import POSTURE_ACTIVE_STATES


def test_pause_trio_preserves_body_pose():
    # Regression: PAUSING zeroed body pose.z back to default because the
    # posture node treated `pausing` / `paused` / `resuming` as inactive
    # states and emitted IDENTITY. Pause must only affect the legs.
    for state in ("pausing", "paused", "resuming"):
        assert state in POSTURE_ACTIVE_STATES, (
            f"{state!r} missing from POSTURE_ACTIVE_STATES — "
            "posture node would publish IDENTITY and snap body pose to default"
        )


def test_reseating_preserves_body_pose():
    # The persistent height offset must keep applying while reseat walks
    # the feet to the new nominal — otherwise the body drops mid-ladder.
    assert "reseating" in POSTURE_ACTIVE_STATES


def test_walking_states_are_active():
    for state in ("stand", "engaging", "gait"):
        assert state in POSTURE_ACTIVE_STATES


def test_pre_stand_states_emit_identity():
    # Legs aren't at nominal footprint in these states — composing a
    # body-pose offset would push IK against the wrong configuration.
    for state in ("folded", "initialize", "folding"):
        assert state not in POSTURE_ACTIVE_STATES
