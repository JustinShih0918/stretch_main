"""Tests for the compact latest-only status formatter."""

from types import SimpleNamespace

from vln_policy.vln_status_monitor import format_status


def test_format_status_shows_current_snapshot():
    status = SimpleNamespace(
        state="EXECUTING",
        instruction="go through the doorway",
        current_action="FORWARD+TURN_LEFT",
        pending_actions=["FORWARD"],
        step_count=3,
        backend="dummy",
        execution_mode="nav2",
        detail="goal accepted",
    )

    output = format_status(status)

    assert "EXECUTING" in output
    assert "go through the doorway" in output
    assert "FORWARD+TURN_LEFT" in output
    assert "dummy / nav2" in output
