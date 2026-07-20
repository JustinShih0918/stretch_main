"""Tests for the pure-geometry helpers of the RViz visualization node."""

import math

import pytest

from vln_policy.backends.base import BACKWARD, FORWARD_M, TURN_RAD, OdomPose
from vln_policy.viz_geometry import (
    action_trajectory,
    records_episode_path,
    split_actions,
)


class FakeStatus:
    def __init__(self, current_action="", pending_actions=()):
        self.current_action = current_action
        self.pending_actions = list(pending_actions)


class TestActionTrajectory:
    def test_forward_only(self):
        points, yaw = action_trajectory(OdomPose(1.0, 2.0, 0.0),
                                        ["FORWARD", "FORWARD"])
        assert points == [(1.0, 2.0),
                          (1.0 + FORWARD_M, 2.0),
                          (1.0 + 2 * FORWARD_M, 2.0)]
        assert yaw == 0.0

    def test_turn_changes_heading_not_position(self):
        points, yaw = action_trajectory(OdomPose(0, 0, 0),
                                        ["TURN_LEFT", "FORWARD"])
        assert len(points) == 2
        assert points[1][0] == pytest.approx(FORWARD_M * math.cos(TURN_RAD))
        assert points[1][1] == pytest.approx(FORWARD_M * math.sin(TURN_RAD))
        assert yaw == pytest.approx(TURN_RAD)

    def test_turns_only_no_extra_points(self):
        points, yaw = action_trajectory(OdomPose(0, 0, 0.5),
                                        ["TURN_RIGHT", "TURN_RIGHT"])
        assert points == [(0, 0)]
        assert yaw == pytest.approx(0.5 - 2 * TURN_RAD)

    def test_backward_draws_behind_current_heading(self):
        points, yaw = action_trajectory(OdomPose(1.0, 2.0, 0.0),
                                        [BACKWARD])
        assert points == [(1.0, 2.0), (1.0 - FORWARD_M, 2.0)]
        assert yaw == 0.0


class TestSplitActions:
    def test_cmd_vel_mode(self):
        s = FakeStatus("FORWARD", ["TURN_LEFT", "FORWARD"])
        assert split_actions(s) == ["FORWARD", "TURN_LEFT", "FORWARD"]

    def test_nav2_mode_joined_batch(self):
        s = FakeStatus("FORWARD+TURN_LEFT+FORWARD", [])
        assert split_actions(s) == ["FORWARD", "TURN_LEFT", "FORWARD"]

    def test_ignores_non_motion_tokens(self):
        s = FakeStatus("IDLE", ["STOP"])
        assert split_actions(s) == []

    def test_keeps_backward(self):
        s = FakeStatus(BACKWARD, [BACKWARD])
        assert split_actions(s) == [BACKWARD, BACKWARD]


class TestEpisodePathState:
    def test_records_only_while_episode_is_active(self):
        assert records_episode_path("RESETTING")
        assert records_episode_path("THINKING")
        assert records_episode_path("EXECUTING")

    def test_stops_outside_episode(self):
        assert not records_episode_path("IDLE")
        assert not records_episode_path("DONE")
        assert not records_episode_path("ERROR")
