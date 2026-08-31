import math

import pytest

from vln_policy.action_executor import (
    ExecStatus,
    TrajectoryFollowerExecutor,
    actions_to_trajectory,
)
from vln_policy.backends.base import OdomPose, TrajectoryPoint


class TwistLog:
    def __init__(self):
        self.values = []

    def __call__(self, linear, angular):
        self.values.append((linear, angular))

    @property
    def last(self):
        return self.values[-1]


def follower(**kwargs):
    log = TwistLog()
    executor = TrajectoryFollowerExecutor(log, **kwargs)
    return executor, log


def test_action_to_trajectory_preserves_every_turn_and_translation():
    points = actions_to_trajectory([
        "FORWARD", "TURN_LEFT", "FORWARD", "TURN_RIGHT"
    ])
    assert len(points) == 4
    assert (points[0].x, points[0].y, points[0].explicit_turn) == (
        pytest.approx(0.25), pytest.approx(0.0), False
    )
    assert points[1].explicit_turn
    assert points[1].yaw == pytest.approx(math.radians(15))
    assert points[2].x == pytest.approx(0.25 + 0.25 * math.cos(math.radians(15)))
    assert points[2].y == pytest.approx(0.25 * math.sin(math.radians(15)))
    assert points[3].explicit_turn
    assert points[3].yaw == pytest.approx(0.0)


def test_pure_rotation_completes_at_five_degree_tolerance():
    executor, log = follower()
    executor.submit_actions(["TURN_LEFT"], OdomPose(), now=0.0)
    assert executor.tick(0.05, OdomPose(), 0.05) is ExecStatus.RUNNING
    assert log.last[0] == 0.0
    assert log.last[1] > 0.0
    end = OdomPose(yaw=math.radians(11))
    assert executor.tick(0.5, end, 0.5) is ExecStatus.DONE
    assert log.last == (0.0, 0.0)


def test_curved_continuous_path_generates_linear_and_angular_control():
    executor, log = follower(linear_accel_mps2=100, angular_accel_rps2=100)
    executor.replace_trajectory(
        [TrajectoryPoint(0.4, 0.1), TrajectoryPoint(0.8, 0.4)],
        OdomPose(), now=0.0,
    )
    executor.tick(0.05, OdomPose(), 0.05)
    linear, angular = log.last
    assert 0.0 < linear <= 0.25
    assert 0.0 < angular <= 0.5


def test_atomic_replacement_uses_latest_robot_frame():
    executor, log = follower(linear_accel_mps2=100, angular_accel_rps2=100)
    executor.replace_trajectory([TrajectoryPoint(1, 0)], OdomPose(), 0.0)
    executor.replace_trajectory(
        [TrajectoryPoint(1, 0)], OdomPose(2, 3, math.pi / 2), 0.1
    )
    # The replacement target is now world (2,4), already straight ahead.
    executor.tick(0.15, OdomPose(2, 3, math.pi / 2), 0.15)
    assert log.last[0] > 0.0
    assert abs(log.last[1]) < 1e-6
    assert len(executor.pending_actions) == 1


def test_cancel_immediately_zeroes_motion():
    executor, log = follower()
    executor.replace_trajectory([TrajectoryPoint(1, 0)], OdomPose(), 0.0)
    executor.tick(0.05, OdomPose(), 0.05)
    executor.cancel()
    assert executor.status is ExecStatus.IDLE
    assert log.last == (0.0, 0.0)


def test_odometry_loss_stops_and_errors():
    executor, log = follower(odom_timeout_s=1.0)
    executor.replace_trajectory([TrajectoryPoint(1, 0)], OdomPose(), 0.0)
    executor.notify_odom(0.0)
    assert executor.tick(1.1, OdomPose(), 0.0) is ExecStatus.ERROR
    assert "odometry lost" in executor.error
    assert log.last == (0.0, 0.0)


def test_no_progress_watchdog_stops_and_errors():
    executor, log = follower(watchdog_s=6.0, odom_timeout_s=10.0)
    executor.replace_trajectory([TrajectoryPoint(1, 0)], OdomPose(), 0.0)
    executor.tick(0.1, OdomPose(), 0.1)
    assert executor.tick(6.1, OdomPose(), 6.1) is ExecStatus.ERROR
    assert "no progress" in executor.error
    assert log.last == (0.0, 0.0)


def test_replanning_does_not_reset_no_progress_watchdog():
    executor, _ = follower(watchdog_s=6.0, odom_timeout_s=10.0)
    for now in (0.0, 2.0, 4.0):
        executor.replace_trajectory(
            [TrajectoryPoint(1, 0)], OdomPose(), now
        )
        executor.tick(now + 0.1, OdomPose(), now + 0.1)
    executor.replace_trajectory([TrajectoryPoint(1, 0)], OdomPose(), 6.0)
    assert executor.tick(6.1, OdomPose(), 6.1) is ExecStatus.ERROR


def test_acceleration_is_limited():
    executor, log = follower(linear_accel_mps2=0.5, angular_accel_rps2=1.0)
    executor.replace_trajectory([TrajectoryPoint(1, 0)], OdomPose(), 0.0)
    executor.tick(0.05, OdomPose(), 0.05)
    assert log.last[0] == pytest.approx(0.025)
    assert log.last[1] == pytest.approx(0.0)
