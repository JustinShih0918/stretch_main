"""Executor geometry/sequencing tests — pure Python, no ROS."""

import math

import pytest

from vln_policy.action_executor import (
    CmdVelExecutor,
    ExecStatus,
    Nav2WaypointExecutor,
    compose_relative,
    wrap_angle,
)
from vln_policy.backends.base import BACKWARD, FORWARD_M, TURN_RAD, OdomPose


class TwistLog:
    def __init__(self):
        self.calls = []

    def __call__(self, linear_x, angular_z):
        self.calls.append((linear_x, angular_z))

    @property
    def last(self):
        return self.calls[-1]


def make_executor(**kwargs):
    log = TwistLog()
    ex = CmdVelExecutor(publish_twist=log, **kwargs)
    return ex, log


class TestCmdVelExecutor:
    def test_forward_completes_on_odom_displacement(self):
        ex, log = make_executor()
        ex.submit(["FORWARD"])
        assert ex.tick(0.0, OdomPose(0, 0, 0)) is ExecStatus.RUNNING
        assert log.last == (ex.v_lin, 0.0)
        # not far enough yet
        assert ex.tick(0.4, OdomPose(FORWARD_M * 0.6, 0, 0)) \
            is ExecStatus.RUNNING
        # displacement reached -> done + zero twist
        assert ex.tick(1.0, OdomPose(FORWARD_M, 0, 0)) is ExecStatus.DONE
        assert log.last == (0.0, 0.0)

    def test_backward_uses_negative_velocity_and_odom_distance(self):
        ex, log = make_executor()
        ex.submit([BACKWARD])
        assert ex.tick(0.0, OdomPose(0, 0, 0)) is ExecStatus.RUNNING
        assert log.last == (-ex.v_lin, 0.0)
        assert ex.tick(1.0, OdomPose(-FORWARD_M, 0, 0)) is ExecStatus.DONE
        assert log.last == (0.0, 0.0)

    def test_turn_left_and_right_termination(self):
        for action, sign in (("TURN_LEFT", 1.0), ("TURN_RIGHT", -1.0)):
            ex, log = make_executor()
            ex.submit([action])
            ex.tick(0.0, OdomPose(0, 0, 0))
            assert log.last == (0.0, sign * ex.v_ang)
            status = ex.tick(0.5, OdomPose(0, 0, sign * TURN_RAD * 1.05))
            assert status is ExecStatus.DONE
            assert log.last == (0.0, 0.0)

    def test_turn_yaw_wraparound(self):
        # start near +pi: TURN_LEFT crosses the branch cut
        start_yaw = math.pi - TURN_RAD / 2.0
        ex, _ = make_executor()
        ex.submit(["TURN_LEFT"])
        ex.tick(0.0, OdomPose(0, 0, start_yaw))
        end_yaw = wrap_angle(start_yaw + TURN_RAD * 1.05)
        assert end_yaw < 0  # wrapped
        assert ex.tick(0.5, OdomPose(0, 0, end_yaw)) is ExecStatus.DONE

    def test_sequence_runs_in_order(self):
        ex, log = make_executor()
        ex.submit(["FORWARD", "TURN_LEFT"])
        ex.tick(0.0, OdomPose(0, 0, 0))
        assert ex.active_action == "FORWARD"
        assert ex.pending_actions == ["TURN_LEFT"]
        ex.tick(0.5, OdomPose(FORWARD_M, 0, 0))  # forward done
        assert ex.status is ExecStatus.RUNNING
        ex.tick(0.6, OdomPose(FORWARD_M, 0, 0))
        assert ex.active_action == "TURN_LEFT"
        status = ex.tick(1.0, OdomPose(FORWARD_M, 0, TURN_RAD))
        assert status is ExecStatus.DONE

    def test_timeout_aborts_with_zero_twist(self):
        ex, log = make_executor(action_timeout_s=2.0)
        ex.submit(["FORWARD"])
        ex.tick(0.0, OdomPose(0, 0, 0))
        assert ex.tick(2.5, OdomPose(0.01, 0, 0)) is ExecStatus.ERROR
        assert "timed out" in ex.error
        assert log.last == (0.0, 0.0)

    def test_odom_silence_times_out(self):
        ex, log = make_executor(action_timeout_s=2.0)
        ex.submit(["FORWARD"])
        assert ex.tick(0.0, None) is ExecStatus.RUNNING
        assert log.last == (0.0, 0.0)  # never move blind
        assert ex.tick(2.5, None) is ExecStatus.ERROR
        assert "no odometry" in ex.error

    def test_cancel_zeroes_twist(self):
        ex, log = make_executor()
        ex.submit(["FORWARD"])
        ex.tick(0.0, OdomPose(0, 0, 0))
        ex.cancel()
        assert ex.status is ExecStatus.IDLE
        assert log.last == (0.0, 0.0)

    def test_rejects_stop_token(self):
        ex, _ = make_executor()
        with pytest.raises(ValueError):
            ex.submit(["STOP"])


class TestComposeRelative:
    def test_forward_turn_forward(self):
        rel = compose_relative(["FORWARD", "TURN_LEFT", "FORWARD"])
        assert rel.x == pytest.approx(FORWARD_M * (1 + math.cos(TURN_RAD)))
        assert rel.y == pytest.approx(FORWARD_M * math.sin(TURN_RAD))
        assert rel.yaw == pytest.approx(TURN_RAD)

    def test_turns_cancel(self):
        rel = compose_relative(["TURN_LEFT", "TURN_RIGHT"])
        assert rel.yaw == pytest.approx(0.0)
        assert rel.x == rel.y == 0.0

    def test_backward_is_negative_robot_x(self):
        rel = compose_relative([BACKWARD, BACKWARD])
        assert rel.x == pytest.approx(-2 * FORWARD_M)
        assert rel.y == pytest.approx(0.0)
        assert rel.yaw == pytest.approx(0.0)


class TestNav2WaypointExecutor:
    def test_goal_composed_in_odom_frame(self):
        goals = []
        ex = Nav2WaypointExecutor(
            send_goal=lambda x, y, yaw: goals.append((x, y, yaw))
        )
        odom = OdomPose(1.0, 2.0, math.pi / 2)  # facing +y
        ex.submit(["FORWARD"], odom, now=0.0)
        assert ex.status is ExecStatus.RUNNING
        x, y, yaw = goals[0]
        assert x == pytest.approx(1.0)
        assert y == pytest.approx(2.0 + FORWARD_M)
        assert yaw == pytest.approx(math.pi / 2)

    def test_result_flow(self):
        ex = Nav2WaypointExecutor(send_goal=lambda *a: None)
        ex.submit(["FORWARD"], OdomPose(), now=0.0)
        ex.notify_result(True)
        assert ex.status is ExecStatus.DONE

        ex.submit(["FORWARD"], OdomPose(), now=0.0)
        ex.notify_result(False, "aborted")
        assert ex.status is ExecStatus.ERROR
        assert "aborted" in ex.error

    def test_no_odom_is_error(self):
        ex = Nav2WaypointExecutor(send_goal=lambda *a: None)
        ex.submit(["FORWARD"], None)
        assert ex.status is ExecStatus.ERROR

    def test_goal_timeout(self):
        ex = Nav2WaypointExecutor(
            send_goal=lambda *a: None, goal_timeout_s=10.0
        )
        ex.submit(["FORWARD"], OdomPose(), now=100.0)
        assert ex.tick(105.0, None) is ExecStatus.RUNNING
        assert ex.tick(111.0, None) is ExecStatus.ERROR
