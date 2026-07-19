"""Executors: turn discrete VLN actions into robot motion.

Pure Python (no rclpy) — ROS I/O is injected as callbacks so the geometry
and sequencing logic is unit-testable:

  * CmdVelExecutor      — timed velocity bursts closed-loop on odometry
                          (mirrors StreamVLN's own real-robot execution).
  * Nav2WaypointExecutor — folds an action batch into one relative SE(2)
                           waypoint and hands it to nav2 navigate_to_pose
                           (costmaps get veto power).
"""

import math
from enum import Enum
from typing import Callable, List, Optional

from .backends.base import (
    FORWARD,
    FORWARD_M,
    MOTION_ACTIONS,
    TURN_LEFT,
    TURN_RAD,
    TURN_RIGHT,
    OdomPose,
)


class ExecStatus(Enum):
    IDLE = "IDLE"          # nothing submitted
    RUNNING = "RUNNING"    # actions in flight
    DONE = "DONE"          # batch finished
    ERROR = "ERROR"        # timeout / nav2 abort; caller must resubmit


def wrap_angle(angle: float) -> float:
    """Wrap to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_relative(actions: List[str]) -> OdomPose:
    """Fold a discrete action batch into one relative SE(2) pose
    (robot frame at batch start: x forward, y left)."""
    x, y, yaw = 0.0, 0.0, 0.0
    for action in actions:
        if action == FORWARD:
            x += FORWARD_M * math.cos(yaw)
            y += FORWARD_M * math.sin(yaw)
        elif action == TURN_LEFT:
            yaw = wrap_angle(yaw + TURN_RAD)
        elif action == TURN_RIGHT:
            yaw = wrap_angle(yaw - TURN_RAD)
        else:
            raise ValueError(f"non-motion action in batch: {action}")
    return OdomPose(x=x, y=y, yaw=yaw)


class CmdVelExecutor:
    """Executes one discrete action at a time as a velocity burst, using
    odometry displacement (not time) as the completion criterion.

    publish_twist(linear_x, angular_z) is called on every tick while an
    action runs, and with (0, 0) whenever motion must stop.
    """

    def __init__(
        self,
        publish_twist: Callable[[float, float], None],
        v_lin: float = 0.25,
        v_ang: float = 0.5,
        action_timeout_s: float = 6.0,
    ):
        self._publish = publish_twist
        self.v_lin = float(v_lin)
        self.v_ang = float(v_ang)
        self.action_timeout_s = float(action_timeout_s)
        self._queue: List[str] = []
        self._active: Optional[str] = None
        self._start_pose: Optional[OdomPose] = None
        self._start_time: Optional[float] = None
        self._status = ExecStatus.IDLE
        self._error = ""

    @property
    def status(self) -> ExecStatus:
        return self._status

    @property
    def active_action(self) -> Optional[str]:
        return self._active

    @property
    def pending_actions(self) -> List[str]:
        return list(self._queue)

    @property
    def error(self) -> str:
        return self._error

    def submit(self, actions: List[str]) -> None:
        for action in actions:
            if action not in MOTION_ACTIONS:
                raise ValueError(f"executor got non-motion action {action}")
        self._queue = list(actions)
        self._active = None
        self._start_pose = None
        self._start_time = None
        self._error = ""
        self._status = ExecStatus.RUNNING if self._queue else ExecStatus.DONE

    def cancel(self) -> None:
        self._queue = []
        self._active = None
        self._status = ExecStatus.IDLE
        self._publish(0.0, 0.0)

    def tick(self, now: float, odom: Optional[OdomPose]) -> ExecStatus:
        if self._status is not ExecStatus.RUNNING:
            return self._status

        if self._active is None:
            if not self._queue:
                self._status = ExecStatus.DONE
                self._publish(0.0, 0.0)
                return self._status
            self._active = self._queue.pop(0)
            self._start_pose = None  # latched from odom below
            self._start_time = now

        if now - self._start_time > self.action_timeout_s:
            self._error = (
                f"action {self._active} timed out after "
                f"{self.action_timeout_s:.1f}s"
                + ("" if odom else " (no odometry received)")
            )
            self._active = None
            self._queue = []
            self._status = ExecStatus.ERROR
            self._publish(0.0, 0.0)
            return self._status

        if odom is None:
            self._publish(0.0, 0.0)  # wait for odometry, timeout still runs
            return self._status

        if self._start_pose is None:
            self._start_pose = OdomPose(odom.x, odom.y, odom.yaw)

        if self._finished(odom):
            self._active = None
            self._start_pose = None
            if not self._queue:
                self._status = ExecStatus.DONE
            self._publish(0.0, 0.0)
            return self._status

        if self._active == FORWARD:
            self._publish(self.v_lin, 0.0)
        elif self._active == TURN_LEFT:
            self._publish(0.0, self.v_ang)
        elif self._active == TURN_RIGHT:
            self._publish(0.0, -self.v_ang)
        return self._status

    def _finished(self, odom: OdomPose) -> bool:
        start = self._start_pose
        if self._active == FORWARD:
            dist = math.hypot(odom.x - start.x, odom.y - start.y)
            return dist >= FORWARD_M
        delta = wrap_angle(odom.yaw - start.yaw)
        if self._active == TURN_LEFT:
            return delta >= TURN_RAD
        return delta <= -TURN_RAD


class Nav2WaypointExecutor:
    """Folds a whole action batch into one goal pose in the odom frame and
    sends it through the injected navigate_to_pose callback.

    send_goal(x, y, yaw) fires the nav2 action goal; the owner reports the
    outcome back via notify_result(success).
    """

    def __init__(
        self,
        send_goal: Callable[[float, float, float], None],
        goal_timeout_s: float = 60.0,
    ):
        self._send_goal = send_goal
        self.goal_timeout_s = float(goal_timeout_s)
        self._status = ExecStatus.IDLE
        self._active_batch: List[str] = []
        self._start_time: Optional[float] = None
        self._error = ""

    @property
    def status(self) -> ExecStatus:
        return self._status

    @property
    def active_action(self) -> Optional[str]:
        return "+".join(self._active_batch) if self._active_batch else None

    @property
    def pending_actions(self) -> List[str]:
        return []  # the whole batch goes out as one goal

    @property
    def error(self) -> str:
        return self._error

    def goal_from(self, odom: OdomPose, actions: List[str]) -> OdomPose:
        """Compose the batch on top of the current odom pose."""
        rel = compose_relative(actions)
        cos_yaw, sin_yaw = math.cos(odom.yaw), math.sin(odom.yaw)
        return OdomPose(
            x=odom.x + rel.x * cos_yaw - rel.y * sin_yaw,
            y=odom.y + rel.x * sin_yaw + rel.y * cos_yaw,
            yaw=wrap_angle(odom.yaw + rel.yaw),
        )

    def submit(self, actions: List[str], odom: Optional[OdomPose],
               now: float = 0.0) -> None:
        if odom is None:
            self._error = "cannot compose waypoint: no odometry yet"
            self._status = ExecStatus.ERROR
            return
        if not actions:
            self._status = ExecStatus.DONE
            return
        goal = self.goal_from(odom, actions)
        self._active_batch = list(actions)
        self._start_time = now
        self._error = ""
        self._status = ExecStatus.RUNNING
        self._send_goal(goal.x, goal.y, goal.yaw)

    def notify_result(self, success: bool, detail: str = "") -> None:
        if self._status is not ExecStatus.RUNNING:
            return
        self._active_batch = []
        if success:
            self._status = ExecStatus.DONE
        else:
            self._error = detail or "navigate_to_pose goal failed"
            self._status = ExecStatus.ERROR

    def cancel(self) -> None:
        self._active_batch = []
        self._status = ExecStatus.IDLE

    def tick(self, now: float, odom: Optional[OdomPose]) -> ExecStatus:
        if (
            self._status is ExecStatus.RUNNING
            and self._start_time is not None
            and now - self._start_time > self.goal_timeout_s
        ):
            self._error = (
                f"navigate_to_pose goal timed out after "
                f"{self.goal_timeout_s:.1f}s"
            )
            self._active_batch = []
            self._status = ExecStatus.ERROR
        return self._status
