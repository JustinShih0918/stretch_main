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
    BACKWARD,
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


def compose_relative(
    actions: List[str],
    forward_m: float = FORWARD_M,
    turn_rad: float = TURN_RAD,
) -> OdomPose:
    """Fold a discrete action batch into one relative SE(2) pose
    (robot frame at batch start: x forward, y left).

    forward_m/turn_rad default to the VLN-CE reference geometry; raising them
    stretches the same action batch into a longer, smoother nav2 waypoint.
    """
    x, y, yaw = 0.0, 0.0, 0.0
    for action in actions:
        if action == FORWARD:
            x += forward_m * math.cos(yaw)
            y += forward_m * math.sin(yaw)
        elif action == BACKWARD:
            x -= forward_m * math.cos(yaw)
            y -= forward_m * math.sin(yaw)
        elif action == TURN_LEFT:
            yaw = wrap_angle(yaw + turn_rad)
        elif action == TURN_RIGHT:
            yaw = wrap_angle(yaw - turn_rad)
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
        v_lin: float = 0.35,
        v_ang: float = 0.15,
        action_timeout_s: float = 6.0,
        forward_m: float = FORWARD_M,
        turn_rad: float = TURN_RAD,
    ):
        self._publish = publish_twist
        self.v_lin = float(v_lin)
        self.v_ang = float(v_ang)
        self.action_timeout_s = float(action_timeout_s)
        # How far one action token travels. Longer steps at the same speed take
        # longer, so action_timeout_s has to keep up (see the params files).
        self.forward_m = float(forward_m)
        self.turn_rad = float(turn_rad)
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
        elif self._active == BACKWARD:
            self._publish(-self.v_lin, 0.0)
        elif self._active == TURN_LEFT:
            self._publish(0.0, self.v_ang)
        elif self._active == TURN_RIGHT:
            self._publish(0.0, -self.v_ang)
        return self._status

    def _finished(self, odom: OdomPose) -> bool:
        start = self._start_pose
        if self._active in (FORWARD, BACKWARD):
            dist = math.hypot(odom.x - start.x, odom.y - start.y)
            return dist >= self.forward_m
        delta = wrap_angle(odom.yaw - start.yaw)
        if self._active == TURN_LEFT:
            return delta >= self.turn_rad
        return delta <= -self.turn_rad


class Nav2WaypointExecutor:
    """Folds a whole action batch into one goal pose in the odom frame and
    sends it through the injected navigate_to_pose callback.

    send_goal(x, y, yaw) fires the goal; how completion is detected depends on
    which nav2 interface the owner uses:

      * action goal  -> the owner calls notify_result(success) with the action
                        result. Exact: nav2 reports success, abort or cancel.
      * /goal_pose topic -> nav2 sends no result back (a topic goal is
                        fire-and-forget), so pass arrival tolerances and this
                        executor completes the batch on odometry instead: the
                        robot must have moved, reached the tolerance ball, and
                        stood still for arrival_settle_s. A nav2 abort is then
                        indistinguishable from slow progress and surfaces as
                        the goal timeout.
    """

    def __init__(
        self,
        send_goal: Callable[[float, float, float], None],
        goal_timeout_s: float = 60.0,
        arrival_xy_tol: float = 0.0,
        arrival_yaw_tol: float = 0.0,
        arrival_settle_s: float = 0.0,
        forward_m: float = FORWARD_M,
        turn_rad: float = TURN_RAD,
    ):
        self._send_goal = send_goal
        self.goal_timeout_s = float(goal_timeout_s)
        # Per-token geometry used to fold a batch into one waypoint. Bigger
        # values put the goal further out, so nav2 drives one long smooth leg
        # instead of stopping and restarting every 0.25 m.
        self.forward_m = float(forward_m)
        self.turn_rad = float(turn_rad)
        # <= 0 disables odometry arrival detection (action mode).
        self.arrival_xy_tol = float(arrival_xy_tol)
        self.arrival_yaw_tol = float(arrival_yaw_tol)
        # The tolerance ball is entered well before nav2 stops driving, and
        # calling that "arrived" makes the next waypoint compose from a pose
        # short of the goal — an under-shoot that compounds over a batch. So
        # also wait for the robot to stand still for this long.
        self.arrival_settle_s = float(arrival_settle_s)
        self._last_pose: Optional[OdomPose] = None
        self._last_move_t: Optional[float] = None
        # A batch can be *inside* the tolerance ball before it starts: a
        # single-step batch may be shorter than arrival_xy_tol (with the
        # default 0.25 m step and 0.3 m tolerance it always is), and a
        # turn-only batch never leaves the ball at all. Without this flag such
        # a batch would report arrival immediately and the robot never moves.
        self._departed = False
        self._status = ExecStatus.IDLE
        self._active_batch: List[str] = []
        self._goal: Optional[OdomPose] = None
        self._start_time: Optional[float] = None
        self._error = ""

    @property
    def detects_arrival(self) -> bool:
        """True when completion comes from odometry, not from a nav2 result."""
        return self.arrival_xy_tol > 0.0

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
        rel = compose_relative(actions, self.forward_m, self.turn_rad)
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
        self._goal = goal
        self._last_pose = None
        self._last_move_t = now
        self._departed = False
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
        self._goal = None
        self._status = ExecStatus.IDLE

    def _note_motion(self, now: float, odom: Optional[OdomPose]) -> None:
        """Remember when the robot last moved, for the settle check."""
        if odom is None:
            return
        if self._last_pose is not None:
            moved = math.hypot(odom.x - self._last_pose.x,
                               odom.y - self._last_pose.y)
            turned = abs(wrap_angle(odom.yaw - self._last_pose.yaw))
            if moved > 0.01 or turned > 0.01:
                self._last_move_t = now
                self._departed = True
        self._last_pose = OdomPose(odom.x, odom.y, odom.yaw)

    def _settled(self, now: float) -> bool:
        if self.arrival_settle_s <= 0.0:
            return True
        if self._last_move_t is None:
            return False
        return now - self._last_move_t >= self.arrival_settle_s

    def arrived(self, odom: Optional[OdomPose]) -> bool:
        """Within tolerance of the active goal (odometry-based completion)."""
        if not self.detects_arrival or odom is None or self._goal is None:
            return False
        if math.hypot(odom.x - self._goal.x, odom.y - self._goal.y) > \
                self.arrival_xy_tol:
            return False
        if self.arrival_yaw_tol <= 0.0:
            return True
        return abs(wrap_angle(odom.yaw - self._goal.yaw)) <= \
            self.arrival_yaw_tol

    def tick(self, now: float, odom: Optional[OdomPose]) -> ExecStatus:
        if self._status is ExecStatus.RUNNING and self.detects_arrival:
            self._note_motion(now, odom)
        if (self._status is ExecStatus.RUNNING and self._departed
                and self.arrived(odom) and self._settled(now)):
            self._active_batch = []
            self._goal = None
            self._status = ExecStatus.DONE
            return self._status
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
            self._goal = None
            self._status = ExecStatus.ERROR
        return self._status
