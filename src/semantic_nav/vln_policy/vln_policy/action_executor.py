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
from dataclasses import dataclass
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


def compose_relative(actions: List[str]) -> OdomPose:
    """Fold a discrete action batch into one relative SE(2) pose
    (robot frame at batch start: x forward, y left)."""
    x, y, yaw = 0.0, 0.0, 0.0
    for action in actions:
        if action == FORWARD:
            x += FORWARD_M * math.cos(yaw)
            y += FORWARD_M * math.sin(yaw)
        elif action == BACKWARD:
            x -= FORWARD_M * math.cos(yaw)
            y -= FORWARD_M * math.sin(yaw)
        elif action == TURN_LEFT:
            yaw = wrap_angle(yaw + TURN_RAD)
        elif action == TURN_RIGHT:
            yaw = wrap_angle(yaw - TURN_RAD)
        else:
            raise ValueError(f"non-motion action in batch: {action}")
    return OdomPose(x=x, y=y, yaw=yaw)


@dataclass(frozen=True)
class RelativeWaypoint:
    x: float
    y: float
    yaw: Optional[float] = None
    explicit_turn: bool = False


@dataclass(frozen=True)
class WorldWaypoint:
    x: float
    y: float
    yaw: Optional[float] = None
    explicit_turn: bool = False


def actions_to_trajectory(actions: List[str]) -> List[RelativeWaypoint]:
    """Convert every discrete action to a lossless relative waypoint.

    Turns remain zero-translation waypoints with an explicit target yaw;
    therefore a forward/turn sequence cannot be smoothed into a different
    command by the continuous follower.
    """
    x, y, yaw = 0.0, 0.0, 0.0
    points = []
    for action in actions:
        if action == FORWARD:
            x += FORWARD_M * math.cos(yaw)
            y += FORWARD_M * math.sin(yaw)
            points.append(RelativeWaypoint(x, y, yaw, False))
        elif action == BACKWARD:
            x -= FORWARD_M * math.cos(yaw)
            y -= FORWARD_M * math.sin(yaw)
            points.append(RelativeWaypoint(x, y, yaw, False))
        elif action == TURN_LEFT:
            yaw = wrap_angle(yaw + TURN_RAD)
            points.append(RelativeWaypoint(x, y, yaw, True))
        elif action == TURN_RIGHT:
            yaw = wrap_angle(yaw - TURN_RAD)
            points.append(RelativeWaypoint(x, y, yaw, True))
        else:
            raise ValueError(f"non-motion action in batch: {action}")
    return points


class TrajectoryFollowerExecutor:
    """Odometry-closed-loop follower shared by action and trajectory models."""

    def __init__(
        self,
        publish_twist: Callable[[float, float], None],
        v_lin: float = 0.25,
        v_ang: float = 0.5,
        lookahead_m: float = 0.35,
        final_tolerance_m: float = 0.12,
        turn_tolerance_rad: float = math.radians(5.0),
        linear_accel_mps2: float = 0.5,
        angular_accel_rps2: float = 1.0,
        watchdog_s: float = 6.0,
        odom_timeout_s: float = 1.0,
        control_rate_hz: float = 20.0,
    ):
        self._publish = publish_twist
        self.v_lin = float(v_lin)
        self.v_ang = float(v_ang)
        self.lookahead_m = float(lookahead_m)
        self.final_tolerance_m = float(final_tolerance_m)
        self.turn_tolerance_rad = float(turn_tolerance_rad)
        self.linear_accel_mps2 = float(linear_accel_mps2)
        self.angular_accel_rps2 = float(angular_accel_rps2)
        self.watchdog_s = float(watchdog_s)
        self.odom_timeout_s = float(odom_timeout_s)
        self.control_period_s = 1.0 / float(control_rate_hz)
        self._waypoints: List[WorldWaypoint] = []
        self._status = ExecStatus.IDLE
        self._error = ""
        self._last_tick = None
        self._last_odom_time = None
        self._last_progress_time = None
        self._progress_pose = None
        self._last_v = 0.0
        self._last_w = 0.0
        self._source = ""
        self.controller_tick_count = 0

    @property
    def status(self) -> ExecStatus:
        return self._status

    @property
    def active_action(self) -> Optional[str]:
        if self._status is not ExecStatus.RUNNING:
            return None
        return "TRAJECTORY" if self._source == "trajectory" else "ACTION_PATH"

    @property
    def pending_actions(self) -> List[str]:
        return ["WAYPOINT"] * len(self._waypoints)

    @property
    def error(self) -> str:
        return self._error

    def submit_actions(
        self, actions: List[str], odom: Optional[OdomPose], now: float = 0.0
    ) -> None:
        self._replace(actions_to_trajectory(actions), odom, now, "actions")

    def replace_trajectory(
        self, points, odom: Optional[OdomPose], now: float = 0.0
    ) -> None:
        relative = [
            RelativeWaypoint(float(p.x), float(p.y), None, False)
            for p in points
        ]
        self._replace(relative, odom, now, "trajectory")

    def submit(self, commands, odom: Optional[OdomPose], now: float = 0.0):
        """Compatibility entry point: action strings or trajectory points."""
        if not commands:
            self._waypoints = []
            self._status = ExecStatus.DONE
        elif isinstance(commands[0], str):
            self.submit_actions(commands, odom, now)
        else:
            self.replace_trajectory(commands, odom, now)

    def _replace(
        self,
        relative: List[RelativeWaypoint],
        odom: Optional[OdomPose],
        now: float,
        source: str,
    ) -> None:
        if odom is None:
            self._fail("cannot follow trajectory: no odometry yet")
            return
        cosine, sine = math.cos(odom.yaw), math.sin(odom.yaw)
        # Build the complete new list before assigning it: callbacks see an
        # atomic replacement even while DualVLN replans during motion.
        replacement = [
            WorldWaypoint(
                x=odom.x + point.x * cosine - point.y * sine,
                y=odom.y + point.x * sine + point.y * cosine,
                yaw=(
                    wrap_angle(odom.yaw + point.yaw)
                    if point.yaw is not None else None
                ),
                explicit_turn=point.explicit_turn,
            )
            for point in relative
        ]
        was_running = self._status is ExecStatus.RUNNING
        self._waypoints = replacement
        self._source = source
        self._error = ""
        self._status = ExecStatus.RUNNING if replacement else ExecStatus.DONE
        self._last_tick = now
        # A fresh model plan is not physical progress. Preserve the watchdog
        # across DualVLN replacements so a stuck robot still stops in 6 s.
        if not was_running or self._progress_pose is None:
            self._last_progress_time = now
            self._progress_pose = OdomPose(odom.x, odom.y, odom.yaw)
        if not replacement:
            self._zero()

    def notify_odom(self, received_time: float) -> None:
        self._last_odom_time = received_time

    def cancel(self) -> None:
        self._waypoints = []
        self._status = ExecStatus.IDLE
        self._error = ""
        self._zero()

    def tick(
        self,
        now: float,
        odom: Optional[OdomPose],
        odom_time: Optional[float] = None,
    ) -> ExecStatus:
        if self._status is not ExecStatus.RUNNING:
            return self._status
        self.controller_tick_count += 1
        if odom_time is not None:
            self._last_odom_time = odom_time
        if odom is None:
            if now - self._last_progress_time > self.watchdog_s:
                self._fail("odometry missing for trajectory follower")
            else:
                self._zero()
            return self._status
        if (
            self._last_odom_time is not None
            and now - self._last_odom_time > self.odom_timeout_s
        ):
            self._fail(
                f"odometry lost for {now - self._last_odom_time:.1f}s"
            )
            return self._status

        self._update_progress(now, odom)
        if now - self._last_progress_time > self.watchdog_s:
            self._fail(
                f"trajectory made no progress for {self.watchdog_s:.1f}s"
            )
            return self._status

        self._discard_reached(odom)
        if not self._waypoints:
            self._status = ExecStatus.DONE
            self._zero()
            return self._status

        target = self._control_target(odom)
        dx, dy = target.x - odom.x, target.y - odom.y
        distance = math.hypot(dx, dy)

        if target.explicit_turn and distance <= self.final_tolerance_m:
            yaw_error = wrap_angle(target.yaw - odom.yaw)
            desired_v = 0.0
            desired_w = max(-self.v_ang, min(self.v_ang, 2.0 * yaw_error))
        else:
            heading_error = wrap_angle(math.atan2(dy, dx) - odom.yaw)
            desired_w = max(
                -self.v_ang, min(self.v_ang, 2.0 * heading_error)
            )
            alignment = max(0.0, math.cos(heading_error))
            desired_v = self.v_lin * alignment
            if abs(heading_error) > math.pi / 2.0:
                desired_v = 0.0
            if target is self._waypoints[-1] and distance < self.lookahead_m:
                desired_v *= max(0.15, distance / self.lookahead_m)

        dt = max(
            1e-3,
            min(0.25, now - self._last_tick)
            if self._last_tick is not None else self.control_period_s,
        )
        self._last_tick = now
        self._last_v = self._rate_limit(
            self._last_v, desired_v, self.linear_accel_mps2 * dt
        )
        self._last_w = self._rate_limit(
            self._last_w, desired_w, self.angular_accel_rps2 * dt
        )
        self._publish(self._last_v, self._last_w)
        return self._status

    def _discard_reached(self, odom: OdomPose) -> None:
        while self._waypoints:
            point = self._waypoints[0]
            distance = math.hypot(point.x - odom.x, point.y - odom.y)
            if point.explicit_turn:
                if distance > self.final_tolerance_m:
                    return
                if abs(wrap_angle(point.yaw - odom.yaw)) > self.turn_tolerance_rad:
                    return
            else:
                tolerance = (
                    self.final_tolerance_m
                    if len(self._waypoints) == 1
                    else min(self.lookahead_m * 0.5, 0.18)
                )
                if distance > tolerance:
                    return
            self._waypoints.pop(0)

    def _control_target(self, odom: OdomPose) -> WorldWaypoint:
        for point in self._waypoints:
            if point.explicit_turn:
                return point
            if math.hypot(point.x - odom.x, point.y - odom.y) >= self.lookahead_m:
                return point
        return self._waypoints[-1]

    def _update_progress(self, now: float, odom: OdomPose) -> None:
        if self._progress_pose is None:
            self._progress_pose = OdomPose(odom.x, odom.y, odom.yaw)
            self._last_progress_time = now
            return
        moved = math.hypot(
            odom.x - self._progress_pose.x, odom.y - self._progress_pose.y
        )
        turned = abs(wrap_angle(odom.yaw - self._progress_pose.yaw))
        if moved >= 0.01 or turned >= math.radians(1.0):
            self._progress_pose = OdomPose(odom.x, odom.y, odom.yaw)
            self._last_progress_time = now

    @staticmethod
    def _rate_limit(current: float, desired: float, delta: float) -> float:
        return max(current - delta, min(current + delta, desired))

    def _zero(self) -> None:
        self._last_v = 0.0
        self._last_w = 0.0
        self._publish(0.0, 0.0)

    def _fail(self, message: str) -> None:
        self._waypoints = []
        self._error = message
        self._status = ExecStatus.ERROR
        self._zero()


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
