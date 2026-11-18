"""The gateway's ROS 2 side: one rclpy node, spun on its own thread.

Everything the panel does to the robot goes through here, straight over DDS to
the Stretch -- the driver's services, cmd_vel, Nav2's navigate_to_pose,
/initialpose -- except starting and stopping processes, which only the
robot's supervisor can do (control_panel/supervisor) and which this node asks
for over /panel/supervisor/request.

The asyncio side reads `snapshot()` and calls the `*_async` methods, which
return concurrent futures it can await with `asyncio.wrap_future`.
"""
from __future__ import annotations

import concurrent.futures
import itertools
import json
import math
import threading
import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, CompressedImage, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
import tf2_ros

from .core import TeleopGuard, encode_png, occupancy_to_rgba, quat_to_yaw, yaw_to_quat

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)

# The Stretch driver's Trigger services (stretch_core stretch_driver.py).
DRIVER_TRIGGERS = {
    "home": "/home_the_robot",
    "stow": "/stow_the_robot",
    "stop": "/stop_the_robot",
    "mode_navigation": "/switch_to_navigation_mode",
    "mode_position": "/switch_to_position_mode",
    "mode_gamepad": "/switch_to_gamepad_mode",
    "mode_trajectory": "/switch_to_trajectory_mode",
}

# Which node names say a unit is up, whoever started it (the supervisor, or
# someone's terminal with hello-robot's own launch files).
DETECT = {
    "driver": ["stretch_driver"],
    "lidar": ["sllidar_node", "rplidar_composition", "rplidar_node"],
    "camera": ["camera", "realsense2_camera_node"],
    "joystick": ["teleop_twist_joy_node", "teleop_node"],
    "slam": ["slam_toolbox"],
    "navigation": ["bt_navigator"],
    "osg_bridge": ["osg_bridge"],
}

NAV_STATUS = {GoalStatus.STATUS_ACCEPTED: "accepted", GoalStatus.STATUS_EXECUTING: "driving",
              GoalStatus.STATUS_CANCELING: "canceling", GoalStatus.STATUS_SUCCEEDED: "arrived",
              GoalStatus.STATUS_CANCELED: "canceled", GoalStatus.STATUS_ABORTED: "failed"}


class RosIO(Node):
    def __init__(self, cfg: Dict):
        super().__init__("panel_gateway")
        self.cfg = cfg
        self.base_frame = cfg.get("base_frame", "base_link")
        self.map_frame = cfg.get("map_frame", "map")
        self.odom_frame = cfg.get("odom_frame", "odom")
        self._lock = threading.Lock()
        group = ReentrantCallbackGroup()

        # ---- what we read
        self.map_png: Optional[bytes] = None
        self.map_info: Optional[dict] = None
        self.map_version = 0
        self.robot = {"mode": None, "homed": None, "runstopped": None, "battery_v": None, "vel": None}
        self.stamps: Dict[str, float] = {}
        self.scan: Optional[LaserScan] = None
        self.plan: List[List[float]] = []
        self.supervisor: Optional[dict] = None
        self.detected: Dict[str, bool] = {}
        self.camera_jpeg: Optional[bytes] = None
        self._camera_wanted = 0.0
        self._camera_sub = None

        self.create_subscription(OccupancyGrid, cfg.get("map_topic", "/map"), self._on_map, LATCHED,
                                 callback_group=group)
        self.create_subscription(String, "/mode", lambda m: self._set("mode", m.data), 10)
        self.create_subscription(Bool, "/is_homed", lambda m: self._set("homed", m.data), 10)
        self.create_subscription(Bool, "/is_runstopped", lambda m: self._set("runstopped", m.data), 10)
        self.create_subscription(BatteryState, "/battery", lambda m: self._set("battery_v", round(m.voltage, 2)), 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, cfg.get("scan_topic", "/scan_filtered"), self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(String, "/panel/supervisor/state", self._on_supervisor, LATCHED)
        self.create_subscription(String, "/panel/supervisor/reply", self._on_reply, 20)

        # ---- what we send
        self.cmd_vel = self.create_publisher(Twist, cfg.get("cmd_vel_topic", "/stretch/cmd_vel"), 10)
        self.initialpose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.sup_request = self.create_publisher(String, "/panel/supervisor/request", 20)
        self.triggers = {k: self.create_client(Trigger, v, callback_group=group) for k, v in DRIVER_TRIGGERS.items()}
        self.runstop_client = self.create_client(SetBool, "/runstop", callback_group=group)
        self.nav = ActionClient(self, NavigateToPose, cfg.get("nav_action", "navigate_to_pose"),
                                callback_group=group)
        self.nav_state = {"active": False, "status": None, "distance": None, "goal": None, "message": ""}
        self._nav_handle = None
        self._pending: Dict[str, concurrent.futures.Future] = {}
        self._ids = itertools.count(1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.teleop = TeleopGuard(max_linear=float(cfg.get("max_linear", 0.3)),
                                  max_angular=float(cfg.get("max_angular", 1.0)))
        self.create_timer(0.05, self._teleop_tick)     # 20 Hz
        self.create_timer(2.0, self._graph_tick)
        self.create_timer(1.0, self._camera_tick)

    # ------------------------------------------------------------ inputs
    def _set(self, key, value):
        with self._lock:
            self.robot[key] = value
            self.stamps[key] = time.time()

    def _on_odom(self, msg: Odometry):
        self._set("vel", [round(msg.twist.twist.linear.x, 3), round(msg.twist.twist.angular.z, 3)])

    def _on_map(self, msg: OccupancyGrid):
        info = msg.info
        png = encode_png(occupancy_to_rgba(msg.data, info.width, info.height))
        o = info.origin
        with self._lock:
            self.map_png = png
            self.map_version += 1
            self.map_info = {"width": info.width, "height": info.height, "resolution": info.resolution,
                             "origin": [o.position.x, o.position.y,
                                        quat_to_yaw(o.orientation.x, o.orientation.y, o.orientation.z,
                                                    o.orientation.w)],
                             "frame": msg.header.frame_id, "version": self.map_version}
            self.stamps["map"] = time.time()

    def _on_scan(self, msg: LaserScan):
        with self._lock:
            self.scan = msg
            self.stamps["scan"] = time.time()

    def _on_plan(self, msg: Path):
        pts = [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)] for p in msg.poses]
        step = max(1, len(pts) // 200)
        with self._lock:
            self.plan = pts[::step] + (pts[-1:] if pts and (len(pts) - 1) % step else [])

    def _on_supervisor(self, msg: String):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self.supervisor = data
            self.stamps["supervisor"] = time.time()

    def _on_reply(self, msg: String):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        fut = self._pending.pop(str(data.get("id")), None)
        if fut is not None and not fut.done():
            fut.set_result((bool(data.get("ok")), str(data.get("message", ""))))

    def _graph_tick(self):
        names = {n for n, _ in self.get_node_names_and_namespaces()}
        with self._lock:
            self.detected = {unit: any(n in names for n in candidates) for unit, candidates in DETECT.items()}

    # ------------------------------------------------------------ camera
    def want_camera(self):
        """The page is looking at the camera: subscribe (lazily -- 1280x720
        JPEG at 15 Hz is not free on the link) and keep it for 10 s."""
        self._camera_wanted = time.time()
        if self._camera_sub is None:
            self._camera_sub = self.create_subscription(
                CompressedImage, self.cfg.get("camera_topic", "/camera/color/image_raw/compressed"),
                self._on_camera, qos_profile_sensor_data)

    def _on_camera(self, msg: CompressedImage):
        if "jpeg" in msg.format or "jpg" in msg.format:
            self.camera_jpeg = bytes(msg.data)
            self.stamps["camera"] = time.time()

    def _camera_tick(self):
        if self._camera_sub is not None and time.time() - self._camera_wanted > 10.0:
            self.destroy_subscription(self._camera_sub)
            self._camera_sub = None
            self.camera_jpeg = None

    # ------------------------------------------------------------ teleop
    def _teleop_tick(self):
        cmd = self.teleop.tick()
        if cmd is not None:
            t = Twist()
            t.linear.x, t.angular.z = cmd
            self.cmd_vel.publish(t)

    def drive(self, vx: float, wz: float):
        if self.nav_state["active"]:
            self.cancel_nav()               # the operator takes over from Nav2
        self.teleop.command(vx, wz)

    # ------------------------------------------------------------- state
    def _pose(self) -> Optional[dict]:
        for frame in (self.map_frame, self.odom_frame):
            try:
                tf = self.tf_buffer.lookup_transform(frame, self.base_frame, Time())
            except tf2_ros.TransformException:
                continue
            q, p = tf.transform.rotation, tf.transform.translation
            age = self.get_clock().now().nanoseconds * 1e-9 - Time.from_msg(tf.header.stamp).nanoseconds * 1e-9
            return {"x": round(p.x, 3), "y": round(p.y, 3), "yaw": round(quat_to_yaw(q.x, q.y, q.z, q.w), 3),
                    "frame": frame, "age": round(age, 2)}
        return None

    def _scan_points(self) -> List[List[float]]:
        scan = self.scan
        if scan is None or time.time() - self.stamps.get("scan", 0) > 2.0:
            return []
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, scan.header.frame_id, Time())
        except tf2_ros.TransformException:
            return []
        r = np.asarray(scan.ranges, dtype=np.float32)
        a = scan.angle_min + np.arange(len(r), dtype=np.float32) * scan.angle_increment
        ok = np.isfinite(r) & (r > max(scan.range_min, 0.05)) & (r < min(scan.range_max, 15.0))
        r, a = r[ok], a[ok]
        step = max(1, len(r) // 360)
        r, a = r[::step], a[::step]
        q, p = tf.transform.rotation, tf.transform.translation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        x = p.x + r * np.cos(a + yaw)
        y = p.y + r * np.sin(a + yaw)
        return np.round(np.stack([x, y], 1), 2).tolist()

    def snapshot(self, with_scan: bool = True) -> dict:
        now = time.time()
        with self._lock:
            robot = dict(self.robot)
            fresh = {k: round(now - v, 1) for k, v in self.stamps.items()}
            sup = self.supervisor
            detected = dict(self.detected)
            plan = list(self.plan)
            map_info = dict(self.map_info) if self.map_info else None
        return {
            "type": "state", "t": now,
            "robot": robot, "pose": self._pose(), "age": fresh,
            "nav": dict(self.nav_state), "plan": plan if self.nav_state["active"] else [],
            "supervisor": sup, "detected": detected, "map": map_info,
            "scan": self._scan_points() if with_scan else None,
            "teleop": {"active": self.teleop.active, "max_linear": self.teleop.max_linear,
                       "max_angular": self.teleop.max_angular},
        }

    # ---------------------------------------------------------- commands
    def _done(self, fut: concurrent.futures.Future, value):
        if not fut.done():
            fut.set_result(value)

    def driver_async(self, action: str) -> concurrent.futures.Future:
        out = concurrent.futures.Future()
        client = self.triggers.get(action)
        if client is None:
            self._done(out, (False, f"unknown driver action {action!r}"))
            return out
        if not client.service_is_ready():
            self._done(out, (False, f"{client.srv_name} is not available: is the driver running?"))
            return out
        call = client.call_async(Trigger.Request())
        call.add_done_callback(lambda f: self._done(
            out, (bool(f.result().success), f.result().message) if f.result() else (False, "no answer")))
        return out

    def runstop_async(self, on: bool) -> concurrent.futures.Future:
        out = concurrent.futures.Future()
        if not self.runstop_client.service_is_ready():
            self._done(out, (False, "/runstop is not available: is the driver running?"))
            return out
        call = self.runstop_client.call_async(SetBool.Request(data=bool(on)))
        call.add_done_callback(lambda f: self._done(
            out, (bool(f.result().success), f.result().message) if f.result() else (False, "no answer")))
        return out

    def supervisor_async(self, req: dict) -> concurrent.futures.Future:
        """Send a request to the robot's supervisor; the future gets its reply.

        A request published before DDS has matched the supervisor's
        subscription is simply lost (reliable, but volatile), so wait for the
        match first -- discovery can take a few seconds after either end starts.
        """
        out = concurrent.futures.Future()
        rid = f"gw{next(self._ids)}"

        def send():
            # Up to 10 s when the supervisor is known to be alive (its latched
            # state is fresh), so a request sent right after either end started
            # waits for discovery instead of being refused; 3 s otherwise.
            alive = time.time() - self.stamps.get("supervisor", 0.0) < 5.0
            end = time.time() + (10.0 if alive else 3.0)
            while self.count_subscribers("/panel/supervisor/request") == 0:
                if time.time() > end:
                    self._done(out, (False, "no supervisor on the robot: start control_panel/supervisor there"))
                    return
                time.sleep(0.1)
            self._pending[rid] = out
            self.sup_request.publish(String(data=json.dumps(dict(req, id=rid))))
        threading.Thread(target=send, daemon=True).start()
        return out

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        q = yaw_to_quat(float(yaw))
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = q
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25 ** 2
        cov[35] = math.radians(15) ** 2
        msg.pose.covariance = cov
        self.initialpose.publish(msg)

    def nav_goal_async(self, x: float, y: float, yaw: float) -> concurrent.futures.Future:
        out = concurrent.futures.Future()
        if not self.nav.server_is_ready():
            self._done(out, (False, "navigate_to_pose is not available: start navigation"))
            return out
        self.teleop.release()
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = yaw_to_quat(float(yaw))
        goal.pose = pose
        self.nav_state.update(active=True, status="sent", distance=None, goal=[x, y, yaw], message="")
        send = self.nav.send_goal_async(goal, feedback_callback=self._on_nav_feedback)

        def accepted(f):
            handle = f.result()
            if handle is None or not handle.accepted:
                self.nav_state.update(active=False, status="rejected", message="Nav2 rejected the goal")
                self._done(out, (False, "Nav2 rejected the goal"))
                return
            self._nav_handle = handle
            self.nav_state["status"] = "driving"
            self._done(out, (True, "goal accepted"))
            handle.get_result_async().add_done_callback(lambda r: self._on_nav_result(handle, r))
        send.add_done_callback(accepted)
        return out

    def _on_nav_feedback(self, fb):
        d = getattr(fb.feedback, "distance_remaining", None)
        self.nav_state["distance"] = None if d is None else round(float(d), 2)

    def _on_nav_result(self, handle, fut):
        if handle is not self._nav_handle:
            return                                  # an older goal, superseded
        status = fut.result().status if fut.result() else GoalStatus.STATUS_UNKNOWN
        self.nav_state.update(active=False, status=NAV_STATUS.get(status, str(status)))
        self._nav_handle = None

    def cancel_nav(self) -> None:
        handle = self._nav_handle
        if handle is not None:
            handle.cancel_goal_async()
            self.nav_state.update(status="canceling")

    def estop(self) -> concurrent.futures.Future:
        """Everything that can make the base move, stopped: teleop off, the
        Nav2 goal cancelled, zero velocity out now, and the driver runstopped."""
        self.teleop.release()
        self.cancel_nav()
        self.cmd_vel.publish(Twist())
        return self.runstop_async(True)


def start(cfg: Dict) -> RosIO:
    rclpy.init()
    node = RosIO(cfg)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True, name="ros")
    thread.start()
    node._executor_and_thread = (executor, thread)
    return node


def stop(node: RosIO) -> None:
    """Executor first, then rclpy: the other order aborts the process."""
    executor, thread = node._executor_and_thread
    executor.shutdown(timeout_sec=2.0)
    thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()
