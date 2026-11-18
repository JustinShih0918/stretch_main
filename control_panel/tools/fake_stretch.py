#!/usr/bin/env python3
"""A pretend Stretch for testing the control panel with no robot.

    python3 control_panel/tools/fake_stretch.py driver|lidar|camera|slam|nav

Each role stands in for one real unit, under the real node names, topics,
services and action, so the supervisor starts these instead of hello-robot's
launch files (control_panel/tests/fake_units.yaml) and the gateway cannot tell
the difference:

  driver  node stretch_driver: /home_the_robot, /stow_the_robot, /stop_the_robot,
          /switch_to_{navigation,position,gamepad,trajectory}_mode, /runstop;
          /mode /is_homed /is_runstopped /battery /odom; obeys /stretch/cmd_vel
          only in navigation mode and not runstopped (as stretch_driver does,
          with its 0.5 s timeout); TF odom -> base_link -> laser
  lidar   node sllidar_node: /scan and /scan_filtered, ray-cast in a made-up floor
  camera  node camera: /camera/color/image_raw/compressed (JPEG, sideways like the D435i)
  slam    node slam_toolbox: /map, TF map -> odom, /initialpose,
          /slam_toolbox/serialize_map (writes <file>.posegraph/.data);
          `slam localization <file>` refuses a posegraph that does not exist
  nav     node bt_navigator: navigate_to_pose, driving by /stretch/cmd_vel, /plan
  bridge  node osg_bridge, idle: the OSG bridge the panel's pre-flight check looks for
          (tools/fake_osg_run.py publishes what OSG would, itself)

The robot's true pose is shared between roles on /fake/truth so the lidar
sees what the driver did.
"""
from __future__ import annotations

import io
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, CompressedImage, LaserScan
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
import tf2_ros

RES = 0.05
W_M, H_M = 16.0, 10.0
ORIGIN = (-3.0, -3.0)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def world() -> np.ndarray:
    """A made-up floor: an outer wall, a dividing wall with a door, two boxes. True = wall."""
    h, w = int(H_M / RES), int(W_M / RES)
    g = np.zeros((h, w), bool)
    g[:4, :] = g[-4:, :] = True
    g[:, :4] = g[:, -4:] = True
    c = int(8.0 / RES)
    g[:, c:c + 4] = True
    g[int(4.0 / RES):int(5.5 / RES), c:c + 4] = False          # the door
    g[int(1.0 / RES):int(2.0 / RES), int(3.0 / RES):int(4.2 / RES)] = True
    g[int(6.5 / RES):int(7.3 / RES), int(11.0 / RES):int(13.0 / RES)] = True
    return g                                                   # row 0 = y origin (OccupancyGrid order)


def cell(x, y):
    return int((y - ORIGIN[1]) / RES), int((x - ORIGIN[0]) / RES)


def yaw_q(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def tf_msg(stamp, parent, child, x, y, yaw):
    t = TransformStamped()
    t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
    t.transform.translation.x, t.transform.translation.y = float(x), float(y)
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = yaw_q(yaw)
    return t


class Driver(Node):
    def __init__(self):
        super().__init__("stretch_driver")
        self.mode, self.homed, self.runstopped = "navigation", False, False
        self.x, self.y, self.yaw = 1.0, 1.0, 0.0           # odom frame == truth here
        self.v, self.w, self.last_twist = 0.0, 0.0, 0.0
        for name, fn in [("/home_the_robot", self._home), ("/stow_the_robot", self._ok("Stowed.")),
                         ("/stop_the_robot", self._ok("Stopped the robot.")),
                         ("/switch_to_navigation_mode", self._mode("navigation")),
                         ("/switch_to_position_mode", self._mode("position")),
                         ("/switch_to_gamepad_mode", self._mode("gamepad")),
                         ("/switch_to_trajectory_mode", self._mode("trajectory"))]:
            self.create_service(Trigger, name, fn)
        self.create_service(SetBool, "/runstop", self._runstop)
        self.create_subscription(Twist, "/stretch/cmd_vel", self._cmd, 1)
        self.pub = {k: self.create_publisher(t, k, 1) for k, t in
                    [("/mode", String), ("/is_homed", Bool), ("/is_runstopped", Bool), ("/battery", BatteryState)]}
        self.odom = self.create_publisher(Odometry, "/odom", 1)
        self.truth = self.create_publisher(Float64MultiArray, "/fake/truth", 1)
        self.tf = tf2_ros.TransformBroadcaster(self)
        self.static = tf2_ros.StaticTransformBroadcaster(self)
        self.static.sendTransform(tf_msg(self.get_clock().now().to_msg(), "base_link", "laser", 0.0, 0.0, math.pi))
        self.walls = world()
        self.create_timer(1 / 30, self._step)
        self.create_timer(0.5, self._status)

    def _ok(self, text):
        def f(_req, res):
            res.success, res.message = True, text
            return res
        return f

    def _home(self, _req, res):
        if self.runstopped:
            res.success, res.message = False, "runstopped"
            return res
        time.sleep(1.0)
        self.homed = True
        res.success, res.message = True, "Homed."
        return res

    def _mode(self, mode):
        def f(_req, res):
            self.mode = mode
            res.success, res.message = True, f"Now in {mode} mode."
            return res
        return f

    def _runstop(self, req, res):
        self.runstopped = bool(req.data)
        self.v = self.w = 0.0
        res.success, res.message = True, "is_runstopped: %s" % self.runstopped
        return res

    def _cmd(self, t: Twist):
        if self.mode != "navigation":
            self.get_logger().error("must be in navigation mode to receive a twist on cmd_vel. "
                                    f"Current mode = {self.mode}.")
            return
        self.v, self.w, self.last_twist = t.linear.x, t.angular.z, time.time()

    def _step(self):
        dt = 1 / 30
        if self.runstopped or time.time() - self.last_twist > 0.5:     # stretch_driver's timeout
            self.v = self.w = 0.0
        nx = self.x + self.v * math.cos(self.yaw) * dt
        ny = self.y + self.v * math.sin(self.yaw) * dt
        r, c = cell(nx, ny)
        if 0 <= r < self.walls.shape[0] and 0 <= c < self.walls.shape[1] and not self.walls[max(r - 3, 0):r + 4, max(c - 3, 0):c + 4].any():
            self.x, self.y = nx, ny
        self.yaw = math.atan2(math.sin(self.yaw + self.w * dt), math.cos(self.yaw + self.w * dt))
        stamp = self.get_clock().now().to_msg()
        self.tf.sendTransform(tf_msg(stamp, "odom", "base_link", self.x, self.y, self.yaw))
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "odom", "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.odom.publish(o)
        self.truth.publish(Float64MultiArray(data=[self.x, self.y, self.yaw]))

    def _status(self):
        self.pub["/mode"].publish(String(data=self.mode))
        self.pub["/is_homed"].publish(Bool(data=self.homed))
        self.pub["/is_runstopped"].publish(Bool(data=self.runstopped))
        self.pub["/battery"].publish(BatteryState(voltage=12.1))


class Lidar(Node):
    def __init__(self):
        super().__init__("sllidar_node")
        self.walls = world()
        self.pose = None
        self.create_subscription(Float64MultiArray, "/fake/truth", lambda m: setattr(self, "pose", list(m.data)), 1)
        self.pubs = [self.create_publisher(LaserScan, t, qos_profile_sensor_data) for t in ("/scan", "/scan_filtered")]
        self.create_timer(0.2, self._scan)

    def _scan(self):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        n = 360
        angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
        ranges = np.full(n, np.inf, np.float32)
        steps = np.arange(0.1, 12.0, RES)
        for k, a in enumerate(angles):
            th = yaw + math.pi + a                       # the laser is mounted backwards (yaw pi)
            xs, ys = x + steps * math.cos(th), y + steps * math.sin(th)
            rr = ((ys - ORIGIN[1]) / RES).astype(int)
            cc = ((xs - ORIGIN[0]) / RES).astype(int)
            inside = (rr >= 0) & (rr < self.walls.shape[0]) & (cc >= 0) & (cc < self.walls.shape[1])
            hit = np.zeros_like(inside)
            hit[inside] = self.walls[rr[inside], cc[inside]]
            idx = np.argmax(hit)
            if hit[idx]:
                ranges[k] = steps[idx]
        msg = LaserScan()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), "laser"
        msg.angle_min, msg.angle_max = -math.pi, math.pi - 2 * math.pi / n
        msg.angle_increment = 2 * math.pi / n
        msg.range_min, msg.range_max = 0.15, 12.0
        msg.ranges = ranges.tolist()
        for p in self.pubs:
            p.publish(msg)


class Camera(Node):
    def __init__(self):
        super().__init__("camera")
        self.pub = self.create_publisher(CompressedImage, "/camera/color/image_raw/compressed", qos_profile_sensor_data)
        self.depth = self.create_publisher(CompressedImage, "/camera/aligned_depth_to_color/image_raw/compressedDepth",
                                           qos_profile_sensor_data)
        self.create_timer(0.5, self._frame)

    def _frame(self):
        from PIL import Image, ImageDraw
        img = Image.new("RGB", (360, 640), (40, 60, 90))
        d = ImageDraw.Draw(img)
        d.text((20, 300), time.strftime("fake D435i %H:%M:%S"), fill=(255, 255, 255))
        img = img.rotate(90, expand=True)                 # the head camera is mounted sideways
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=70)
        msg = CompressedImage(format="rgb8; jpeg compressed bgr8", data=buf.getvalue())
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


class Slam(Node):
    def __init__(self, mode: str, posegraph: str = ""):
        super().__init__("slam_toolbox")
        from slam_toolbox.srv import SerializePoseGraph
        if mode == "localization":
            if not Path(posegraph + ".posegraph").exists():
                self.get_logger().fatal(f"no pose graph at {posegraph}.posegraph")
                raise SystemExit(1)
        self.mode = mode
        self.offset = [0.0, 0.0, 0.0]                     # map -> odom
        self.walls = world()
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", LATCHED)
        self.tf = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._initialpose, 10)
        self.create_service(SerializePoseGraph, "/slam_toolbox/serialize_map", self._serialize)
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self._tf)
        self.create_timer(2.0, self._map)

    def _tf(self):
        x, y, yaw = self.offset
        self.tf.sendTransform(tf_msg(self.get_clock().now().to_msg(), "map", "odom", x, y, yaw))

    def _map(self):
        m = OccupancyGrid()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), "map"
        m.info.resolution = RES
        m.info.height, m.info.width = self.walls.shape
        m.info.origin.position.x, m.info.origin.position.y = ORIGIN
        m.info.origin.orientation.w = 1.0
        data = np.where(self.walls, 100, 0).astype(np.int8)
        data[:, int(12.5 / RES):] = -1 if self.mode == "mapping" else data[:, int(12.5 / RES):]
        m.data = data.ravel().tolist()
        self.map_pub.publish(m)

    def _initialpose(self, msg: PoseWithCovarianceStamped):
        """Make base_link land where the operator said: map->odom = want * (odom->base)^-1."""
        try:
            ob = self.tf_buffer.lookup_transform("odom", "base_link", Time())
        except tf2_ros.TransformException:
            return
        q = msg.pose.pose.orientation
        want = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        oq = ob.transform.rotation
        oyaw = math.atan2(2 * (oq.w * oq.z), 1 - 2 * oq.z * oq.z)
        yaw = want[2] - oyaw
        ox, oy = ob.transform.translation.x, ob.transform.translation.y
        self.offset = [want[0] - (ox * math.cos(yaw) - oy * math.sin(yaw)),
                       want[1] - (ox * math.sin(yaw) + oy * math.cos(yaw)), yaw]
        self.get_logger().info(f"relocalised to {want}")

    def _serialize(self, req, res):
        Path(req.filename + ".posegraph").write_bytes(b"fake pose graph\n")
        Path(req.filename + ".data").write_bytes(b"fake data\n")
        res.result = 0
        return res


class Nav(Node):
    def __init__(self):
        super().__init__("bt_navigator")
        group = ReentrantCallbackGroup()
        self.cmd = self.create_publisher(Twist, "/stretch/cmd_vel", 10)
        self.plan = self.create_publisher(PathMsg, "/plan", 10)
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)
        ActionServer(self, NavigateToPose, "navigate_to_pose", self._execute, callback_group=group,
                     goal_callback=lambda g: GoalResponse.ACCEPT,
                     cancel_callback=lambda g: CancelResponse.ACCEPT)

    def _pose(self):
        tf = self.tf_buffer.lookup_transform("map", "base_link", Time())
        q = tf.transform.rotation
        return tf.transform.translation.x, tf.transform.translation.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)

    def _execute(self, handle):
        gx, gy = handle.request.pose.pose.position.x, handle.request.pose.pose.position.y
        fb = NavigateToPose.Feedback()
        t0 = time.time()
        while rclpy.ok():
            if handle.is_cancel_requested:
                self.cmd.publish(Twist())
                handle.canceled()
                return NavigateToPose.Result()
            try:
                x, y, yaw = self._pose()
            except tf2_ros.TransformException:
                time.sleep(0.1)
                continue
            d = math.hypot(gx - x, gy - y)
            path = PathMsg()
            path.header.frame_id = "map"
            for k in range(11):
                p = PoseStamped()
                p.pose.position.x, p.pose.position.y = x + (gx - x) * k / 10, y + (gy - y) * k / 10
                path.poses.append(p)
            self.plan.publish(path)
            fb.distance_remaining = float(d)
            handle.publish_feedback(fb)
            if d < 0.15:
                self.cmd.publish(Twist())
                handle.succeed()
                return NavigateToPose.Result()
            if time.time() - t0 > 120:
                self.cmd.publish(Twist())
                handle.abort()
                return NavigateToPose.Result()
            err = math.atan2(math.sin(math.atan2(gy - y, gx - x) - yaw), math.cos(math.atan2(gy - y, gx - x) - yaw))
            t = Twist()
            t.angular.z = max(-1.0, min(1.0, 2.0 * err))
            t.linear.x = 0.3 if abs(err) < 0.4 else 0.0
            self.cmd.publish(t)
            time.sleep(0.1)


def main():
    role = sys.argv[1] if len(sys.argv) > 1 else "driver"
    rclpy.init()
    node = {"driver": Driver, "lidar": Lidar, "camera": Camera, "nav": Nav,
            "bridge": lambda: Node("osg_bridge")}.get(role)
    if role == "slam":
        mode = sys.argv[2] if len(sys.argv) > 2 else "mapping"
        node = Slam(mode, sys.argv[3] if len(sys.argv) > 3 else "")
    elif node is None:
        raise SystemExit(f"role? driver|lidar|camera|slam|nav|bridge, not {role}")
    else:
        node = node()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
