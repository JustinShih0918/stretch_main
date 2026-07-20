#!/usr/bin/env python3
"""Visualization for the VLN demo (RViz).

Consumes /vln/status + /rgb + /odom and publishes:

  /vln/viz_image   sensor_msgs/Image — camera view with a HUD overlay
                   (instruction, state, action batch with the current one
                   highlighted, step count, backend detail).
  /vln/viz_markers visualization_msgs/MarkerArray in the odom frame —
                   the commanded action batch drawn as a trajectory ribbon
                   + heading arrow + floating state text above the robot.
  /vln/path        nav_msgs/Path — breadcrumbs of the executed motion.

Everything is derived from topics, so this node is optional and read-only:
run it beside the agent (vln_demo.launch.py starts it by default).
"""

import math
import threading

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from btcpp_ros2_interfaces.msg import VlnStatus

from .backends.base import BACKWARD, FORWARD, TURN_LEFT, TURN_RIGHT, OdomPose
from .image_rotation import normalize_rgb_rotation, rotate_rgb_image
from .viz_geometry import action_trajectory, records_episode_path, split_actions

# HUD colors (BGR)
_GREEN = (80, 220, 80)
_GRAY = (170, 170, 170)
_WHITE = (255, 255, 255)
_YELLOW = (60, 200, 240)
_RED = (60, 60, 230)

_STATE_COLOR = {
    "IDLE": _GRAY,
    "RESETTING": _YELLOW,
    "THINKING": _YELLOW,
    "EXECUTING": _GREEN,
    "DONE": _GREEN,
    "ERROR": _RED,
}

_GLYPH = {
    FORWARD: "FWD",
    BACKWARD: "BACK",
    TURN_LEFT: "L15",
    TURN_RIGHT: "R15",
    "STOP": "STOP",
}


class VlnVizNode(Node):
    def __init__(self):
        super().__init__("vln_viz_node")

        self.declare_parameter("rgb_topic", "/rgb")
        self.declare_parameter("rgb_rotation", "clockwise_90")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("marker_frame", "odom")
        self.declare_parameter("path_min_step_m", 0.05)
        self.declare_parameter("path_max_poses", 500)

        self._bridge = None
        self.rgb_rotation = normalize_rgb_rotation(
            self.get_parameter("rgb_rotation").value
        )
        self._lock = threading.Lock()
        self._latest_img = None
        self._status = None
        self._odom = None
        self._odom_msg = None
        self._path = Path()
        self._path.header.frame_id = str(
            self.get_parameter("marker_frame").value
        )
        self._episode_active = False

        self.create_subscription(
            Image, str(self.get_parameter("rgb_topic").value),
            self._on_image, 1,
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._on_odom, 10,
        )
        self.create_subscription(VlnStatus, "/vln/status", self._on_status, 1)
        self.create_subscription(
            String, "/vln_instruction", self._on_instruction, 1
        )

        self.image_pub = self.create_publisher(Image, "/vln/viz_image", 1)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/vln/viz_markers", 1
        )
        self.path_pub = self.create_publisher(Path, "/vln/path", 1)

        self.create_timer(0.1, self._publish_image)    # 10 Hz HUD
        self.create_timer(0.2, self._publish_markers)  # 5 Hz markers
        self.get_logger().info(
            f"vln_viz_node ready: rgb_rotation={self.rgb_rotation} -> "
            "/vln/viz_image /vln/viz_markers /vln/path"
        )

    # -- inputs -----------------------------------------------------------
    def _on_image(self, msg: Image):
        with self._lock:
            self._latest_img = msg

    def _on_status(self, msg: VlnStatus):
        with self._lock:
            self._status = msg
            self._episode_active = records_episode_path(msg.state)

    def _on_instruction(self, msg: String):
        if not msg.data.strip():
            return
        # The instruction topic is the episode boundary.  Unlike RESETTING,
        # it cannot be hidden by a fast status transition, and it also works
        # when the exact same instruction is submitted twice.
        with self._lock:
            self._episode_active = True
            odom_msg = self._odom_msg
        self._clear_episode_visuals(
            self.get_clock().now().to_msg(), odom_msg
        )

    def _on_odom(self, msg: Odometry):
        pose = msg.pose.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z
                   + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2),
        )
        with self._lock:
            self._odom = OdomPose(pose.position.x, pose.position.y, yaw)
            self._odom_msg = msg
            episode_active = self._episode_active
        if episode_active:
            self._append_path(msg)

    # -- executed path ----------------------------------------------------
    def _clear_episode_visuals(self, stamp, odom_msg=None):
        """Drop breadcrumbs and commanded markers from the prior episode."""
        path = Path()
        path.header.frame_id = str(
            self.get_parameter("marker_frame").value
        )
        path.header.stamp = stamp
        # RViz reliably replaces the previous Path when the new message has a
        # pose.  Some RViz versions leave the old drawing visible after a
        # completely empty Path, so seed the episode at the current odometry.
        if odom_msg is not None:
            start = PoseStamped()
            start.header = odom_msg.header
            start.pose = odom_msg.pose.pose
            path.poses.append(start)
            path.header.stamp = odom_msg.header.stamp
        with self._lock:
            self._path = path
        self.path_pub.publish(path)

        clear = self._marker(0, Marker.LINE_STRIP)
        clear.action = Marker.DELETEALL
        clear.header.stamp = stamp
        self.marker_pub.publish(MarkerArray(markers=[clear]))

    def _append_path(self, msg: Odometry):
        min_step = float(self.get_parameter("path_min_step_m").value)
        with self._lock:
            if self._path.poses:
                last = self._path.poses[-1].pose.position
                p = msg.pose.pose.position
                if math.hypot(p.x - last.x, p.y - last.y) < min_step:
                    return
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            self._path.poses.append(ps)
            max_poses = int(self.get_parameter("path_max_poses").value)
            if len(self._path.poses) > max_poses:
                self._path.poses = self._path.poses[-max_poses:]
            self._path.header.stamp = msg.header.stamp
            self.path_pub.publish(self._path)

    # -- HUD image --------------------------------------------------------
    def _publish_image(self):
        with self._lock:
            img_msg = self._latest_img
            status = self._status
        if img_msg is None:
            return

        import cv2
        if self._bridge is None:
            from cv_bridge import CvBridge
            self._bridge = CvBridge()

        frame = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
        frame = rotate_rgb_image(frame, self.rgb_rotation)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        h, w = frame.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.45, w / 1400.0)
        line_h = int(30 * scale / 0.45)

        # translucent HUD bar
        bar_h = int(3.4 * line_h)
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, bar_h), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.55, frame, 0.45, 0)

        if status is None:
            cv2.putText(frame, "waiting for /vln/status ...",
                        (10, line_h), font, scale, _GRAY, 1, cv2.LINE_AA)
        else:
            instr = status.instruction or "(no instruction yet)"
            if len(instr) > 90:
                instr = instr[:87] + "..."
            cv2.putText(frame, f'"{instr}"', (10, line_h),
                        font, scale, _WHITE, 1, cv2.LINE_AA)

            color = _STATE_COLOR.get(status.state, _WHITE)
            line2 = (f"{status.state}  step {status.step_count}"
                     f"  [{status.backend}/{status.execution_mode}]")
            if status.detail:
                line2 += f"  {status.detail}"
            cv2.putText(frame, line2, (10, 2 * line_h),
                        font, scale, color, 1, cv2.LINE_AA)

            # action batch: current highlighted, pending gray
            x = 10
            y = 3 * line_h
            current = status.current_action or ""
            for token in current.split("+"):
                if not token:
                    continue
                text = _GLYPH.get(token, token)
                (tw, _), _ = cv2.getTextSize(text, font, scale, 2)
                cv2.putText(frame, text, (x, y), font, scale, _GREEN, 2,
                            cv2.LINE_AA)
                x += tw + int(14 * scale / 0.45)
            for token in status.pending_actions:
                text = _GLYPH.get(token, token)
                (tw, _), _ = cv2.getTextSize(text, font, scale, 1)
                cv2.putText(frame, text, (x, y), font, scale, _GRAY, 1,
                            cv2.LINE_AA)
                x += tw + int(14 * scale / 0.45)

        out = self._bridge.cv2_to_imgmsg(
            cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), encoding="rgb8"
        )
        out.header = img_msg.header
        self.image_pub.publish(out)

    # -- 3D markers -------------------------------------------------------
    def _marker(self, mid, mtype):
        m = Marker()
        m.header.frame_id = str(self.get_parameter("marker_frame").value)
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "vln"
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.lifetime = Duration(seconds=1.0).to_msg()
        return m

    def _publish_markers(self):
        with self._lock:
            status = self._status
            odom = self._odom
        if status is None or odom is None:
            return

        markers = MarkerArray()

        # floating state text above the robot
        text = self._marker(0, Marker.TEXT_VIEW_FACING)
        text.pose.position.x = odom.x
        text.pose.position.y = odom.y
        text.pose.position.z = 1.6
        text.scale.z = 0.18
        r, g, b = _STATE_COLOR.get(status.state, _WHITE)[::-1]
        text.color = ColorRGBA(r=r / 255.0, g=g / 255.0, b=b / 255.0, a=1.0)
        label = status.state
        if status.current_action:
            label += f": {status.current_action}"
        if status.pending_actions:
            label += f" (+{len(status.pending_actions)})"
        text.text = label
        markers.markers.append(text)

        # commanded batch as a trajectory ribbon + final heading arrow
        actions = split_actions(status)
        if actions:
            points, final_yaw = action_trajectory(odom, actions)

            if len(points) > 1:
                ribbon = self._marker(1, Marker.LINE_STRIP)
                ribbon.scale.x = 0.05
                ribbon.color = ColorRGBA(r=0.3, g=0.85, b=0.3, a=0.9)
                ribbon.points = [
                    Point(x=px, y=py, z=0.05) for px, py in points
                ]
                markers.markers.append(ribbon)

            arrow = self._marker(2, Marker.ARROW)
            ax, ay = points[-1]
            arrow.points = [
                Point(x=ax, y=ay, z=0.05),
                Point(x=ax + 0.3 * math.cos(final_yaw),
                      y=ay + 0.3 * math.sin(final_yaw), z=0.05),
            ]
            arrow.scale.x = 0.06   # shaft
            arrow.scale.y = 0.12   # head width
            arrow.scale.z = 0.12   # head length
            arrow.color = ColorRGBA(r=0.95, g=0.75, b=0.2, a=0.95)
            markers.markers.append(arrow)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = VlnVizNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
