#!/usr/bin/env python3
"""Draws VLM bounding boxes + labels onto the RGB image and
publishes the annotated result on /semantic_detection_viz for RViz."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from btcpp_ros2_interfaces.msg import SemanticDetection2D
from cv_bridge import CvBridge
import cv2
import threading

from .image_rotation import (
    normalize_rgb_rotation,
    rotate_box,
    rotate_rgb_image,
)

# Match the camera, not the default: RealSense publishes BEST_EFFORT, and a
# RELIABLE subscriber silently receives nothing from it. BEST_EFFORT also
# matches Isaac's RELIABLE publisher, so this works in both environments.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# Colour per traversability
_COL_TRAV = (50, 220, 50)    # green  – traversable
_COL_NTRAV = (50, 50, 220)   # red-ish (BGR) – not traversable
_FONT = cv2.FONT_HERSHEY_SIMPLEX


class DetectionVizNode(Node):
    def __init__(self):
        super().__init__("detection_viz_node")
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_img = None
        self._detections = []
        self._latest_stamp = None

        # Topics are parameters (not just remappings) so the whole pipeline can
        # be driven from one params file — see docker/vlm/config/.
        self.declare_parameter("rgb_topic", "/rgb")
        self.declare_parameter("detection_topic", "/semantic_detection")
        self.declare_parameter("viz_topic", "/semantic_detection_viz")
        # Cosmetic only — detections arrive in raw-image pixels either way, so
        # the boxes are rotated along with the frame. Keep it equal to the
        # perception node's rgb_rotation to see what the VLM sees.
        self.declare_parameter("rgb_rotation", "none")
        rgb_topic = self.get_parameter("rgb_topic").value
        detection_topic = self.get_parameter("detection_topic").value
        viz_topic = self.get_parameter("viz_topic").value
        self.rgb_rotation = normalize_rgb_rotation(
            self.get_parameter("rgb_rotation").value
        )

        self.create_subscription(Image, rgb_topic, self._on_image, SENSOR_QOS)
        self.create_subscription(
            SemanticDetection2D, detection_topic, self._on_det, 10
        )
        self._pub = self.create_publisher(Image, viz_topic, 1)
        self.create_timer(0.1, self._publish)          # 10 Hz output
        self.get_logger().info(
            f"detection_viz_node ready: rgb={rgb_topic} + {detection_topic} "
            f"-> {viz_topic}"
        )

    def _on_image(self, msg: Image):
        with self._lock:
            self._latest_img = msg

    def _on_det(self, msg: SemanticDetection2D):
        with self._lock:
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if stamp != self._latest_stamp:
                self._latest_stamp = stamp
                self._detections = []
            self._detections.append(msg)

    def _publish(self):
        with self._lock:
            img_msg = self._latest_img
            detections = list(self._detections)
        if img_msg is None:
            return

        frame = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
        raw_height, raw_width = frame.shape[:2]
        frame = rotate_rgb_image(frame, self.rgb_rotation)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        for det in detections:
            col = _COL_TRAV if det.traversable else _COL_NTRAV
            box = rotate_box(
                {"x": det.x, "y": det.y,
                 "width": det.width, "height": det.height},
                self.rgb_rotation,
                raw_width,
                raw_height,
            )
            x1, y1 = box["x"], box["y"]
            x2, y2 = x1 + box["width"], y1 + box["height"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2)

            label = f"{det.label} {det.confidence:.2f}"
            tag = "TRAVERSABLE" if det.traversable else "BLOCKED"
            text = f"{label}  [{tag}]"

            # Background pill for readability
            (tw, th), _ = cv2.getTextSize(text, _FONT, 0.55, 1)
            cv2.rectangle(frame, (x1, y1 - th - 8), (x1 + tw + 4, y1), col, -1)
            cv2.putText(frame, text, (x1 + 2, y1 - 4),
                        _FONT, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

        out = self._bridge.cv2_to_imgmsg(
            cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), encoding="rgb8"
        )
        out.header = img_msg.header
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionVizNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
