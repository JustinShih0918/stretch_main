#!/usr/bin/env python3
"""Draws VLM bounding boxes + labels onto the RGB image and
publishes the annotated result on /semantic_detection_viz for RViz."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from btcpp_ros2_interfaces.msg import SemanticDetection2D
from cv_bridge import CvBridge
import cv2
import threading


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

        self.create_subscription(Image, "/rgb", self._on_image, 1)
        self.create_subscription(
            SemanticDetection2D, "/semantic_detection", self._on_det, 10
        )
        self._pub = self.create_publisher(Image, "/semantic_detection_viz", 1)
        self.create_timer(0.1, self._publish)          # 10 Hz output
        self.get_logger().info(
            "detection_viz_node ready → /semantic_detection_viz"
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
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        for det in detections:
            col = _COL_TRAV if det.traversable else _COL_NTRAV
            x1, y1 = det.x, det.y
            x2, y2 = det.x + det.width, det.y + det.height
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
