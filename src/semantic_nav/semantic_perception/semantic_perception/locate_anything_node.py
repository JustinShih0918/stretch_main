#!/usr/bin/env python3
"""LocateAnything perception node for action-aware semantic navigation."""

import os
import sys

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from btcpp_ros2_interfaces.msg import SemanticDetection2D

from .locate_anything_parser import parse_labeled_boxes


class LocateAnythingNode(Node):
    def __init__(self):
        super().__init__("locate_anything_node")

        self.declare_parameter("rgb_topic", "/rgb")
        self.declare_parameter("detection_topic", "detection")
        self.declare_parameter("targets_file", "")
        self.declare_parameter("prompt", "")
        self.declare_parameter(
            "model_path", "/opt/locate_anything/LocateAnything-3B"
        )
        self.declare_parameter(
            "worker_path",
            "/home/user/stretch_main/src/models/Eagle/Embodied",
        )
        self.declare_parameter("device", "cuda")
        self.declare_parameter("dtype", "bfloat16")
        self.declare_parameter("generation_mode", "hybrid")
        self.declare_parameter("max_new_tokens", 512)

        rgb_topic = self.get_parameter("rgb_topic").value
        detection_topic = self.get_parameter("detection_topic").value
        self.targets = self._load_targets(
            self.get_parameter("targets_file").value
        )
        self.prompt = str(self.get_parameter("prompt").value).strip()

        self._worker = None
        self._bridge = None
        self._model_load_failed = False

        self.pub = self.create_publisher(
            SemanticDetection2D, detection_topic, 10
        )
        self.sub = self.create_subscription(
            Image, rgb_topic, self._on_image, 1
        )
        self.instruction_sub = self.create_subscription(
            String, "~/instruction", self._on_instruction, 1
        )

        mode = "phrase grounding" if self.prompt else "target detection"
        self.get_logger().info(
            f"locate_anything_node up: rgb={rgb_topic} -> "
            f"{detection_topic}; mode={mode}"
        )

    def _load_targets(self, path: str) -> dict:
        if not path or not os.path.isfile(path):
            self.get_logger().warn(
                f"targets_file '{path}' not found; "
                "defaulting to curtain=traversable"
            )
            return {"curtain": True}
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        targets = data.get("semantic_targets", data)
        return {str(label): bool(value) for label, value in targets.items()}

    def _on_instruction(self, msg: String):
        self.prompt = msg.data.strip()
        self.get_logger().info(f"active prompt set to '{self.prompt}'")

    def _attribute_for_label(self, phrase: str) -> tuple[str, bool]:
        phrase_lower = phrase.lower().strip()
        for label, traversable in self.targets.items():
            label_lower = label.lower()
            if label_lower in phrase_lower or phrase_lower in label_lower:
                return label, traversable
        return phrase.strip(), False

    def _ensure_model(self) -> bool:
        if self._worker is not None:
            return True
        if self._model_load_failed:
            return False

        try:
            import torch
            from cv_bridge import CvBridge

            worker_path = self.get_parameter("worker_path").value
            worker_file = os.path.join(
                worker_path, "locateanything_worker.py"
            )
            if not os.path.isfile(worker_file):
                raise FileNotFoundError(
                    f"LocateAnything worker not found at {worker_file}"
                )
            if worker_path not in sys.path:
                sys.path.insert(0, worker_path)
            from locateanything_worker import LocateAnythingWorker

            dtype_name = str(self.get_parameter("dtype").value).lower()
            dtype_by_name = {
                "bfloat16": torch.bfloat16,
                "float16": torch.float16,
                "float32": torch.float32,
            }
            if dtype_name not in dtype_by_name:
                raise ValueError(
                    "dtype must be one of: bfloat16, float16, float32"
                )

            model_path = self.get_parameter("model_path").value
            device = self.get_parameter("device").value
            self._worker = LocateAnythingWorker(
                model_path,
                device=device,
                dtype=dtype_by_name[dtype_name],
            )
            self._bridge = CvBridge()
        except Exception as exc:  # pragma: no cover - container/GPU dependent
            self._model_load_failed = True
            self.get_logger().error(
                f"LocateAnything unavailable ({exc}); detections disabled. "
                "Build the sim image with LOCATE_ANYTHING=YES."
            )
            return False

        self.get_logger().info("LocateAnything model loaded")
        return True

    def _on_image(self, msg: Image):
        if not self._ensure_model():
            return

        from PIL import Image as PILImage

        rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        image = PILImage.fromarray(rgb)
        generation_kwargs = {
            "generation_mode": self.get_parameter(
                "generation_mode"
            ).value,
            "max_new_tokens": int(
                self.get_parameter("max_new_tokens").value
            ),
            "verbose": False,
        }

        try:
            if self.prompt:
                result = self._worker.ground_multi(
                    image, self.prompt, **generation_kwargs
                )
                fallback_label = self.prompt
            else:
                result = self._worker.detect(
                    image, list(self.targets), **generation_kwargs
                )
                fallback_label = None
        except Exception as exc:  # pragma: no cover - model dependent
            self.get_logger().error(f"LocateAnything inference failed: {exc}")
            return

        height, width = rgb.shape[:2]
        detections = parse_labeled_boxes(
            str(result.get("answer", "")),
            width,
            height,
            fallback_label=fallback_label,
        )
        for box in detections:
            label, traversable = self._attribute_for_label(box["label"])
            detection = SemanticDetection2D()
            detection.header = msg.header
            detection.label = label
            detection.traversable = traversable
            # LocateAnything's generated output has no calibrated score.
            detection.confidence = 1.0
            detection.x = box["x"]
            detection.y = box["y"]
            detection.width = box["width"]
            detection.height = box["height"]
            self.pub.publish(detection)


def main(args=None):
    rclpy.init(args=args)
    node = LocateAnythingNode()
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
