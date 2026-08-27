#!/usr/bin/env python3
"""Open-set VLM perception node (paper 2310.08873, Grounding DINO).

Subscribes an RGB image, runs Grounding DINO with a text prompt built from the
static action-aware target map, and publishes one SemanticDetection2D per
detected landmark (bounding box + label + traversable attribute + confidence).

The prompt is also runtime-settable on ~/instruction (std_msgs/String) so a BT
node (or, later, an LLM) can switch the active landmark interactively.

Grounding DINO (torch + weights) is imported lazily: the node still starts if
the model is unavailable, logging a clear error, so the rest of the pipeline
(projection + costmap layer) can be exercised with a static region publisher.
"""

import os

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

from btcpp_ros2_interfaces.msg import SemanticDetection2D

from .image_rotation import (
    normalize_rgb_rotation,
    rotate_rgb_image,
    unrotate_box,
)


class GroundingDinoNode(Node):
    def __init__(self):
        super().__init__("grounding_dino_node")

        self.declare_parameter("rgb_topic", "/rgb")
        self.declare_parameter("detection_topic", "detection")
        self.declare_parameter("targets_file", "")
        self.declare_parameter("prompt", "")  # overrides targets-derived prompt
        self.declare_parameter("box_threshold", 0.35)
        self.declare_parameter("text_threshold", 0.25)
        self.declare_parameter("model_config", "")
        self.declare_parameter("model_weights", "")
        self.declare_parameter("device", "cuda")
        # none | clockwise_90 | counterclockwise_90 | 180
        self.declare_parameter("rgb_rotation", "clockwise_90")

        rgb_topic = self.get_parameter("rgb_topic").value
        det_topic = self.get_parameter("detection_topic").value
        self.box_threshold = float(self.get_parameter("box_threshold").value)
        self.text_threshold = float(self.get_parameter("text_threshold").value)
        self.rgb_rotation = normalize_rgb_rotation(
            self.get_parameter("rgb_rotation").value
        )

        self.targets = self._load_targets(self.get_parameter("targets_file").value)
        prompt_param = self.get_parameter("prompt").value
        self.prompt = prompt_param or self._prompt_from_targets()

        self._model = None  # lazy
        self._bridge = None

        self.pub = self.create_publisher(SemanticDetection2D, det_topic, 10)
        self.sub = self.create_subscription(Image, rgb_topic, self._on_image, 1)
        self.instr_sub = self.create_subscription(
            String, "~/instruction", self._on_instruction, 1
        )

        self.get_logger().info(
            f"grounding_dino_node up: rgb={rgb_topic} -> {det_topic}; "
            f"prompt='{self.prompt}'; rgb_rotation={self.rgb_rotation}"
        )

    # ---- config helpers -------------------------------------------------
    def _load_targets(self, path: str) -> dict:
        if not path or not os.path.isfile(path):
            self.get_logger().warn(
                f"targets_file '{path}' not found; defaulting to curtain=traversable"
            )
            return {"curtain": True}
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        targets = data.get("semantic_targets", data)
        return {str(k): bool(v) for k, v in targets.items()}

    def _prompt_from_targets(self) -> str:
        # Grounding DINO caption format: phrases separated by " . ".
        return " . ".join(self.targets.keys())

    def _attr_for_label(self, phrase: str) -> (str, bool):
        # Match the detected phrase against known labels (substring both ways).
        phrase_l = phrase.lower().strip()
        for label, traversable in self.targets.items():
            if label in phrase_l or phrase_l in label:
                return label, traversable
        return phrase_l, False

    def _on_instruction(self, msg: String):
        self.prompt = msg.data
        self.get_logger().info(f"active prompt set to '{self.prompt}'")

    # ---- model ----------------------------------------------------------
    def _ensure_model(self) -> bool:
        if self._model is not None:
            return True
        try:
            import torch  # noqa: F401
            from groundingdino.util.inference import load_model
            from cv_bridge import CvBridge
        except Exception as exc:  # pragma: no cover - depends on container
            self.get_logger().error(
                f"Grounding DINO / cv_bridge unavailable ({exc}); detections "
                "disabled. Install via docker/sim/modules/install_grounding_dino.sh"
            )
            return False

        cfg = self.get_parameter("model_config").value
        weights = self.get_parameter("model_weights").value
        if not cfg or not weights:
            self.get_logger().error(
                "model_config / model_weights params are empty; cannot load model"
            )
            return False
        self._bridge = CvBridge()
        self._device = self.get_parameter("device").value
        self._model = load_model(cfg, weights)
        self.get_logger().info("Grounding DINO model loaded")
        return True

    # ---- inference ------------------------------------------------------
    def _on_image(self, msg: Image):
        if not self._ensure_model():
            return
        from groundingdino.util.inference import predict
        import groundingdino.datasets.transforms as T
        from PIL import Image as PILImage
        import numpy as np

        cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        # The model sees the upright frame; boxes are mapped back below.
        model_img = rotate_rgb_image(cv_img, self.rgb_rotation)
        pil = PILImage.fromarray(model_img)
        transform = T.Compose(
            [
                T.RandomResize([800], max_size=1333),
                T.ToTensor(),
                T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )
        image_tensor, _ = transform(pil, None)

        boxes, logits, phrases = predict(
            model=self._model,
            image=image_tensor,
            caption=self.prompt,
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            device=self._device,
        )

        h, w = model_img.shape[:2]
        for box, score, phrase in zip(boxes, logits, phrases):
            # box is normalized cx, cy, bw, bh in [0, 1] of the rotated image.
            cx, cy, bw, bh = (float(v) for v in box)
            px = max(0, int((cx - bw / 2.0) * w))
            py = max(0, int((cy - bh / 2.0) * h))
            pw = max(0, int(bw * w))
            ph = max(0, int(bh * h))
            label, traversable = self._attr_for_label(phrase)

            # Back to raw-camera pixels: projection_node pairs these with the
            # unrotated /depth + /camera_info.
            px, py, pw, ph = unrotate_box(
                (px, py, pw, ph), self.rgb_rotation, w, h
            )

            det = SemanticDetection2D()
            det.header = msg.header
            det.label = label
            det.traversable = traversable
            det.confidence = float(score)
            det.x = max(0, px)
            det.y = max(0, py)
            det.width = max(0, pw)
            det.height = max(0, ph)
            self.pub.publish(det)


def main(args=None):
    rclpy.init(args=args)
    node = GroundingDinoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
