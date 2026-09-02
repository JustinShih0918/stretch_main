"""Camera subscription over raw or compressed image transport.

The robot's RealSense publishes both `<topic>` (sensor_msgs/Image) and
`<topic>/compressed` (sensor_msgs/CompressedImage, JPEG). Measured on the
lab robot at 1280x720 / 15 Hz: 2700 KiB per raw frame (333 Mbit/s) against
261 KiB compressed (32 Mbit/s) — the same pixels for a tenth of the wire.
Over a shared link the raw stream is the difference between a live view and a
visibly lagging one, so anything reading the robot's camera across the network
should prefer `compressed`.

Isaac Sim's bridge publishes raw only, hence `raw` stays the default and the
robot launch opts in.
"""

from typing import Callable, Optional

from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

RAW = "raw"
COMPRESSED = "compressed"
TRANSPORTS = (RAW, COMPRESSED)

# Cameras publish BEST_EFFORT (RealSense does; Isaac's ROS 2 bridge publishes
# RELIABLE). A BEST_EFFORT subscriber matches both, while a RELIABLE one
# receives nothing at all from a BEST_EFFORT publisher — the only symptom is a
# one-line incompatible-QoS warning at discovery.
# depth=1: consumers here are slower than the camera, and a deeper queue would
# only hand them stale frames — exactly the latency this module exists to cut.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def normalize_transport(value) -> str:
    """Validate an `rgb_transport` parameter value."""
    transport = str(value or RAW).strip().lower()
    if transport not in TRANSPORTS:
        raise ValueError(
            f"rgb_transport must be one of {TRANSPORTS}, got {value!r}"
        )
    return transport


class RgbSource:
    """Subscribes to a colour stream and hands back RGB numpy frames.

    Same interface for both transports: the caller stores whatever message
    arrives and calls `to_rgb()` on it when it actually needs pixels — so a
    consumer that samples at 1 Hz never pays to decode the frames it skips.
    """

    def __init__(
        self,
        node,
        topic: str,
        transport: str = RAW,
        callback: Optional[Callable] = None,
        qos=SENSOR_QOS,
    ):
        self.transport = normalize_transport(transport)
        self.base_topic = topic
        # image_transport's convention; the RealSense driver already
        # publishes it next to the raw topic.
        self.topic = (
            topic if self.transport == RAW else f"{topic.rstrip('/')}/compressed"
        )
        self._bridge = None
        msg_type = Image if self.transport == RAW else CompressedImage
        self.subscription = node.create_subscription(
            msg_type, self.topic, callback, qos
        ) if callback is not None else None

    def to_rgb(self, msg, reduction: int = 1):
        """Decode a received message to an RGB (H, W, 3) uint8 array.

        `reduction` (1, 2, 4, 8) decodes a JPEG straight to 1/N scale, which
        libjpeg does during decoding — several times cheaper than decoding
        full size and resizing afterwards. Use it for views (the HUD); keep 1
        wherever the pixels feed a model.
        """
        if self.transport == COMPRESSED:
            import cv2
            import numpy as np

            flags = {
                1: cv2.IMREAD_COLOR,
                2: cv2.IMREAD_REDUCED_COLOR_2,
                4: cv2.IMREAD_REDUCED_COLOR_4,
                8: cv2.IMREAD_REDUCED_COLOR_8,
            }.get(int(reduction), cv2.IMREAD_COLOR)
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(buf, flags)
            if bgr is None:
                raise ValueError(
                    f"could not decode {getattr(msg, 'format', '?')} frame "
                    f"on {self.topic}"
                )
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        if self._bridge is None:
            from cv_bridge import CvBridge
            self._bridge = CvBridge()
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
