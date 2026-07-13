#!/usr/bin/env python3
"""Milestone-1 helper: publish a hand-authored traversable SemanticRegion.

Lets you validate the SemanticTraversabilityLayer + planner end-to-end WITHOUT
the VLM/projection pipeline -- just declare a polygon over the known curtain
location and watch the costmap clear and the planner route through it.

Params:
  frame_id   (str)   frame the polygon is expressed in (default: world)
  label      (str)   region label (default: curtain)
  cost       (float) informational; the layer's traversable_cost param wins
  polygon    (float[]) flat [x0,y0,x1,y1,...] ground polygon (>=3 points)
  period_sec (float) republish period
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point32
from btcpp_ros2_interfaces.msg import SemanticRegion, SemanticRegionArray


class StaticRegionPublisher(Node):
    def __init__(self):
        super().__init__("static_region_publisher")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("label", "curtain")
        self.declare_parameter("cost", 0.0)
        # default: a 1m x 0.4m strip around (2.0, 0.0)
        self.declare_parameter(
            "polygon", [1.5, -0.2, 2.5, -0.2, 2.5, 0.2, 1.5, 0.2]
        )
        self.declare_parameter("period_sec", 1.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.label = self.get_parameter("label").value
        self.cost = float(self.get_parameter("cost").value)
        flat = list(self.get_parameter("polygon").value)
        self.points = [
            (float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat) - 1, 2)
        ]

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(SemanticRegionArray, "/semantic_regions", qos)
        self.timer = self.create_timer(
            float(self.get_parameter("period_sec").value), self._tick
        )
        self.get_logger().info(
            f"static_region_publisher: '{self.label}' {len(self.points)} pts in "
            f"'{self.frame_id}'"
        )

    def _tick(self):
        region = SemanticRegion()
        region.label = self.label
        region.traversable = True
        region.cost = self.cost
        region.polygon.header.stamp = self.get_clock().now().to_msg()
        region.polygon.header.frame_id = self.frame_id
        for x, y in self.points:
            p = Point32()
            p.x, p.y, p.z = x, y, 0.0
            region.polygon.polygon.points.append(p)

        out = SemanticRegionArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.regions.append(region)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = StaticRegionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
