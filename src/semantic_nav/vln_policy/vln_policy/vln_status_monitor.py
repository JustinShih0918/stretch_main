#!/usr/bin/env python3
"""A compact, latest-only terminal view of ``/vln/status``."""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from btcpp_ros2_interfaces.msg import VlnStatus


def format_status(msg) -> str:
    """Format a VlnStatus-like object without retaining prior samples."""
    instruction = msg.instruction or "(waiting for an instruction)"
    active = msg.current_action or "-"
    pending = " -> ".join(msg.pending_actions) or "-"
    detail = msg.detail or "-"
    return (
        "== /vln/status (latest) ==\n"
        f"state       : {msg.state}\n"
        f"instruction : {instruction}\n"
        f"action      : {active}\n"
        f"pending     : {pending}\n"
        f"step        : {msg.step_count}\n"
        f"backend     : {msg.backend} / {msg.execution_mode}\n"
        f"detail      : {detail}\n"
        "\nThis pane refreshes in place; old heartbeat samples are discarded.\n"
    )


class VlnStatusMonitor(Node):
    def __init__(self):
        super().__init__("vln_status_monitor")
        self.create_subscription(VlnStatus, "/vln/status", self._on_status, 1)
        self._write("== /vln/status (latest) ==\nwaiting for publisher ...\n")

    @staticmethod
    def _write(text: str):
        # ANSI home + clear gives tmux one stable status page instead of an
        # indefinitely growing ros2-topic-echo transcript.
        sys.stdout.write("\033[H\033[2J" + text)
        sys.stdout.flush()

    def _on_status(self, msg: VlnStatus):
        self._write(format_status(msg))


def main(args=None):
    rclpy.init(args=args)
    node = VlnStatusMonitor()
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
