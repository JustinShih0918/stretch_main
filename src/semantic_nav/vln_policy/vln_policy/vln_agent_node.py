#!/usr/bin/env python3
"""VLN agent node: instruction in, discrete VLN actions out, robot moving.

Pipeline (standalone — no BT engine required):

  /vln_instruction (String)
      -> backend.reset()                       [RESETTING]
      -> loop: backend.step(latest /rgb frame) [THINKING]
               -> executor runs the batch      [EXECUTING]
      -> DONE on model STOP / max_steps; ERROR degrades gracefully.

Live command output: btcpp_ros2_interfaces/VlnStatus on /vln/status plus the
bare action token on /vln/current_action. Motion goes out either as /cmd_vel
bursts (execution_mode:=cmd_vel) or as one relative waypoint per batch to
nav2 navigate_to_pose (execution_mode:=nav2).
"""

import math
import threading
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from btcpp_ros2_interfaces.msg import VlnStatus
from nav2_msgs.action import NavigateToPose

from .action_executor import (
    CmdVelExecutor,
    ExecStatus,
    Nav2WaypointExecutor,
)
from .backends import make_backend
from .backends.base import (
    STOP,
    BackendError,
    OdomPose,
    StepResult,
)

# node states (mirrors VlnStatus.msg STATE_* constants)
IDLE = "IDLE"
RESETTING = "RESETTING"
THINKING = "THINKING"
EXECUTING = "EXECUTING"
DONE = "DONE"
ERROR = "ERROR"


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class VlnAgentNode(Node):
    def __init__(self):
        super().__init__("vln_agent_node")

        self.declare_parameter("backend", "streamvln")
        self.declare_parameter("execution_mode", "cmd_vel")
        self.declare_parameter("server_url", "http://localhost:18080")
        self.declare_parameter("timeout_s", 30.0)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("rgb_topic", "/rgb")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_frame", "odom")
        self.declare_parameter("max_steps", 150)
        self.declare_parameter("v_lin", 0.25)
        self.declare_parameter("v_ang", 0.5)
        self.declare_parameter("action_timeout_s", 6.0)
        self.declare_parameter("nav2_goal_timeout_s", 60.0)
        self.declare_parameter("dummy_actions", "")
        self.declare_parameter("tick_rate_hz", 20.0)

        self.backend_name = str(self.get_parameter("backend").value)
        self.execution_mode = str(self.get_parameter("execution_mode").value)
        if self.execution_mode not in ("cmd_vel", "nav2"):
            raise ValueError("execution_mode must be 'cmd_vel' or 'nav2'")
        self.goal_frame = str(self.get_parameter("goal_frame").value)
        self.max_steps = int(self.get_parameter("max_steps").value)

        self.backend = self._make_backend()
        self.executor_impl = self._make_executor()

        # episode state (mutated only from ROS callbacks/timers; worker
        # threads hand results over via _pending under _lock)
        self.state = IDLE
        self.instruction = ""
        self.step_count = 0
        self.detail = ""
        self.episode_done_pending = False
        self._lock = threading.Lock()
        self._pending: Optional[dict] = None
        self._worker: Optional[threading.Thread] = None
        self._episode_seq = 0

        self._latest_rgb: Optional[Image] = None
        self._odom: Optional[OdomPose] = None
        self._bridge = None
        self._nav2_client = None
        self._nav2_goal_handle = None

        self.status_pub = self.create_publisher(VlnStatus, "/vln/status", 10)
        self.action_pub = self.create_publisher(
            String, "/vln/current_action", 10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.create_subscription(
            Image, str(self.get_parameter("rgb_topic").value),
            self._on_rgb, 1,
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._on_odom, 10,
        )
        self.create_subscription(
            String, "~/instruction", self._on_instruction, 1
        )

        tick_hz = float(self.get_parameter("tick_rate_hz").value)
        self.create_timer(1.0 / tick_hz, self._tick)
        self.create_timer(0.5, self._publish_status)  # heartbeat

        self.get_logger().info(
            f"vln_agent_node up: backend={self.backend_name} "
            f"mode={self.execution_mode} — waiting for instruction on "
            "~/instruction"
        )

    # -- construction helpers --------------------------------------------
    def _make_backend(self):
        kwargs = {}
        if self.backend_name in ("streamvln", "navila"):
            kwargs = {
                "server_url": str(self.get_parameter("server_url").value),
                "timeout_s": float(self.get_parameter("timeout_s").value),
                "jpeg_quality": int(self.get_parameter("jpeg_quality").value),
            }
        elif self.backend_name == "dummy":
            script = str(self.get_parameter("dummy_actions").value).strip()
            if script:
                kwargs = {"actions_csv": script}
        return make_backend(self.backend_name, **kwargs)

    def _make_executor(self):
        if self.execution_mode == "cmd_vel":
            return CmdVelExecutor(
                publish_twist=self._publish_twist,
                v_lin=float(self.get_parameter("v_lin").value),
                v_ang=float(self.get_parameter("v_ang").value),
                action_timeout_s=float(
                    self.get_parameter("action_timeout_s").value
                ),
            )
        return Nav2WaypointExecutor(
            send_goal=self._send_nav2_goal,
            goal_timeout_s=float(
                self.get_parameter("nav2_goal_timeout_s").value
            ),
        )

    # -- ROS I/O ----------------------------------------------------------
    def _publish_twist(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def _on_rgb(self, msg: Image):
        self._latest_rgb = msg

    def _on_odom(self, msg: Odometry):
        pose = msg.pose.pose
        self._odom = OdomPose(
            x=pose.position.x,
            y=pose.position.y,
            yaw=_yaw_from_quat(pose.orientation),
        )

    def _on_instruction(self, msg: String):
        instruction = msg.data.strip()
        if not instruction:
            return
        self.get_logger().info(f"[VLN] new instruction: '{instruction}'")
        self._cancel_motion()
        self.instruction = instruction
        self.step_count = 0
        self.detail = ""
        self.episode_done_pending = False
        self._episode_seq += 1
        self._set_state(RESETTING)
        self._spawn_worker("reset", instruction=instruction)

    # -- worker thread bridge ---------------------------------------------
    def _spawn_worker(self, kind: str, **kwargs):
        seq = self._episode_seq

        def run():
            result = {"kind": kind, "seq": seq}
            try:
                if kind == "reset":
                    self.backend.reset(kwargs["instruction"])
                else:
                    result["step"] = self.backend.step(
                        kwargs["rgb"], kwargs["odom"]
                    )
            except BackendError as exc:
                result["error"] = str(exc)
            except Exception as exc:  # noqa: BLE001 — surface, don't die
                result["error"] = f"{type(exc).__name__}: {exc}"
            with self._lock:
                if result["seq"] == self._episode_seq:
                    self._pending = result
                # else: stale thread from a cancelled episode — drop it

        self._worker = threading.Thread(target=run, daemon=True)
        self._worker.start()

    def _take_pending(self) -> Optional[dict]:
        with self._lock:
            result, self._pending = self._pending, None
        return result

    # -- state machine ----------------------------------------------------
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_state(self, state: str, detail: str = None):
        if detail is not None:
            self.detail = detail
        if state != self.state:
            self.state = state
            self._publish_status()

    def _cancel_motion(self):
        self.executor_impl.cancel()
        if self._nav2_goal_handle is not None:
            try:
                self._nav2_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001 — handle may already be done
                pass
            self._nav2_goal_handle = None

    def _tick(self):
        if self.state == RESETTING:
            result = self._take_pending()
            if result is None:
                return
            if "error" in result:
                self.get_logger().error(f"[VLN] reset failed: "
                                        f"{result['error']}")
                self._set_state(ERROR, result["error"])
                return
            self._set_state(THINKING, "")
            self._request_step()

        elif self.state == THINKING:
            result = self._take_pending()
            if result is None:
                if self._worker is None or not self._worker.is_alive():
                    self._request_step()  # frame wasn't ready; retry
                return
            if "error" in result:
                self.get_logger().error(f"[VLN] step failed: "
                                        f"{result['error']}")
                self._set_state(ERROR, result["error"])
                return
            self._handle_step_result(result["step"])

        elif self.state == EXECUTING:
            status = self.executor_impl.tick(self._now_s(), self._odom)
            if status is ExecStatus.ERROR:
                self.get_logger().error(
                    f"[VLN] executor error: {self.executor_impl.error}"
                )
                self._cancel_motion()
                self._set_state(ERROR, self.executor_impl.error)
            elif status is ExecStatus.DONE:
                if self.episode_done_pending:
                    self.get_logger().info(
                        f"[VLN] episode DONE after {self.step_count} actions"
                    )
                    self._set_state(DONE, "model returned STOP")
                elif self.step_count >= self.max_steps:
                    self.get_logger().warn(
                        f"[VLN] max_steps={self.max_steps} reached, stopping"
                    )
                    self._set_state(DONE, "max_steps reached")
                else:
                    self._set_state(THINKING, "")
                    self._request_step()

    def _request_step(self):
        if self._worker is not None and self._worker.is_alive():
            return
        rgb = None
        if self.backend.requires_rgb:
            if self._latest_rgb is None:
                self.get_logger().warn(
                    f"[VLN] waiting for image on "
                    f"{self.get_parameter('rgb_topic').value}",
                    throttle_duration_sec=5.0,
                )
                return
            rgb = self._rgb_to_numpy(self._latest_rgb)
        self._spawn_worker("step", rgb=rgb, odom=self._odom)

    def _rgb_to_numpy(self, msg: Image):
        if self._bridge is None:
            from cv_bridge import CvBridge
            self._bridge = CvBridge()
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

    def _handle_step_result(self, step: StepResult):
        actions = list(step.actions)
        if STOP in actions:
            actions = actions[:actions.index(STOP)]
            self.episode_done_pending = True
        if step.done:
            self.episode_done_pending = True
        if step.detail:
            self.detail = step.detail

        remaining = self.max_steps - self.step_count
        actions = actions[:max(0, remaining)]

        if not actions:
            self.get_logger().info(
                f"[VLN] episode DONE after {self.step_count} actions"
            )
            self._set_state(DONE, self.detail or "model returned STOP")
            return

        self.step_count += len(actions)
        self.get_logger().info(
            f"[VLN] step {self.step_count}: {' '.join(actions)}"
            + (" (final batch)" if self.episode_done_pending else "")
        )
        if self.execution_mode == "cmd_vel":
            self.executor_impl.submit(actions)
        else:
            self.executor_impl.submit(actions, self._odom, self._now_s())
            if self.executor_impl.status is ExecStatus.ERROR:
                self._set_state(ERROR, self.executor_impl.error)
                return
        self._set_state(EXECUTING)

    # -- nav2 goal plumbing ------------------------------------------------
    def _send_nav2_goal(self, x: float, y: float, yaw: float):
        if self._nav2_client is None:
            self._nav2_client = ActionClient(
                self, NavigateToPose, "navigate_to_pose"
            )
        if not self._nav2_client.wait_for_server(timeout_sec=5.0):
            self.executor_impl.notify_result(
                False, "navigate_to_pose server not available"
            )
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.goal_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.get_logger().info(
            f"[VLN] nav2 goal -> ({x:.2f}, {y:.2f}, yaw {math.degrees(yaw):.0f}"
            f" deg) in {self.goal_frame}"
        )
        future = self._nav2_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav2_goal_response)

    def _on_nav2_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.executor_impl.notify_result(False, "nav2 rejected the goal")
            return
        self._nav2_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav2_result)

    def _on_nav2_result(self, future):
        from action_msgs.msg import GoalStatus
        self._nav2_goal_handle = None
        status = future.result().status
        self.executor_impl.notify_result(
            status == GoalStatus.STATUS_SUCCEEDED,
            f"nav2 goal ended with status {status}",
        )

    # -- status ------------------------------------------------------------
    def _publish_status(self):
        msg = VlnStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.instruction = self.instruction
        msg.current_action = self.executor_impl.active_action or ""
        msg.pending_actions = self.executor_impl.pending_actions
        msg.backend = self.backend_name
        msg.execution_mode = self.execution_mode
        msg.step_count = self.step_count
        msg.detail = self.detail
        self.status_pub.publish(msg)

        action = String()
        action.data = msg.current_action or self.state
        self.action_pub.publish(action)


def main(args=None):
    rclpy.init(args=args)
    node = VlnAgentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.backend.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
