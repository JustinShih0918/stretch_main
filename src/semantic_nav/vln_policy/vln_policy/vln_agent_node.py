#!/usr/bin/env python3
"""VLN agent with shared remote inference and trajectory execution."""

import math
import threading
import uuid
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from btcpp_ros2_interfaces.msg import (
    TrajectoryPoint as TrajectoryPointMsg,
    VlnInferenceStep,
    VlnStatus,
)
from btcpp_ros2_interfaces.srv import PrepareVlnEpisode
from nav2_msgs.action import NavigateToPose

from .action_executor import (
    CmdVelExecutor,
    ExecStatus,
    Nav2WaypointExecutor,
    TrajectoryFollowerExecutor,
)
from .backends import make_backend
from .backends.base import (
    FORWARD_M,
    STOP,
    BackendError,
    CameraIntrinsics,
    OdomPose,
    StepResult,
)
from .image_rotation import (
    normalize_rgb_rotation,
    rotate_camera_intrinsics,
    rotate_depth_image,
    rotate_rgb_image,
)
from .motion_intent import parse_robot_relative_command

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


def _copy_pose(pose: Optional[OdomPose]) -> OdomPose:
    return OdomPose(pose.x, pose.y, pose.yaw) if pose else OdomPose()


class VlnAgentNode(Node):
    def __init__(self, **node_kwargs):
        # node_kwargs lets tests inject parameter_overrides without touching
        # the global rclpy context.
        super().__init__("vln_agent_node", **node_kwargs)
        self._declare_parameters()

        self.backend_name = str(self.get_parameter("backend").value)
        self.execution_mode = str(self.get_parameter("execution_mode").value)
        if self.execution_mode not in ("cmd_vel", "nav2", "trajectory"):
            raise ValueError(
                "execution_mode must be 'cmd_vel', 'nav2', or 'trajectory'"
            )
        self.goal_frame = str(self.get_parameter("goal_frame").value)
        self.max_steps = int(self.get_parameter("max_steps").value)
        self.max_model_steps = int(
            self.get_parameter("max_model_steps").value
        )
        self.rgb_rotation = normalize_rgb_rotation(
            self.get_parameter("rgb_rotation").value
        )
        self.replan_period_s = float(
            self.get_parameter("dualvln_replan_period_s").value
        )

        self.backend = self._make_backend()
        self.executor_impl = None

        self.state = IDLE
        self.instruction = ""
        self.step_count = 0
        self.model_step_count = 0
        self.detail = ""
        self.terminal_reason = ""
        self.episode_id = ""
        self._lock = threading.Lock()
        self._pending = None
        self._worker = None
        self._episode_seq = 0
        self._inference_active = False
        self._reset_inflight = False
        self._start_after_reset = True
        self._prepared_instruction = None
        self._next_dual_request_s = 0.0
        self._stop_after_motion = False

        self._latest_rgb = None
        self._latest_synced = None
        self._camera_intrinsics = None
        self._odom = None
        self._odom_time = None
        self._bridge = None
        self._nav2_client = None
        self._nav2_goal_handle = None

        self.status_pub = self.create_publisher(VlnStatus, "/vln/status", 1)
        self.step_pub = self.create_publisher(
            VlnInferenceStep, "/vln/inference_step", 10
        )
        self.action_pub = self.create_publisher(
            String, "/vln/current_action", 1
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.executor_impl = self._make_executor()
        self._create_sensor_subscriptions()
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._on_odom, 10,
        )
        self.create_subscription(String, "~/instruction", self._on_instruction, 1)
        self.create_service(
            PrepareVlnEpisode, "~/prepare_episode", self._on_prepare_episode
        )
        self.create_service(Trigger, "~/cancel", self._on_cancel)

        tick_hz = float(self.get_parameter("tick_rate_hz").value)
        self.create_timer(1.0 / tick_hz, self._tick)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            f"vln_agent_node up: backend={self.backend_name} "
            f"mode={self.execution_mode} rgb_rotation={self.rgb_rotation}"
        )

    def _declare_parameters(self):
        declarations = (
            ("backend", "streamvln"),
            ("execution_mode", "cmd_vel"),
            ("server_url", "http://localhost:18080"),
            ("timeout_s", 30.0),
            ("jpeg_quality", 85),
            ("rgb_topic", "/rgb"),
            ("depth_topic", "/depth"),
            ("camera_info_topic", "/camera_info"),
            ("sync_slop_s", 0.08),
            ("rgb_rotation", "clockwise_90"),
            ("odom_topic", "/odom"),
            ("cmd_vel_topic", "/cmd_vel"),
            ("goal_frame", "odom"),
            ("max_steps", 150),
            ("max_model_steps", 150),
            ("v_lin", 0.25),
            ("v_ang", 0.5),
            ("action_timeout_s", 6.0),
            ("nav2_goal_timeout_s", 60.0),
            ("trajectory_lookahead_m", 0.35),
            ("trajectory_final_tolerance_m", 0.12),
            ("trajectory_turn_tolerance_deg", 5.0),
            ("trajectory_linear_accel_mps2", 0.5),
            ("trajectory_angular_accel_rps2", 1.0),
            ("trajectory_watchdog_s", 6.0),
            ("trajectory_odom_timeout_s", 1.0),
            ("dualvln_replan_period_s", 0.3),
            ("dummy_actions", ""),
            ("tick_rate_hz", 20.0),
        )
        for name, default in declarations:
            self.declare_parameter(name, default)

    def _make_backend(self):
        kwargs = {}
        if self.backend_name in ("streamvln", "dualvln", "navila"):
            kwargs = {
                "server_url": str(self.get_parameter("server_url").value),
                "timeout_s": float(self.get_parameter("timeout_s").value),
                "jpeg_quality": int(
                    self.get_parameter("jpeg_quality").value
                ),
            }
        elif self.backend_name == "dummy":
            script = str(self.get_parameter("dummy_actions").value).strip()
            if script:
                kwargs = {"actions_csv": script}
        return make_backend(self.backend_name, **kwargs)

    def _make_executor(self):
        if self.execution_mode == "cmd_vel":
            return CmdVelExecutor(
                self._publish_twist,
                v_lin=float(self.get_parameter("v_lin").value),
                v_ang=float(self.get_parameter("v_ang").value),
                action_timeout_s=float(
                    self.get_parameter("action_timeout_s").value
                ),
            )
        if self.execution_mode == "nav2":
            return Nav2WaypointExecutor(
                self._send_nav2_goal,
                goal_timeout_s=float(
                    self.get_parameter("nav2_goal_timeout_s").value
                ),
            )
        return TrajectoryFollowerExecutor(
            self._publish_twist,
            v_lin=float(self.get_parameter("v_lin").value),
            v_ang=float(self.get_parameter("v_ang").value),
            lookahead_m=float(
                self.get_parameter("trajectory_lookahead_m").value
            ),
            final_tolerance_m=float(
                self.get_parameter("trajectory_final_tolerance_m").value
            ),
            turn_tolerance_rad=math.radians(float(
                self.get_parameter("trajectory_turn_tolerance_deg").value
            )),
            linear_accel_mps2=float(
                self.get_parameter("trajectory_linear_accel_mps2").value
            ),
            angular_accel_rps2=float(
                self.get_parameter("trajectory_angular_accel_rps2").value
            ),
            watchdog_s=float(
                self.get_parameter("trajectory_watchdog_s").value
            ),
            odom_timeout_s=float(
                self.get_parameter("trajectory_odom_timeout_s").value
            ),
            control_rate_hz=float(self.get_parameter("tick_rate_hz").value),
        )

    def _create_sensor_subscriptions(self):
        rgb_topic = str(self.get_parameter("rgb_topic").value)
        if self.backend.requires_depth:
            from message_filters import ApproximateTimeSynchronizer, Subscriber

            self._rgb_filter = Subscriber(self, Image, rgb_topic)
            self._depth_filter = Subscriber(
                self, Image, str(self.get_parameter("depth_topic").value)
            )
            self._sensor_sync = ApproximateTimeSynchronizer(
                [self._rgb_filter, self._depth_filter],
                queue_size=5,
                slop=float(self.get_parameter("sync_slop_s").value),
            )
            self._sensor_sync.registerCallback(self._on_rgb_depth)
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter("camera_info_topic").value),
                self._on_camera_info,
                1,
            )
        else:
            self.create_subscription(Image, rgb_topic, self._on_rgb, 1)

    def _publish_twist(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        try:
            self.cmd_vel_pub.publish(msg)
        except RuntimeError:
            # SIGINT may invalidate the rcl context before ``finally`` asks
            # the executor to zero one last time.
            pass

    def _on_rgb(self, msg: Image):
        self._latest_rgb = msg

    def _on_rgb_depth(self, rgb_msg: Image, depth_msg: Image):
        self._latest_synced = (rgb_msg, depth_msg)
        self._latest_rgb = rgb_msg

    def _on_camera_info(self, msg: CameraInfo):
        self._camera_intrinsics = CameraIntrinsics(
            fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5],
            width=msg.width, height=msg.height,
        )

    def _on_odom(self, msg: Odometry):
        pose = msg.pose.pose
        self._odom = OdomPose(
            x=pose.position.x,
            y=pose.position.y,
            yaw=_yaw_from_quat(pose.orientation),
        )
        self._odom_time = self._now_s()
        if isinstance(self.executor_impl, TrajectoryFollowerExecutor):
            self.executor_impl.notify_odom(self._odom_time)

    def _on_prepare_episode(self, request, response):
        instruction = request.instruction.strip()
        if not instruction:
            response.accepted = False
            response.message = "instruction is empty"
            return response
        self._begin_model_episode(instruction, start_after_reset=False)
        response.accepted = True
        response.episode_id = self.episode_id
        response.message = "remote reset started"
        return response

    def _on_cancel(self, request, response):
        self._episode_seq += 1
        self._inference_active = False
        self._reset_inflight = False
        self._prepared_instruction = None
        self._cancel_motion()
        self._finish(DONE, "cancelled", "episode cancelled")
        response.success = True
        response.message = "episode cancelled and velocity zeroed"
        return response

    def _on_instruction(self, msg: String):
        instruction = msg.data.strip()
        if not instruction:
            return
        self.get_logger().info(f"[VLN] new instruction: '{instruction}'")
        if (
            self._prepared_instruction == instruction
            and self.state == IDLE
            and not self._reset_inflight
        ):
            self._prepared_instruction = None
            self._set_state(THINKING, "")
            self._request_step(force=True)
            return

        direct_actions = parse_robot_relative_command(instruction)
        if direct_actions is not None:
            self._begin_local_episode(instruction, direct_actions)
            return
        self._begin_model_episode(instruction, start_after_reset=True)

    def _reset_episode_fields(self, instruction: str):
        self._episode_seq += 1
        self._inference_active = False
        self._reset_inflight = False
        self.episode_id = uuid.uuid4().hex
        self.instruction = instruction
        self.step_count = 0
        self.model_step_count = 0
        self.detail = ""
        self.terminal_reason = ""
        self._prepared_instruction = None
        self._next_dual_request_s = 0.0
        self._stop_after_motion = False
        self._cancel_motion()
        with self._lock:
            self._pending = None

    def _begin_model_episode(self, instruction: str, start_after_reset: bool):
        self._reset_episode_fields(instruction)
        self._start_after_reset = start_after_reset
        self._reset_inflight = True
        self._set_state(RESETTING)
        self._spawn_worker("reset", instruction=instruction)

    def _begin_local_episode(self, instruction: str, direct_actions):
        self._reset_episode_fields(instruction)
        requested = len(direct_actions)
        actions = direct_actions[:self.max_steps]
        self.step_count = len(actions)
        distance_m = len(actions) * FORWARD_M
        self.detail = f"local robot-relative reverse command: {distance_m:.2f} m"
        if len(actions) < requested:
            self.detail += f" (limited by max_steps={self.max_steps})"
        self._submit_actions(actions)
        if self.executor_impl.status is ExecStatus.ERROR:
            self._finish(ERROR, "executor_error", self.executor_impl.error)
        else:
            self.terminal_reason = "local_command_complete"
            self._set_state(EXECUTING)

    def _spawn_worker(self, kind: str, **kwargs):
        if self._inference_active:
            return
        seq = self._episode_seq
        self._inference_active = True

        def run():
            result = {"kind": kind, "seq": seq}
            try:
                if kind == "reset":
                    self.backend.reset(kwargs["instruction"])
                else:
                    result["step"] = self.backend.step(
                        kwargs["rgb"], kwargs["odom"],
                        depth=kwargs.get("depth"),
                        depth_scale_m=kwargs.get("depth_scale_m"),
                        intrinsics=kwargs.get("intrinsics"),
                        image_timestamp_s=kwargs.get("image_timestamp_s"),
                    )
                    result["request_pose"] = kwargs["odom"]
                    result["request_timestamp_s"] = kwargs[
                        "request_timestamp_s"
                    ]
            except BackendError as exc:
                result["error"] = str(exc)
            except Exception as exc:  # noqa: BLE001
                result["error"] = f"{type(exc).__name__}: {exc}"
            with self._lock:
                if result["seq"] == self._episode_seq:
                    self._pending = result

        self._worker = threading.Thread(target=run, daemon=True)
        self._worker.start()

    def _take_pending(self):
        with self._lock:
            result, self._pending = self._pending, None
        return result

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_state(self, state: str, detail: str = None):
        if detail is not None:
            self.detail = detail
        if state != self.state:
            self.state = state
            self._publish_status()

    def _finish(self, state: str, reason: str, detail: str):
        # Invalidate a late HTTP response before clearing the active flag.
        self._episode_seq += 1
        self.terminal_reason = reason
        self._inference_active = False
        self._reset_inflight = False
        self._cancel_motion()
        self._set_state(state, detail)

    def _cancel_motion(self):
        if self.executor_impl is not None:
            self.executor_impl.cancel()
        self._publish_twist(0.0, 0.0)
        if self._nav2_goal_handle is not None:
            try:
                self._nav2_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            self._nav2_goal_handle = None

    def _tick(self):
        pending = self._take_pending()
        if pending is not None:
            self._inference_active = False
            if "error" in pending:
                reason = (
                    "backend_error" if pending["kind"] == "step"
                    else "reset_error"
                )
                self.get_logger().error(f"[VLN] {reason}: {pending['error']}")
                self._finish(ERROR, reason, pending["error"])
                return
            if pending["kind"] == "reset":
                self._reset_inflight = False
                if self._start_after_reset:
                    self._set_state(THINKING, "")
                    self._request_step(force=True)
                else:
                    self._prepared_instruction = self.instruction
                    self._set_state(IDLE, "episode prepared")
                return
            self._handle_step_result(
                pending["step"], pending.get("request_pose"),
                pending.get("request_timestamp_s"),
            )

        if self.state in (IDLE, RESETTING, DONE, ERROR):
            return

        if self.executor_impl.status is ExecStatus.RUNNING:
            if isinstance(self.executor_impl, TrajectoryFollowerExecutor):
                status = self.executor_impl.tick(
                    self._now_s(), self._odom, self._odom_time
                )
            else:
                status = self.executor_impl.tick(self._now_s(), self._odom)
            if status is ExecStatus.ERROR:
                self._finish(
                    ERROR, "executor_error", self.executor_impl.error
                )
                return

        if (
            self.terminal_reason == "local_command_complete"
            and self.executor_impl.status is ExecStatus.DONE
        ):
            self._finish(
                DONE, "local_command_complete",
                "robot-relative reverse command complete",
            )
            return

        if (
            self._stop_after_motion
            and self.executor_impl.status is ExecStatus.DONE
        ):
            self._finish(
                DONE, "model_stop", self.detail or "model returned STOP"
            )
            return

        if self.model_step_count >= self.max_model_steps:
            self._finish(DONE, "max_model_steps", "max_model_steps reached")
            return

        if self.backend_name == "dualvln":
            if (
                not self._inference_active
                and self._now_s() >= self._next_dual_request_s
            ):
                self._request_step()
        elif (
            not self._inference_active
            and self.executor_impl.status in (ExecStatus.DONE, ExecStatus.IDLE)
        ):
            self._request_step()

        if self.executor_impl.status is ExecStatus.RUNNING:
            self._set_state(EXECUTING)
        elif self._inference_active:
            self._set_state(THINKING)

    def _request_step(self, force: bool = False):
        if self._inference_active:
            return
        if self.backend_name == "dualvln" and not force:
            if self._now_s() < self._next_dual_request_s:
                return
        try:
            observation = self._latest_observation()
        except BackendError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return
        if observation is None:
            return
        self._next_dual_request_s = self._now_s() + self.replan_period_s
        self._spawn_worker(
            "step", odom=_copy_pose(self._odom),
            request_timestamp_s=self._now_s(), **observation,
        )
        if self.executor_impl.status is not ExecStatus.RUNNING:
            self._set_state(THINKING)

    def _latest_observation(self):
        if not self.backend.requires_rgb:
            return {
                "rgb": None, "depth": None, "depth_scale_m": None,
                "intrinsics": None, "image_timestamp_s": self._now_s(),
            }
        if self.backend.requires_depth:
            if self._latest_synced is None:
                raise BackendError("waiting for synchronized RGB/depth frames")
            if self._camera_intrinsics is None:
                raise BackendError("waiting for camera calibration")
            rgb_msg, depth_msg = self._latest_synced
            rgb = self._rgb_to_numpy(rgb_msg)
            depth, scale = self._depth_to_uint16(depth_msg)
            depth = rotate_depth_image(depth, self.rgb_rotation)
            intrinsics = rotate_camera_intrinsics(
                self._camera_intrinsics, self.rgb_rotation
            )
            return {
                "rgb": rgb, "depth": depth, "depth_scale_m": scale,
                "intrinsics": intrinsics,
                "image_timestamp_s": self._stamp_s(rgb_msg.header.stamp),
            }
        if self._latest_rgb is None:
            raise BackendError(
                f"waiting for image on {self.get_parameter('rgb_topic').value}"
            )
        return {
            "rgb": self._rgb_to_numpy(self._latest_rgb),
            "depth": None,
            "depth_scale_m": None,
            "intrinsics": None,
            "image_timestamp_s": self._stamp_s(self._latest_rgb.header.stamp),
        }

    @staticmethod
    def _stamp_s(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _rgb_to_numpy(self, msg: Image):
        if self._bridge is None:
            from cv_bridge import CvBridge
            self._bridge = CvBridge()
        rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        return rotate_rgb_image(rgb, self.rgb_rotation)

    def _depth_to_uint16(self, msg: Image):
        import numpy as np

        if self._bridge is None:
            from cv_bridge import CvBridge
            self._bridge = CvBridge()
        depth = np.asarray(
            self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        )
        encoding = msg.encoding.upper()
        if encoding in ("16UC1", "MONO16"):
            return depth.astype(np.uint16, copy=False), 0.001
        if encoding == "32FC1":
            clean = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            scale = 0.001
            return np.clip(clean / scale, 0, 65535).astype(np.uint16), scale
        raise BackendError(
            f"unsupported depth encoding '{msg.encoding}' "
            "(expected 16UC1 or 32FC1)"
        )

    def _handle_step_result(
        self,
        step: StepResult,
        request_pose: Optional[OdomPose],
        request_timestamp_s: Optional[float],
    ):
        self.model_step_count += 1
        self._publish_inference_step(
            step, request_pose, request_timestamp_s
        )
        if step.detail:
            self.detail = step.detail

        stop_requested = STOP in step.actions or step.done
        actions = list(step.actions)
        if STOP in actions:
            actions = actions[:actions.index(STOP)]
        if stop_requested and not actions and not step.trajectory:
            # A bare STOP cancels a DualVLN path immediately.
            self._finish(
                DONE, "model_stop", self.detail or "model returned STOP"
            )
            return

        if step.trajectory:
            if self.execution_mode != "trajectory":
                self._finish(
                    ERROR, "backend_error",
                    "trajectory response requires execution_mode:=trajectory",
                )
                return
            self.executor_impl.replace_trajectory(
                step.trajectory, self._odom, self._now_s()
            )
            count = len(step.trajectory)
            self.get_logger().info(
                f"[VLN] model step {self.model_step_count}: "
                f"trajectory with {count} points"
            )
        else:
            remaining = max(0, self.max_steps - self.step_count)
            actions = actions[:remaining]
            if not actions:
                self._finish(DONE, "max_steps", "max_steps reached")
                return
            self.step_count += len(actions)
            self._submit_actions(actions)
            self.get_logger().info(
                f"[VLN] model step {self.model_step_count}: {' '.join(actions)}"
            )

        # Preserve every action before a batch's STOP, then zero immediately
        # when that action-derived path completes.
        self._stop_after_motion = stop_requested

        if self.executor_impl.status is ExecStatus.ERROR:
            self._finish(ERROR, "executor_error", self.executor_impl.error)
            return
        self._set_state(EXECUTING)

    def _submit_actions(self, actions):
        if self.execution_mode == "cmd_vel":
            self.executor_impl.submit(actions)
        elif self.execution_mode == "nav2":
            self.executor_impl.submit(actions, self._odom, self._now_s())
        else:
            self.executor_impl.submit_actions(
                actions, self._odom, self._now_s()
            )

    def _publish_inference_step(
        self, step, request_pose, request_timestamp_s
    ):
        msg = VlnInferenceStep()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        image_stamp_s = step.image_timestamp_s or 0.0
        msg.image_stamp.sec = int(image_stamp_s)
        msg.image_stamp.nanosec = int((image_stamp_s % 1.0) * 1e9)
        request_timestamp_s = request_timestamp_s or 0.0
        msg.request_stamp.sec = int(request_timestamp_s)
        msg.request_stamp.nanosec = int(
            (request_timestamp_s % 1.0) * 1e9
        )
        msg.episode_id = self.episode_id
        msg.model_step = self.model_step_count
        msg.backend = self.backend_name
        msg.output_type = step.output_type
        msg.output_size = len(step.trajectory or step.actions)
        msg.done = bool(step.done or STOP in step.actions)
        request_pose = request_pose or OdomPose()
        response_pose = self._odom or OdomPose()
        for target, pose in (
            (msg.request_pose, request_pose), (msg.response_pose, response_pose)
        ):
            target.x, target.y, target.theta = pose.x, pose.y, pose.yaw
        timings = step.timings
        msg.client_latency_ms = self._timing_value(timings.client_ms)
        msg.server_total_ms = self._timing_value(timings.total_ms)
        msg.preprocessing_ms = self._timing_value(timings.preprocessing_ms)
        msg.system1_ms = self._timing_value(timings.system1_ms)
        msg.system2_ms = self._timing_value(timings.system2_ms)
        msg.actions = list(step.actions)
        msg.trajectory = [
            TrajectoryPointMsg(x=float(point.x), y=float(point.y))
            for point in step.trajectory
        ]
        self.step_pub.publish(msg)

    @staticmethod
    def _timing_value(value):
        return float(value) if value is not None else float("nan")

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
        msg.episode_id = self.episode_id
        msg.model_step_count = self.model_step_count
        msg.inference_active = self._inference_active
        msg.motion_active = self.executor_impl.status is ExecStatus.RUNNING
        msg.terminal_reason = self.terminal_reason
        msg.controller_tick_count = getattr(
            self.executor_impl, "controller_tick_count", 0
        )
        self.status_pub.publish(msg)
        action = String()
        action.data = msg.current_action or self.state
        self.action_pub.publish(action)

    def destroy_node(self):
        self._cancel_motion()
        return super().destroy_node()


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
