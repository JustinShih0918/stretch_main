#!/usr/bin/env python3
"""Run exactly one reset-confirmed Isaac Sim benchmark trial."""

import json
import math
import os
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from btcpp_ros2_interfaces.msg import VlnInferenceStep, VlnStatus
from btcpp_ros2_interfaces.srv import PrepareVlnEpisode
from simulation_interfaces.msg import Result
from simulation_interfaces.srv import SetEntityState

from .benchmark import (
    PathIntegrator,
    episode_rates,
    load_manifest,
    prompt_hash,
    reset_pose_in_tolerance,
    spl,
)


def _yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _optional(value):
    value = float(value)
    return value if math.isfinite(value) else None


class BenchmarkTrialNode(Node):
    def __init__(self, **node_kwargs):
        # node_kwargs lets tests inject parameter_overrides without touching
        # the global rclpy context.
        super().__init__("vln_benchmark_trial", **node_kwargs)
        for name, default in (
            ("manifest", ""), ("route_id", ""), ("backend", ""),
            ("repetition", 1), ("output_file", ""),
            ("odom_topic", "/odom"), ("rgb_topic", "/rgb"),
            ("cmd_vel_topic", "/cmd_vel"),
            ("set_entity_state_service", "/set_entity_state"),
            ("prepare_service", "/vln_agent_node/prepare_episode"),
            ("cancel_service", "/vln_agent_node/cancel"),
            ("reset_confirm_timeout_s", 10.0),
        ):
            self.declare_parameter(name, default)

        self.manifest_path = str(self.get_parameter("manifest").value)
        self.manifest = load_manifest(self.manifest_path)
        route_id = str(self.get_parameter("route_id").value)
        matches = [
            route for route in self.manifest["routes"]
            if route["route_id"] == route_id
        ]
        if len(matches) != 1:
            raise ValueError(f"route_id '{route_id}' not found exactly once")
        self.route = matches[0]
        self.backend = str(self.get_parameter("backend").value)
        self.repetition = int(self.get_parameter("repetition").value)
        self.output_file = str(self.get_parameter("output_file").value)
        if not self.output_file:
            raise ValueError("output_file is required")

        self.stage = "WAIT_SERVICES"
        self.stage_started = time.monotonic()
        self.done = False
        self.failure = None
        self._future = None
        self._latest_odom = None
        self._latest_status = None
        self._episode_id = ""
        self._confirm_count = 0
        self._recording = False
        self._start_monotonic = None
        self._start_ros_s = None
        self._camera_frames = 0
        self._path = PathIntegrator()
        self._steps = []
        self._controller_start = 0
        self._controller_end = 0
        self._terminal_pending = None

        self.zero_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.instruction_pub = self.create_publisher(
            String, "/vln_instruction", 1
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._on_odom, 50,
        )
        self.create_subscription(
            Image, str(self.get_parameter("rgb_topic").value),
            self._on_camera, 10,
        )
        self.create_subscription(VlnStatus, "/vln/status", self._on_status, 10)
        self.create_subscription(
            VlnInferenceStep, "/vln/inference_step", self._on_step, 50
        )
        self.entity_client = self.create_client(
            SetEntityState,
            str(self.get_parameter("set_entity_state_service").value),
        )
        self.prepare_client = self.create_client(
            PrepareVlnEpisode,
            str(self.get_parameter("prepare_service").value),
        )
        self.cancel_client = self.create_client(
            Trigger, str(self.get_parameter("cancel_service").value)
        )
        self.nav_cancel_client = self.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal"
        )
        self.create_timer(0.05, self._tick)

    def _set_stage(self, stage):
        self.stage = stage
        self.stage_started = time.monotonic()
        self._future = None
        self.get_logger().info(f"benchmark stage: {stage}")

    def _zero(self):
        self.zero_pub.publish(Twist())

    def _on_odom(self, msg):
        self._latest_odom = msg
        if self.stage == "VERIFY_RESET":
            start = self.route["start"]
            pose = msg.pose.pose
            self._confirm_count = (
                self._confirm_count + 1
                if reset_pose_in_tolerance(
                    pose.position.x, pose.position.y,
                    _yaw(pose.orientation), start,
                )
                else 0
            )
        if self._recording:
            pose = msg.pose.pose.position
            self._path.add(pose.x, pose.y)

    def _on_camera(self, msg):
        if self._recording:
            self._camera_frames += 1

    def _on_status(self, msg):
        self._latest_status = msg
        if msg.episode_id == self._episode_id:
            self._controller_end = int(msg.controller_tick_count)
        if (
            self._recording
            and msg.episode_id == self._episode_id
            and msg.state in ("DONE", "ERROR")
        ):
            # The step event is published before status, but they are distinct
            # DDS topics. Give it one short delivery window before freezing
            # the measurement record.
            self._terminal_pending = msg.terminal_reason or msg.state.lower()
            self._set_stage("FINALIZING")

    def _on_step(self, msg):
        if not self._recording or msg.episode_id != self._episode_id:
            return
        self._steps.append({
            "backend": msg.backend,
            "route_id": self.route["route_id"],
            "repetition": self.repetition,
            "episode_id": msg.episode_id,
            "model_step": int(msg.model_step),
            "image_time_s": _seconds(msg.image_stamp),
            "request_time_s": _seconds(msg.request_stamp),
            "response_time_s": _seconds(msg.header.stamp),
            "request_pose": {
                "x": msg.request_pose.x, "y": msg.request_pose.y,
                "yaw": msg.request_pose.theta,
            },
            "response_pose": {
                "x": msg.response_pose.x, "y": msg.response_pose.y,
                "yaw": msg.response_pose.theta,
            },
            "output_type": msg.output_type,
            "output_size": int(msg.output_size),
            "actions": list(msg.actions),
            "trajectory": [[point.x, point.y] for point in msg.trajectory],
            "client_latency_ms": _optional(msg.client_latency_ms),
            "server_total_ms": _optional(msg.server_total_ms),
            "preprocessing_ms": _optional(msg.preprocessing_ms),
            "system1_ms": _optional(msg.system1_ms),
            "system2_ms": _optional(msg.system2_ms),
            "done": bool(msg.done),
        })

    def _tick(self):
        if self.done:
            return
        now = time.monotonic()
        if self.stage == "WAIT_SERVICES":
            if self.entity_client.service_is_ready() and self.prepare_client.service_is_ready():
                self._zero()
                self._set_stage("CANCEL_NAV")
                if self.nav_cancel_client.service_is_ready():
                    self._future = self.nav_cancel_client.call_async(
                        CancelGoal.Request()
                    )
            elif now - self.stage_started > 20.0:
                self._fail("required reset/agent services are unavailable")

        elif self.stage == "CANCEL_NAV":
            self._zero()
            # The benchmark execution mode has no nav2 goal; when a nav2
            # cancel service exists, allow its cancel-all request to settle.
            if self._future is None or self._future.done() or now - self.stage_started > 1.0:
                self._teleport()

        elif self.stage == "TELEPORT":
            self._zero()
            if self._future.done():
                try:
                    result = self._future.result().result
                except Exception as exc:  # noqa: BLE001
                    self._fail(f"SetEntityState failed: {exc}")
                    return
                if result.result != Result.RESULT_OK:
                    self._fail(
                        f"SetEntityState result {result.result}: "
                        f"{result.error_message}"
                    )
                    return
                self._confirm_count = 0
                self._set_stage("VERIFY_RESET")

        elif self.stage == "VERIFY_RESET":
            self._zero()
            if self._confirm_count >= 5:
                self._prepare()
            elif now - self.stage_started > float(
                self.get_parameter("reset_confirm_timeout_s").value
            ):
                self._fail("odometry did not confirm reset pose")

        elif self.stage == "PREPARE":
            self._zero()
            if self._future.done():
                try:
                    response = self._future.result()
                except Exception as exc:  # noqa: BLE001
                    self._fail(f"prepare_episode failed: {exc}")
                    return
                if not response.accepted:
                    self._fail(f"prepare_episode rejected: {response.message}")
                    return
                self._episode_id = response.episode_id
                self._set_stage("WAIT_PREPARED")

        elif self.stage == "WAIT_PREPARED":
            self._zero()
            if (
                self._latest_status is not None
                and self._latest_status.episode_id == self._episode_id
                and self._latest_status.state == "IDLE"
                and not self._latest_status.inference_active
            ):
                self._start_measurement()
            elif now - self.stage_started > 120.0:
                self._fail("remote model reset did not complete")

        elif self.stage == "RUNNING":
            if now - self._start_monotonic >= float(
                self.manifest["episode_timeout_s"]
            ):
                self._zero()
                if self.cancel_client.service_is_ready():
                    self.cancel_client.call_async(Trigger.Request())
                self._finalize("timeout")

        elif self.stage == "FINALIZING":
            if now - self.stage_started >= 0.2:
                self._finalize(self._terminal_pending)

    def _teleport(self):
        request = SetEntityState.Request()
        request.entity = self.manifest["entity_path"]
        request.state.header.frame_id = "world"
        start = self.route["start"]
        request.state.pose.position.x = float(start["x"])
        request.state.pose.position.y = float(start["y"])
        request.state.pose.position.z = float(start.get("z", 0.0))
        yaw = float(start["yaw"])
        request.state.pose.orientation.z = math.sin(yaw / 2.0)
        request.state.pose.orientation.w = math.cos(yaw / 2.0)
        # simulation_interfaces/SetEntityState carries no per-field enable
        # flags: the whole EntityState is applied, so the default-zero twist
        # and acceleration are exactly the requested "teleport at rest".
        request.state.header.stamp = self.get_clock().now().to_msg()
        self._future = self.entity_client.call_async(request)
        self.stage = "TELEPORT"
        self.stage_started = time.monotonic()

    def _prepare(self):
        request = PrepareVlnEpisode.Request()
        request.instruction = self.route["instruction"]
        self._future = self.prepare_client.call_async(request)
        self.stage = "PREPARE"
        self.stage_started = time.monotonic()

    def _start_measurement(self):
        if self._latest_odom is None:
            self._fail("odometry disappeared before measurement")
            return
        position = self._latest_odom.pose.pose.position
        self._path.reset(position.x, position.y)
        self._camera_frames = 0
        self._steps = []
        self._controller_start = int(
            self._latest_status.controller_tick_count
        )
        self._controller_end = self._controller_start
        self._recording = True
        self._start_monotonic = time.monotonic()
        self._start_ros_s = self.get_clock().now().nanoseconds * 1e-9
        instruction = String()
        instruction.data = self.route["instruction"]
        self.instruction_pub.publish(instruction)
        self._set_stage("RUNNING")

    def _finalize(self, terminal_reason):
        if self.done:
            return
        end_monotonic = time.monotonic()
        end_ros_s = self.get_clock().now().nanoseconds * 1e-9
        self._recording = False
        self._zero()
        duration = max(0.0, end_monotonic - self._start_monotonic)
        if self._latest_odom is None:
            final_x = final_y = float("nan")
            nav_error = float("inf")
        else:
            final = self._latest_odom.pose.pose.position
            final_x, final_y = final.x, final.y
            goal = self.route["goal"]
            nav_error = math.hypot(
                final_x - float(goal["x"]), final_y - float(goal["y"])
            )
        stopped = terminal_reason == "model_stop"
        success_1m = bool(stopped and nav_error <= 1.0)
        success_3m = bool(stopped and nav_error <= 3.0)

        def timings(key):
            return [step[key] for step in self._steps]
        rates = episode_rates(
            duration, self._camera_frames, len(self._steps),
            timings("client_latency_ms"), timings("server_total_ms"),
            timings("system1_ms"), timings("system2_ms"),
            max(0, self._controller_end - self._controller_start),
        )
        shortest = float(self.route["shortest_path_m"])
        episode = {
            "backend": self.backend,
            "route_id": self.route["route_id"],
            "repetition": self.repetition,
            "episode_id": self._episode_id,
            "prompt_hash": prompt_hash(self.route["instruction"]),
            "terminal_reason": terminal_reason,
            "final_x": final_x,
            "final_y": final_y,
            "final_navigation_error_m": nav_error,
            "executed_path_m": self._path.length_m,
            "shortest_path_m": shortest,
            "success_1m": success_1m,
            "success_3m": success_3m,
            "spl_1m": spl(success_1m, shortest, self._path.length_m),
            "spl_3m": spl(success_3m, shortest, self._path.length_m),
            "model_step_count": len(self._steps),
            "duration_s": duration,
            "measurement_start_ros_s": self._start_ros_s,
            "measurement_end_ros_s": end_ros_s,
            **rates,
        }
        os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
        with open(self.output_file, "w", encoding="utf-8") as stream:
            json.dump({"episode": episode, "steps": self._steps}, stream, indent=2)
        self.done = True
        self.get_logger().info(
            f"trial complete: {terminal_reason}, error={nav_error:.2f} m"
        )

    def _fail(self, message):
        self._zero()
        self.failure = message
        self.done = True
        self.get_logger().error(message)


def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkTrialNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.failure:
            raise RuntimeError(node.failure)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
