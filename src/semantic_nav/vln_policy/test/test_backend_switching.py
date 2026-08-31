"""Both remote backend names, end to end, against fake HTTP servers.

Switching backends must change only the model session (name, URL, sensor
needs). The instruction and every trajectory-follower setting are shared, so
an A/B benchmark compares models rather than controllers.
"""

import math
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from vln_policy.action_executor import TrajectoryFollowerExecutor  # noqa: E402
from vln_policy.vln_agent_node import VlnAgentNode  # noqa: E402

INSTRUCTION = "Walk past the reception desk and stop at the second door."
FOLLOWER_SETTINGS = (
    "v_lin", "v_ang", "lookahead_m", "final_tolerance_m",
    "turn_tolerance_rad", "linear_accel_mps2", "angular_accel_rps2",
    "watchdog_s", "odom_timeout_s", "control_period_s",
)
URLS = {
    "streamvln": "http://fake-stream:18080",
    "dualvln": "http://fake-dual:18082",
}


class FakeServers:
    """Stands in for both remote model servers on the ``requests`` module."""

    def __init__(self):
        self.resets = []
        self.steps = []

    def post(self, url, json=None, timeout=None):
        if url.endswith("/reset"):
            self.resets.append((url, json["instruction"]))
            return FakeResponse({"session_id": f"s{len(self.resets)}"})
        self.steps.append((url, json))
        if "dual" in url:
            return FakeResponse({
                "trajectory": [[0.25, 0.0], [0.5, 0.1]],
                "done": False,
                "timings": {"total_ms": 90.0, "system1_ms": 20.0},
            })
        return FakeResponse({
            "actions": ["FORWARD", "TURN_LEFT"], "done": False,
            "timings": {"total_ms": 200.0},
        })


class FakeResponse:
    status_code = 200
    text = ""

    def __init__(self, payload):
        self._payload = payload

    def json(self):
        return self._payload


def _rgb_msg(stamp):
    msg = Image()
    msg.header.stamp = stamp
    msg.height, msg.width = 12, 16
    msg.encoding = "rgb8"
    msg.step = msg.width * 3
    msg.data = np.zeros((12, 16, 3), dtype=np.uint8).tobytes()
    return msg


def _depth_msg(stamp):
    msg = Image()
    msg.header.stamp = stamp
    msg.height, msg.width = 12, 16
    msg.encoding = "16UC1"
    msg.step = msg.width * 2
    msg.data = np.full((12, 16), 1500, dtype=np.uint16).tobytes()
    return msg


def _camera_info():
    msg = CameraInfo()
    msg.height, msg.width = 12, 16
    msg.k = [100.0, 0.0, 7.5, 0.0, 101.0, 5.5, 0.0, 0.0, 1.0]
    return msg


def make_agent(backend):
    return VlnAgentNode(parameter_overrides=[
        Parameter("backend", value=backend),
        Parameter("execution_mode", value="trajectory"),
        Parameter("server_url", value=URLS[backend]),
        Parameter("rgb_rotation", value="none"),
    ])


def follower_settings(agent):
    follower = agent.executor_impl
    assert isinstance(follower, TrajectoryFollowerExecutor)
    return {name: getattr(follower, name) for name in FOLLOWER_SETTINGS}


def feed_observation(agent):
    """Deliver one frame the way the sim would, bypassing DDS timing."""
    stamp = agent.get_clock().now().to_msg()
    rgb, depth = _rgb_msg(stamp), _depth_msg(stamp)
    agent._on_camera_info(_camera_info())
    if agent.backend.requires_depth:
        agent._on_rgb_depth(rgb, depth)
    else:
        agent._on_rgb(rgb)


def run_episode(backend, servers, monkeypatch):
    monkeypatch.setattr("requests.post", servers.post)
    agent = make_agent(backend)
    executor = SingleThreadedExecutor()
    executor.add_node(agent)
    try:
        feed_observation(agent)
        steps_before = len(servers.steps)
        agent._on_instruction(String(data=INSTRUCTION))
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and len(servers.steps) == steps_before:
            executor.spin_once(timeout_sec=0.05)
        assert len(servers.steps) > steps_before, f"{backend} never stepped"
        settings = follower_settings(agent)
    finally:
        executor.shutdown()
        agent.destroy_node()
    return settings


@pytest.fixture
def ros():
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def test_both_backends_run_with_shared_prompt_and_follower(ros, monkeypatch):
    servers = FakeServers()
    stream_settings = run_episode("streamvln", servers, monkeypatch)
    dual_settings = run_episode("dualvln", servers, monkeypatch)

    # Each backend talked to its own server with the identical instruction.
    assert [url for url, _ in servers.resets] == [
        f"{URLS['streamvln']}/reset", f"{URLS['dualvln']}/reset"
    ]
    assert {instruction for _, instruction in servers.resets} == {INSTRUCTION}
    assert [url for url, _ in servers.steps] == [
        f"{URLS['streamvln']}/step", f"{URLS['dualvln']}/step"
    ]

    # Switching the model never changes the controller.
    assert stream_settings == dual_settings
    assert stream_settings["v_lin"] == 0.25
    assert stream_settings["v_ang"] == 0.5
    assert stream_settings["lookahead_m"] == 0.35
    assert stream_settings["final_tolerance_m"] == 0.12
    assert stream_settings["turn_tolerance_rad"] == pytest.approx(
        math.radians(5.0)
    )
    assert stream_settings["watchdog_s"] == 6.0
    assert stream_settings["control_period_s"] == pytest.approx(1.0 / 20.0)


def test_only_dualvln_sends_depth_and_intrinsics(ros, monkeypatch):
    servers = FakeServers()
    run_episode("streamvln", servers, monkeypatch)
    run_episode("dualvln", servers, monkeypatch)

    stream_payload = servers.steps[0][1]
    dual_payload = servers.steps[1][1]
    assert "depth_png_b64" not in stream_payload
    assert "camera_intrinsics" not in stream_payload
    assert dual_payload["depth_scale_m"] == pytest.approx(0.001)
    assert dual_payload["camera_intrinsics"] == {
        "fx": 100.0, "fy": 101.0, "cx": 7.5, "cy": 5.5,
        "width": 16, "height": 12,
    }
    assert dual_payload["image_timestamp_s"] > 0.0
