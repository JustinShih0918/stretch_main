"""Reset sequencing driven against fake Isaac / agent services.

Runs the real BenchmarkTrialNode over a fake ``/set_entity_state`` service, a
fake ``prepare_episode`` service, and hand-published odometry/status, so the
ordering the benchmark depends on — teleport, odometry confirmation, model
reset, and only then the instruction that opens the measurement window — is
checked without Isaac Sim.
"""

import json
import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("simulation_interfaces")

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from simulation_interfaces.msg import Result  # noqa: E402
from simulation_interfaces.srv import SetEntityState  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from btcpp_ros2_interfaces.msg import VlnStatus  # noqa: E402
from btcpp_ros2_interfaces.srv import PrepareVlnEpisode  # noqa: E402

from vln_policy.benchmark_trial_node import BenchmarkTrialNode  # noqa: E402

MANIFEST = """
version: 1
benchmark_id: reset_sequencing_test
scene_id: fake_scene
entity_path: /World/stretch3
repetitions: 1
episode_timeout_s: 300.0
success_radii_m: [1.0, 3.0]
routes:
  - route_id: r1
    instruction: "Go to the nurses station."
    start: {x: 1.0, y: -2.0, z: 0.037, yaw: 1.5707963267948966}
    goal: {x: 3.0, y: -2.0}
    reference_path:
      - [1.0, -2.0]
      - [2.0, -2.0]
      - [3.0, -2.0]
"""

START_X, START_Y, START_YAW = 1.0, -2.0, math.pi / 2.0
FAR_X = START_X + 0.4  # well outside the 5 cm confirmation window
EPISODE_ID = "episode-under-test"
INSTRUCTION = "Go to the nurses station."


def _odom(x, y, yaw):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


class FakeWorld(Node):
    """Isaac Sim + vln_agent_node stand-in that records the call order."""

    def __init__(self):
        super().__init__("fake_world")
        self.events = []
        self.entity_requests = []
        self.instructions = []
        self.zero_twists = 0
        self.create_service(
            SetEntityState, "/set_entity_state", self._on_set_entity_state
        )
        self.create_service(
            PrepareVlnEpisode,
            "/vln_agent_node/prepare_episode",
            self._on_prepare,
        )
        self.create_subscription(
            String, "/vln_instruction", self._on_instruction, 1
        )
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 50)
        self.status_pub = self.create_publisher(VlnStatus, "/vln/status", 10)

    def _on_set_entity_state(self, request, response):
        self.entity_requests.append(request)
        self.events.append("teleport")
        response.result.result = Result.RESULT_OK
        return response

    def _on_prepare(self, request, response):
        self.events.append(f"prepare:{request.instruction}")
        response.accepted = True
        response.episode_id = EPISODE_ID
        response.message = "ok"
        return response

    def _on_instruction(self, msg):
        self.events.append("instruction")
        self.instructions.append(msg.data)

    def _on_cmd_vel(self, msg):
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            self.zero_twists += 1

    def publish_odom(self, x=START_X, y=START_Y, yaw=START_YAW):
        self.odom_pub.publish(_odom(x, y, yaw))

    def publish_status(self, state, terminal_reason="", ticks=0):
        msg = VlnStatus()
        msg.state = state
        msg.episode_id = EPISODE_ID
        msg.terminal_reason = terminal_reason
        msg.inference_active = False
        msg.controller_tick_count = ticks
        self.status_pub.publish(msg)


class Harness:
    """Drives both nodes in real time; one ``spin`` step is one odom sample."""

    STEP_S = 0.02

    def __init__(self, tmp_path):
        manifest = tmp_path / "manifest.yaml"
        manifest.write_text(MANIFEST, encoding="utf-8")
        self.output = tmp_path / "trial.json"
        rclpy.init()
        self.world = FakeWorld()
        self.trial = BenchmarkTrialNode(parameter_overrides=[
            Parameter("manifest", value=str(manifest)),
            Parameter("route_id", value="r1"),
            Parameter("backend", value="dummy"),
            Parameter("repetition", value=1),
            Parameter("output_file", value=str(self.output)),
            Parameter("reset_confirm_timeout_s", value=60.0),
        ])
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.world)
        self.executor.add_node(self.trial)

    def spin(self, iterations=1, odom=True, pose=None):
        for _ in range(iterations):
            if odom:
                self.world.publish_odom(*(pose or ()))
            deadline = time.monotonic() + self.STEP_S
            while time.monotonic() < deadline:
                self.executor.spin_once(timeout_sec=0.005)

    def run_until(self, predicate, iterations=250, odom=True, pose=None):
        for _ in range(iterations):
            if predicate():
                return True
            self.spin(1, odom=odom, pose=pose)
        return bool(predicate())

    def reach_verify_reset(self):
        # No odometry until VERIFY_RESET, so the confirmation counter starts
        # from a known zero.
        assert self.run_until(
            lambda: self.trial.stage == "VERIFY_RESET", odom=False
        )
        assert self.trial._confirm_count == 0

    def close(self):
        self.executor.shutdown()
        self.trial.destroy_node()
        self.world.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def harness(tmp_path):
    instance = Harness(tmp_path)
    try:
        yield instance
    finally:
        instance.close()


def test_teleport_request_carries_entity_start_pose_and_zero_twist(harness):
    assert harness.run_until(
        lambda: harness.world.entity_requests, odom=False
    )
    request = harness.world.entity_requests[0]
    assert request.entity == "/World/stretch3"
    assert request.state.pose.position.x == pytest.approx(START_X)
    assert request.state.pose.position.y == pytest.approx(START_Y)
    assert request.state.pose.position.z == pytest.approx(0.037)
    yaw = 2.0 * math.atan2(
        request.state.pose.orientation.z, request.state.pose.orientation.w
    )
    assert yaw == pytest.approx(START_YAW)
    twist = request.state.twist
    assert (twist.linear.x, twist.linear.y, twist.linear.z) == (0.0, 0.0, 0.0)
    assert (
        twist.angular.x, twist.angular.y, twist.angular.z
    ) == (0.0, 0.0, 0.0)
    # Velocity is zeroed throughout the reset sequence.
    assert harness.world.zero_twists > 0


def test_out_of_tolerance_odometry_never_confirms_the_reset(harness):
    harness.reach_verify_reset()
    harness.spin(40, pose=(FAR_X, START_Y, START_YAW))
    assert harness.trial.stage == "VERIFY_RESET"
    assert harness.trial._confirm_count == 0
    assert not any(
        event.startswith("prepare") for event in harness.world.events
    )


def test_confirmation_needs_five_consecutive_in_tolerance_samples(harness):
    harness.reach_verify_reset()
    harness.spin(4)
    assert harness.trial._confirm_count == 4
    assert harness.trial.stage == "VERIFY_RESET"

    harness.spin(1, pose=(FAR_X, START_Y, START_YAW))
    assert harness.trial._confirm_count == 0
    assert harness.trial.stage == "VERIFY_RESET"

    assert harness.run_until(lambda: harness.trial.stage != "VERIFY_RESET")


def test_instruction_is_published_only_after_teleport_and_model_reset(harness):
    harness.reach_verify_reset()
    assert harness.run_until(lambda: harness.trial.stage == "WAIT_PREPARED")
    assert not harness.world.instructions

    harness.world.publish_status("IDLE")
    assert harness.run_until(lambda: harness.world.instructions)
    assert harness.world.events == [
        "teleport", f"prepare:{INSTRUCTION}", "instruction"
    ]
    assert harness.world.instructions == [INSTRUCTION]
    assert harness.trial.stage == "RUNNING"


def test_reset_motion_is_excluded_from_the_measured_path(harness):
    harness.reach_verify_reset()
    assert harness.run_until(lambda: harness.trial.stage == "WAIT_PREPARED")
    harness.world.publish_status("IDLE", ticks=100)
    assert harness.run_until(lambda: harness.trial.stage == "RUNNING")

    # Only this post-instruction motion may count: 1.5 m of travel.
    for step in range(1, 16):
        harness.spin(1, pose=(START_X + step / 10.0, START_Y, START_YAW))
    harness.world.publish_status("DONE", "model_stop", ticks=140)
    assert harness.run_until(
        lambda: harness.trial.done, pose=(2.5, START_Y, START_YAW)
    )

    episode = json.loads(harness.output.read_text(encoding="utf-8"))["episode"]
    assert episode["terminal_reason"] == "model_stop"
    assert episode["episode_id"] == EPISODE_ID
    assert episode["executed_path_m"] == pytest.approx(1.5, abs=0.02)
    assert episode["shortest_path_m"] == pytest.approx(2.0)
    # Final pose (2.5, -2) is 0.5 m from the (3, -2) goal.
    assert episode["final_navigation_error_m"] == pytest.approx(0.5, abs=0.02)
    assert episode["success_1m"] is True
    assert episode["success_3m"] is True
    assert episode["spl_1m"] == pytest.approx(1.0)
    assert episode["duration_s"] > 0.0
    assert episode["trajectory_control_hz"] is not None
