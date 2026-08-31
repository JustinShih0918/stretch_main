import copy
import math

import pytest

from vln_policy.benchmark import (
    ManifestError,
    PathIntegrator,
    episode_rates,
    polyline_length,
    reset_pose_in_tolerance,
    spl,
    summarize,
    validate_manifest,
)
from vln_policy.benchmark_orchestrator import backend_order


def manifest():
    return {
        "version": 1,
        "benchmark_id": "test",
        "scene_id": "scene",
        "entity_path": "/World/stretch3",
        "repetitions": 5,
        "episode_timeout_s": 300,
        "success_radii_m": [1.0, 3.0],
        "routes": [{
            "route_id": "r1",
            "instruction": "go",
            "start": {"x": 0, "y": 0, "yaw": 0},
            "goal": {"x": 3, "y": 4},
            "reference_path": [[0, 0], [0, 4], [3, 4]],
        }],
    }


def test_manifest_uses_polyline_length_not_straight_line():
    parsed = validate_manifest(manifest())
    assert parsed["routes"][0]["shortest_path_m"] == 7.0
    assert polyline_length([[0, 0], [0, 4], [3, 4]]) == 7.0


@pytest.mark.parametrize("endpoint", ["start", "goal"])
def test_manifest_rejects_reference_endpoint_mismatch(endpoint):
    data = manifest()
    if endpoint == "start":
        data["routes"][0]["reference_path"][0] = [1, 1]
    else:
        data["routes"][0]["reference_path"][-1] = [4, 5]
    with pytest.raises(ManifestError, match="within 0.25"):
        validate_manifest(data)


def test_odometry_path_integration_starts_only_at_explicit_reset():
    integrator = PathIntegrator()
    integrator.add(100, 100)  # pre-window motion
    integrator.reset(0, 0)
    integrator.add(3, 4)
    integrator.add(6, 8)
    assert integrator.length_m == 10.0


def test_spl_both_success_radii_and_executed_path_floor():
    assert spl(True, 5, 4) == 1.0
    assert spl(True, 5, 10) == 0.5
    assert spl(False, 5, 5) == 0.0


def test_all_rate_formulas_are_separate():
    rates = episode_rates(
        duration_s=10,
        camera_frames=100,
        completed_steps=4,
        client_timings_ms=[100, 100, 100, 100],
        server_timings_ms=[50, 50, 50, 50],
        system1_timings_ms=[20, 20, 20, 20],
        system2_timings_ms=[30, 30, 30, 30],
        controller_ticks=200,
    )
    assert rates == {
        "camera_input_hz": 10.0,
        "model_updates_per_episode_s": 0.4,
        "client_request_response_hz": 10.0,
        "server_compute_fps": 20.0,
        "system1_fps": 50.0,
        "system2_fps": pytest.approx(100 / 3),
        "trajectory_control_hz": 20.0,
    }


def test_reset_pose_tolerance_handles_yaw_wraparound():
    start = {"x": 1, "y": 2, "yaw": math.pi - 0.01}
    assert reset_pose_in_tolerance(1.03, 2.02, -math.pi + 0.01, start)
    assert not reset_pose_in_tolerance(1.06, 2, start["yaw"], start)


def test_launch_order_alternates_across_five_repetitions():
    assert [backend_order(i)[0] for i in range(1, 6)] == [
        "streamvln", "dualvln", "streamvln", "dualvln", "streamvln"
    ]


def test_summary_has_route_and_aggregate_mean_std():
    base = {
        "route_id": "r", "backend": "streamvln",
        "success_1m": True, "success_3m": True,
        "spl_1m": 1, "spl_3m": 1,
        "final_navigation_error_m": 0.5, "duration_s": 10,
        "model_step_count": 2, "camera_input_hz": 10,
        "model_updates_per_episode_s": 0.2,
        "client_request_response_hz": 2,
        "server_compute_fps": 4, "system1_fps": None,
        "system2_fps": 5, "trajectory_control_hz": 20,
    }
    other = copy.deepcopy(base)
    other["duration_s"] = 20
    summary = summarize([base, other])
    stats = summary["aggregate"]["duration_s"]
    assert stats == {"mean": 15.0, "std": 5.0, "count": 2}
