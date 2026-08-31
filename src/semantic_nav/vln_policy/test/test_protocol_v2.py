"""Executable specification for the additive HTTP protocol-v2 fields."""

import base64
import json

import cv2
import numpy as np
import pytest

from vln_policy.backends.base import BackendError, CameraIntrinsics
from vln_policy.backends.dualvln_http import DualVLNHttpBackend
from vln_policy.backends.http import validate_trajectory
from vln_policy.backends.streamvln_http import StreamVLNHttpBackend
from vln_policy.server_url import resolve_server_url


class FakeResponse:
    def __init__(self, payload, status_code=200):
        self.payload = payload
        self.status_code = status_code
        self.text = json.dumps(payload)

    def json(self):
        return self.payload


@pytest.fixture
def observation():
    return (
        np.zeros((12, 16, 3), dtype=np.uint8),
        np.full((12, 16), 1234, dtype=np.uint16),
        CameraIntrinsics(100.0, 101.0, 7.5, 5.5, 16, 12),
    )


def test_dualvln_marshals_paired_depth_intrinsics_and_timings(
    monkeypatch, observation
):
    calls = []

    def post(url, json=None, timeout=None):
        calls.append((url, json))
        if url.endswith("/reset"):
            return FakeResponse({"session_id": "session-a"})
        return FakeResponse({
            "trajectory": [[0, 0], [0.25, 0.1]],
            "done": False,
            "timings": {
                "total_ms": 100.0,
                "preprocessing_ms": 5.0,
                "system1_ms": 20.0,
                "system2_ms": 70.0,
            },
        })

    monkeypatch.setattr("requests.post", post)
    rgb, depth, intrinsics = observation
    backend = DualVLNHttpBackend()
    backend.reset("go")
    result = backend.step(
        rgb, None, depth=depth, depth_scale_m=0.001,
        intrinsics=intrinsics, image_timestamp_s=12.5,
    )

    payload = calls[-1][1]
    assert payload["camera_intrinsics"] == intrinsics.as_dict()
    assert payload["image_timestamp_s"] == 12.5
    png = base64.b64decode(payload["depth_png_b64"])
    decoded = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
    assert decoded.dtype == np.uint16
    assert np.array_equal(decoded, depth)
    assert [(p.x, p.y) for p in result.trajectory] == [
        (0.0, 0.0), (0.25, 0.1)
    ]
    assert result.timings.system1_ms == 20.0
    assert result.timings.system2_ms == 70.0
    assert result.output_type == "trajectory"


@pytest.mark.parametrize("reply", [
    {"actions": ["FORWARD"], "trajectory": [[0, 0]]},
    {"done": False},
    {"trajectory": [[0, float("nan")]]},
    {"trajectory": [[0, 1, 2]]},
    {"trajectory": []},
])
def test_malformed_or_ambiguous_output_is_rejected(
    monkeypatch, observation, reply
):
    def post(url, json=None, timeout=None):
        if url.endswith("/reset"):
            return FakeResponse({"session_id": "session-a"})
        return FakeResponse(reply)

    monkeypatch.setattr("requests.post", post)
    rgb, depth, intrinsics = observation
    backend = DualVLNHttpBackend()
    backend.reset("go")
    with pytest.raises(BackendError):
        backend.step(
            rgb, None, depth=depth, depth_scale_m=0.001,
            intrinsics=intrinsics,
        )


def test_invalid_timing_is_rejected(monkeypatch, observation):
    def post(url, json=None, timeout=None):
        if url.endswith("/reset"):
            return FakeResponse({"session_id": "session-a"})
        return FakeResponse({
            "actions": ["FORWARD"],
            "timings": {"system1_ms": -1},
        })

    monkeypatch.setattr("requests.post", post)
    rgb, depth, intrinsics = observation
    backend = DualVLNHttpBackend()
    backend.reset("go")
    with pytest.raises(BackendError, match="system1_ms"):
        backend.step(
            rgb, None, depth=depth, depth_scale_m=0.001,
            intrinsics=intrinsics,
        )


def test_stale_session_http_conflict_is_backend_error(monkeypatch, observation):
    def post(url, json=None, timeout=None):
        if url.endswith("/reset"):
            return FakeResponse({"session_id": "old"})
        return FakeResponse({"detail": "unknown session_id"}, status_code=409)

    monkeypatch.setattr("requests.post", post)
    rgb, depth, intrinsics = observation
    backend = DualVLNHttpBackend()
    backend.reset("go")
    with pytest.raises(BackendError, match="409"):
        backend.step(
            rgb, None, depth=depth, depth_scale_m=0.001,
            intrinsics=intrinsics,
        )


def test_validate_trajectory_rejects_non_finite_values():
    with pytest.raises(BackendError, match="finite"):
        validate_trajectory([[float("inf"), 0]])


def test_backend_specific_url_selection_and_override():
    assert resolve_server_url("streamvln", "", "http://s", "http://d") == "http://s"
    assert resolve_server_url("dualvln", "", "http://s", "http://d") == "http://d"
    assert resolve_server_url(
        "dualvln", "http://override", "http://s", "http://d"
    ) == "http://override"


def test_both_remote_backends_send_the_identical_reset_prompt(monkeypatch):
    prompts = []

    def post(url, json=None, timeout=None):
        assert url.endswith("/reset")
        prompts.append(json["instruction"])
        return FakeResponse({"session_id": str(len(prompts))})

    monkeypatch.setattr("requests.post", post)
    instruction = "Take the same exact route to the nurses station."
    StreamVLNHttpBackend("http://stream").reset(instruction)
    DualVLNHttpBackend("http://dual").reset(instruction)
    assert prompts == [instruction, instruction]
