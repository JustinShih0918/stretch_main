"""Backend tests. The StreamVLN HTTP tests double as the client-side spec of
the wire contract in vln_policy/DESIGN.md."""

import base64
import json

import numpy as np
import pytest

from vln_policy.backends import make_backend
from vln_policy.backends.base import BackendError, validate_actions
from vln_policy.backends.dummy import DummyBackend
from vln_policy.backends.streamvln_http import StreamVLNHttpBackend


class TestValidateActions:
    def test_normalizes_case_and_whitespace(self):
        assert validate_actions([" forward ", "Stop"]) == ["FORWARD", "STOP"]

    def test_rejects_unknown_token(self):
        with pytest.raises(BackendError):
            validate_actions(["FORWARD", "JUMP"])

    def test_backward_is_not_a_streamvln_wire_action(self):
        with pytest.raises(BackendError):
            validate_actions(["BACKWARD"])


class TestDummyBackend:
    def test_chunks_and_stops(self):
        backend = DummyBackend("FORWARD,FORWARD,TURN_LEFT,FORWARD,STOP",
                               chunk_size=2)
        backend.reset("go")
        first = backend.step(None, None)
        assert first.actions == ["FORWARD", "FORWARD"]
        assert not first.done
        second = backend.step(None, None)
        assert second.actions == ["TURN_LEFT", "FORWARD"]
        third = backend.step(None, None)
        assert third.actions == ["STOP"]
        assert third.done

    def test_never_splits_past_stop(self):
        backend = DummyBackend("FORWARD,STOP,FORWARD", chunk_size=4)
        backend.reset("go")
        result = backend.step(None, None)
        assert result.actions == ["FORWARD", "STOP"]
        assert result.done

    def test_appends_stop_when_missing(self):
        backend = DummyBackend("FORWARD", chunk_size=4)
        backend.reset("go")
        result = backend.step(None, None)
        assert result.actions[-1] == "STOP"
        assert result.done

    def test_reset_restarts_script(self):
        backend = DummyBackend("FORWARD,STOP", chunk_size=1)
        backend.reset("a")
        backend.step(None, None)
        backend.reset("b")
        assert backend.step(None, None).actions == ["FORWARD"]

    def test_invalid_script_raises(self):
        with pytest.raises(BackendError):
            DummyBackend("FORWARD,FLY")


class FakeResponse:
    def __init__(self, payload, status_code=200):
        self._payload = payload
        self.status_code = status_code
        self.text = json.dumps(payload)

    def json(self):
        return self._payload

    def raise_for_status(self):
        pass


@pytest.fixture
def rgb_frame():
    return np.zeros((48, 64, 3), dtype=np.uint8)


class TestStreamVLNHttpBackend:
    def test_reset_and_step_marshalling(self, monkeypatch, rgb_frame):
        requests_log = []

        def fake_post(url, json=None, timeout=None):
            requests_log.append((url, json))
            if url.endswith("/reset"):
                return FakeResponse({"session_id": "abc"})
            return FakeResponse({
                "actions": ["FORWARD", "TURN_LEFT"],
                "done": False,
                "latency_ms": 270.0,
            })

        monkeypatch.setattr("requests.post", fake_post)
        backend = StreamVLNHttpBackend("http://localhost:18080/")
        backend.reset("go to the door")
        result = backend.step(rgb_frame, None)

        reset_url, reset_body = requests_log[0]
        assert reset_url == "http://localhost:18080/reset"
        assert reset_body == {"instruction": "go to the door"}

        step_url, step_body = requests_log[1]
        assert step_url == "http://localhost:18080/step"
        assert step_body["session_id"] == "abc"
        assert step_body["odom"] is None
        # the frame must be a base64 JPEG
        jpeg = base64.b64decode(step_body["image_jpeg_b64"])
        assert jpeg[:2] == b"\xff\xd8"

        assert result.actions == ["FORWARD", "TURN_LEFT"]
        assert not result.done
        assert "270" in result.detail

    def test_step_before_reset_raises(self, rgb_frame):
        backend = StreamVLNHttpBackend()
        with pytest.raises(BackendError):
            backend.step(rgb_frame, None)

    def test_unknown_action_token_rejected(self, monkeypatch, rgb_frame):
        def fake_post(url, json=None, timeout=None):
            if url.endswith("/reset"):
                return FakeResponse({"session_id": "abc"})
            return FakeResponse({"actions": ["SPRINT"], "done": False})

        monkeypatch.setattr("requests.post", fake_post)
        backend = StreamVLNHttpBackend()
        backend.reset("go")
        with pytest.raises(BackendError):
            backend.step(rgb_frame, None)

    def test_server_down_is_backend_error(self, monkeypatch):
        import requests as requests_mod

        def fake_post(url, json=None, timeout=None):
            raise requests_mod.ConnectionError("refused")

        monkeypatch.setattr("requests.post", fake_post)
        backend = StreamVLNHttpBackend(reset_retries=1)
        with pytest.raises(BackendError, match="unreachable"):
            backend.reset("go")

    def test_http_error_status(self, monkeypatch, rgb_frame):
        def fake_post(url, json=None, timeout=None):
            if url.endswith("/reset"):
                return FakeResponse({"session_id": "abc"})
            return FakeResponse({"error": "oom"}, status_code=500)

        monkeypatch.setattr("requests.post", fake_post)
        backend = StreamVLNHttpBackend()
        backend.reset("go")
        with pytest.raises(BackendError, match="500"):
            backend.step(rgb_frame, None)


class TestRegistry:
    def test_known_backends(self):
        assert make_backend("dummy").name == "dummy"
        assert make_backend("streamvln").name == "streamvln"
        dualvln = make_backend("dualvln")
        assert dualvln.name == "dualvln"
        assert dualvln.server_url == "http://localhost:18082"
        navila = make_backend("navila")
        assert navila.name == "navila"
        assert navila.server_url == "http://localhost:18081"

    def test_unknown_backend(self):
        with pytest.raises(ValueError):
            make_backend("gpt")
