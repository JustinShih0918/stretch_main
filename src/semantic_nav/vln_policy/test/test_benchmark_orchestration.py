"""Result-directory contract of the A/B benchmark runner.

Both launches and both trials are faked, so this checks what the orchestrator
itself owns: launch-time model alternation, the five immutable artifacts, and
that a mid-run failure keeps every trial already collected.
"""

import argparse
import csv
import json
import subprocess

import pytest
import yaml

from vln_policy import benchmark_orchestrator as orchestrator

MANIFEST = {
    "version": 1,
    "benchmark_id": "orchestration_test",
    "scene_id": "fake_scene",
    "entity_path": "/World/stretch3",
    "repetitions": 2,
    "episode_timeout_s": 30.0,
    "success_radii_m": [1.0, 3.0],
    "routes": [{
        "route_id": "r1",
        "instruction": "Stop at the second door.",
        "start": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "goal": {"x": 2.0, "y": 0.0},
        "reference_path": [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
    }],
}


class FakeProcess:
    pid = 4321
    returncode = 0

    def poll(self):
        return None

    def wait(self, timeout=None):
        return 0


def _episode(backend, repetition):
    return {
        "backend": backend,
        "route_id": "r1",
        "repetition": repetition,
        "episode_id": f"{backend}-{repetition}",
        "prompt_hash": "hash-of-the-one-instruction",
        "terminal_reason": "model_stop",
        "final_navigation_error_m": 0.4,
        "executed_path_m": 2.4,
        "shortest_path_m": 2.0,
        "success_1m": True,
        "success_3m": True,
        "spl_1m": 2.0 / 2.4,
        "spl_3m": 2.0 / 2.4,
        "model_step_count": 7,
        "duration_s": 21.0,
        "camera_input_hz": 30.0,
        "model_updates_per_episode_s": 0.33,
        "client_request_response_hz": 4.0,
        "server_compute_fps": 5.0,
        "system1_fps": None,
        "system2_fps": 4.5,
        "trajectory_control_hz": 20.0,
    }


class FakeRuns:
    """Stands in for ``ros2 run vln_benchmark_trial`` only."""

    def __init__(self, fail_on=None):
        self.launched = []
        self.fail_on = fail_on

    def parse(self, command):
        values = {}
        for index, token in enumerate(command):
            if token == "-p":
                key, _, value = command[index + 1].partition(":=")
                values[key] = value
        return values

    def __call__(self, command, **kwargs):
        assert "vln_benchmark_trial" in command, command
        values = self.parse(command)
        backend, repetition = values["backend"], int(values["repetition"])
        self.launched.append((repetition, backend))
        if self.fail_on == len(self.launched):
            raise subprocess.CalledProcessError(1, command)
        with open(values["output_file"], "w", encoding="utf-8") as stream:
            json.dump({
                "episode": _episode(backend, repetition),
                "steps": [{
                    "backend": backend, "route_id": "r1",
                    "repetition": repetition, "model_step": 1,
                }],
            }, stream)
        return subprocess.CompletedProcess(command, 0)


@pytest.fixture
def runner(tmp_path, monkeypatch):
    manifest_path = tmp_path / "manifest.yaml"
    manifest_path.write_text(yaml.safe_dump(MANIFEST), encoding="utf-8")
    monkeypatch.setattr(
        orchestrator, "_health",
        lambda url: {"status": "ok", "model_revision": url},
    )
    monkeypatch.setattr(orchestrator, "_git_revision", lambda: "deadbeef")
    monkeypatch.setattr(orchestrator, "_wait_for_agent", lambda process: None)
    monkeypatch.setattr(orchestrator, "_stop_process", lambda process: None)
    monkeypatch.setattr(
        orchestrator.subprocess, "Popen",
        lambda *args, **kwargs: FakeProcess(),
    )

    def build(fail_on=None):
        runs = FakeRuns(fail_on=fail_on)
        monkeypatch.setattr(orchestrator.subprocess, "run", runs)
        args = argparse.Namespace(
            manifest=str(manifest_path),
            output=str(tmp_path / "results"),
            streamvln_url="http://stream:18080",
            dualvln_url="http://dual:18082",
        )
        return runs, args

    return build


def test_complete_run_alternates_models_and_writes_every_artifact(
    runner, tmp_path
):
    runs, args = runner()
    output_dir = tmp_path / "results"
    assert orchestrator.run(args) == str(output_dir)

    assert runs.launched == [
        (1, "streamvln"), (1, "dualvln"), (2, "dualvln"), (2, "streamvln")
    ]
    for name in (
        "manifest.yaml", "metadata.json", "steps.jsonl",
        "episodes.csv", "summary.json",
    ):
        assert (output_dir / name).is_file(), name

    snapshot = yaml.safe_load((output_dir / "manifest.yaml").read_text())
    assert snapshot["benchmark_id"] == "orchestration_test"

    metadata = json.loads((output_dir / "metadata.json").read_text())
    assert metadata["complete"] is True
    assert metadata["git_revision"] == "deadbeef"
    assert set(metadata["server_health"]) == {"streamvln", "dualvln"}
    assert metadata["parameters"]["execution_mode"] == "trajectory"

    with open(output_dir / "episodes.csv", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 4
    assert sorted(row["backend"] for row in rows) == [
        "dualvln", "dualvln", "streamvln", "streamvln"
    ]

    steps = [
        json.loads(line)
        for line in (output_dir / "steps.jsonl").read_text().splitlines()
    ]
    assert len(steps) == 4

    summary = json.loads((output_dir / "summary.json").read_text())
    assert set(summary["per_backend"]) == {"streamvln", "dualvln"}
    assert summary["aggregate"]["spl_1m"]["mean"] == pytest.approx(2.0 / 2.4)


def test_a_failed_trial_keeps_the_trials_already_collected(runner, tmp_path):
    runs, args = runner(fail_on=3)
    output_dir = tmp_path / "results"
    with pytest.raises(subprocess.CalledProcessError):
        orchestrator.run(args)

    metadata = json.loads((output_dir / "metadata.json").read_text())
    assert metadata["complete"] is False
    assert metadata["completed_trials"] == 2
    assert "CalledProcessError" in metadata["error"]

    with open(output_dir / "episodes.csv", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert [row["backend"] for row in rows] == ["streamvln", "dualvln"]


def test_a_mismatched_prompt_hash_fails_validation():
    rows = [
        dict(_episode("streamvln", 1), prompt_hash="a"),
        dict(_episode("streamvln", 2), prompt_hash="b"),
        dict(_episode("dualvln", 1), prompt_hash="a"),
        dict(_episode("dualvln", 2), prompt_hash="a"),
    ]
    with pytest.raises(RuntimeError, match="prompt hash mismatch"):
        orchestrator._validate_complete(
            {"repetitions": 2, "routes": [{"route_id": "r1"}]}, rows
        )
