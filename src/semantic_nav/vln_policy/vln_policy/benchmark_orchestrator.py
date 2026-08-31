#!/usr/bin/env python3
"""Launch-time A/B orchestration and immutable result aggregation."""

import argparse
import csv
import datetime as dt
import json
import os
import shutil
import signal
import subprocess
import tempfile
import time

import requests

from .benchmark import load_manifest, summarize


def backend_order(repetition):
    return (
        ["streamvln", "dualvln"]
        if repetition % 2 == 1
        else ["dualvln", "streamvln"]
    )


def _health(url):
    response = requests.get(f"{url.rstrip('/')}/health", timeout=30.0)
    response.raise_for_status()
    payload = response.json()
    if payload.get("status") != "ok":
        raise RuntimeError(f"server at {url} is not healthy: {payload}")
    return payload


def _git_revision():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _wait_for_agent(process, timeout_s=45.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(
                f"VLN launch exited early with status {process.returncode}"
            )
        try:
            nodes = subprocess.check_output(
                ["ros2", "node", "list"], text=True,
                stderr=subprocess.DEVNULL, timeout=5.0,
            ).splitlines()
            if "/vln_agent_node" in nodes:
                return
        except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
            pass
        time.sleep(0.5)
    raise RuntimeError("timed out waiting for /vln_agent_node")


def _stop_process(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.0)


def _write_outputs(output_dir, rows, steps):
    """Write every artifact for whatever trials have completed so far."""
    if not rows:
        return
    episodes_path = os.path.join(output_dir, "episodes.csv")
    fieldnames = list(rows[0].keys())
    with open(episodes_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    with open(
        os.path.join(output_dir, "steps.jsonl"), "w", encoding="utf-8"
    ) as stream:
        for step in steps:
            stream.write(json.dumps(step, sort_keys=True) + "\n")
    with open(
        os.path.join(output_dir, "summary.json"), "w", encoding="utf-8"
    ) as stream:
        json.dump(summarize(rows), stream, indent=2, sort_keys=True)


def _validate_complete(manifest, rows):
    expected_repetitions = int(manifest["repetitions"])
    for route in manifest["routes"]:
        route_rows = [row for row in rows if row["route_id"] == route["route_id"]]
        hashes = {row["prompt_hash"] for row in route_rows}
        if len(hashes) != 1:
            raise RuntimeError(f"prompt hash mismatch for {route['route_id']}")
        for backend in ("streamvln", "dualvln"):
            count = sum(row["backend"] == backend for row in route_rows)
            if count != expected_repetitions:
                raise RuntimeError(
                    f"{route['route_id']} has {count} {backend} rows; "
                    f"expected {expected_repetitions}"
                )


def run(args):
    manifest_path = os.path.abspath(args.manifest)
    manifest = load_manifest(manifest_path)
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = os.path.abspath(
        args.output or f"vln_benchmark_{manifest['benchmark_id']}_{timestamp}"
    )
    os.makedirs(output_dir, exist_ok=False)
    manifest_snapshot = os.path.join(output_dir, "manifest.yaml")
    shutil.copy2(manifest_path, manifest_snapshot)
    os.chmod(manifest_snapshot, 0o444)

    urls = {
        "streamvln": args.streamvln_url.rstrip("/"),
        "dualvln": args.dualvln_url.rstrip("/"),
    }
    health = {name: _health(url) for name, url in urls.items()}
    metadata = {
        "benchmark_id": manifest["benchmark_id"],
        "scene_id": manifest["scene_id"],
        "git_revision": _git_revision(),
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "server_health": health,
        "parameters": {
            "execution_mode": "trajectory",
            "server_urls": urls,
            "repetitions": manifest["repetitions"],
            "episode_timeout_s": manifest["episode_timeout_s"],
            "success_radii_m": manifest["success_radii_m"],
            "controller": {
                "rate_hz": 20.0,
                "max_linear_mps": 0.25,
                "max_angular_rps": 0.5,
                "lookahead_m": 0.35,
                "final_tolerance_m": 0.12,
                "explicit_turn_tolerance_deg": 5.0,
                "no_progress_watchdog_s": 6.0,
            },
            "sensors": {
                "streamvln": ["rgb"],
                "dualvln": [
                    "rgb", "synchronized_depth", "camera_info"
                ],
            },
        },
    }

    def write_metadata():
        with open(
            os.path.join(output_dir, "metadata.json"), "w", encoding="utf-8"
        ) as stream:
            json.dump(metadata, stream, indent=2, sort_keys=True)

    write_metadata()

    rows, steps = [], []
    scratch = tempfile.mkdtemp(prefix="vln_benchmark_trials.")
    try:
        for repetition in range(1, int(manifest["repetitions"]) + 1):
            for route in manifest["routes"]:
                for backend in backend_order(repetition):
                    launch = None
                    trial_file = os.path.join(
                        scratch,
                        f"{route['route_id']}_{repetition}_{backend}.json",
                    )
                    print(
                        f"[{repetition}/{manifest['repetitions']}] "
                        f"{route['route_id']} {backend}",
                        flush=True,
                    )
                    try:
                        launch = subprocess.Popen(
                            [
                                "ros2", "launch", "vln_policy",
                                "vln_demo.launch.py",
                                f"backend:={backend}",
                                "execution_mode:=trajectory",
                                f"server_url:={urls[backend]}",
                                "viz:=false", "rviz:=false",
                                "use_sim_time:=True",
                            ],
                            start_new_session=True,
                        )
                        _wait_for_agent(launch)
                        command = [
                            "ros2", "run", "vln_policy",
                            "vln_benchmark_trial", "--ros-args",
                            "-p", f"manifest:={manifest_path}",
                            "-p", f"route_id:={route['route_id']}",
                            "-p", f"backend:={backend}",
                            "-p", f"repetition:={repetition}",
                            "-p", f"output_file:={trial_file}",
                            "-p", "use_sim_time:=True",
                        ]
                        subprocess.run(
                            command,
                            check=True,
                            timeout=float(manifest["episode_timeout_s"]) + 180.0,
                        )
                    finally:
                        _stop_process(launch)
                    with open(trial_file, "r", encoding="utf-8") as stream:
                        result = json.load(stream)
                    rows.append(result["episode"])
                    steps.extend(result["steps"])
        _validate_complete(manifest, rows)
        _write_outputs(output_dir, rows, steps)
        metadata["completed_at"] = dt.datetime.now(
            dt.timezone.utc
        ).isoformat()
        metadata["complete"] = True
        write_metadata()
    except BaseException as exc:
        # Hours of completed trials must survive one failed launch or trial:
        # keep the partial artifacts and say so in metadata, then re-raise.
        metadata["complete"] = False
        metadata["aborted_at"] = dt.datetime.now(dt.timezone.utc).isoformat()
        metadata["error"] = f"{type(exc).__name__}: {exc}"
        metadata["completed_trials"] = len(rows)
        write_metadata()
        _write_outputs(output_dir, rows, steps)
        print(
            f"benchmark aborted after {len(rows)} trials; partial results in "
            f"{output_dir}",
            flush=True,
        )
        raise
    finally:
        shutil.rmtree(scratch, ignore_errors=True)
    print(f"benchmark results: {output_dir}", flush=True)
    return output_dir


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    from ament_index_python.packages import get_package_share_directory

    package_manifest = os.path.join(
        get_package_share_directory("vln_policy"),
        "config", "benchmark_manifest_v1.yaml",
    )
    parser.add_argument("--manifest", default=package_manifest)
    parser.add_argument("--output")
    parser.add_argument(
        "--streamvln-url",
        default=os.environ.get(
            "STREAMVLN_SERVER_URL", "http://140.114.89.63:18080"
        ),
    )
    parser.add_argument(
        "--dualvln-url",
        default=os.environ.get("DUALVLN_SERVER_URL", "http://localhost:18082"),
    )
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()
