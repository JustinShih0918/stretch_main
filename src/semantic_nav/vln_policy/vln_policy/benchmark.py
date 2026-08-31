"""Pure benchmark manifest, path, SPL, rate, and summary calculations."""

import hashlib
import math
import statistics
from dataclasses import dataclass

import yaml


class ManifestError(ValueError):
    pass


def _finite(value, label):
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ManifestError(f"{label} must be numeric") from exc
    if not math.isfinite(number):
        raise ManifestError(f"{label} must be finite")
    return number


def polyline_length(points) -> float:
    if len(points) < 2:
        raise ManifestError("reference_path must contain at least two points")
    parsed = []
    for index, point in enumerate(points):
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            raise ManifestError(f"reference_path[{index}] must be [x, y]")
        parsed.append((
            _finite(point[0], f"reference_path[{index}].x"),
            _finite(point[1], f"reference_path[{index}].y"),
        ))
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(parsed, parsed[1:])
    )


def validate_manifest(data: dict) -> dict:
    if not isinstance(data, dict):
        raise ManifestError("manifest root must be a mapping")
    for key in ("version", "benchmark_id", "scene_id", "routes"):
        if key not in data:
            raise ManifestError(f"manifest missing {key}")
    if int(data["version"]) != 1:
        raise ManifestError("unsupported manifest version (expected 1)")
    if int(data.get("repetitions", 0)) <= 0:
        raise ManifestError("repetitions must be positive")
    if _finite(data.get("episode_timeout_s", 0), "episode_timeout_s") <= 0:
        raise ManifestError("episode_timeout_s must be positive")
    radii = [
        _finite(value, "success radius")
        for value in data.get("success_radii_m", [])
    ]
    if radii != [1.0, 3.0]:
        raise ManifestError("success_radii_m must be [1.0, 3.0]")
    if not data.get("entity_path"):
        raise ManifestError("entity_path must not be empty")
    if not isinstance(data["routes"], list) or not data["routes"]:
        raise ManifestError("routes must be a non-empty list")

    route_ids = set()
    for route in data["routes"]:
        route_id = str(route.get("route_id", "")).strip()
        if not route_id or route_id in route_ids:
            raise ManifestError("route_id values must be non-empty and unique")
        route_ids.add(route_id)
        if not str(route.get("instruction", "")).strip():
            raise ManifestError(f"route {route_id} has an empty instruction")
        start, goal = route.get("start", {}), route.get("goal", {})
        sx, sy = _finite(start.get("x"), "start.x"), _finite(
            start.get("y"), "start.y"
        )
        _finite(start.get("yaw"), "start.yaw")
        gx, gy = _finite(goal.get("x"), "goal.x"), _finite(
            goal.get("y"), "goal.y"
        )
        path = route.get("reference_path", [])
        shortest = polyline_length(path)
        first, last = path[0], path[-1]
        if math.hypot(float(first[0]) - sx, float(first[1]) - sy) > 0.25:
            raise ManifestError(
                f"route {route_id} reference path does not start within 0.25 m"
            )
        if math.hypot(float(last[0]) - gx, float(last[1]) - gy) > 0.25:
            raise ManifestError(
                f"route {route_id} reference path does not end within 0.25 m"
            )
        if shortest <= 0.0:
            raise ManifestError(f"route {route_id} shortest path is zero")
        route["shortest_path_m"] = shortest
    return data


def load_manifest(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as stream:
        return validate_manifest(yaml.safe_load(stream))


def prompt_hash(instruction: str) -> str:
    return hashlib.sha256(instruction.encode("utf-8")).hexdigest()


def reset_pose_in_tolerance(
    x, y, yaw, start, position_tolerance_m=0.05,
    yaw_tolerance_rad=math.radians(3.0),
) -> bool:
    position_error = math.hypot(float(x) - float(start["x"]),
                                float(y) - float(start["y"]))
    yaw_error = math.atan2(
        math.sin(float(yaw) - float(start["yaw"])),
        math.cos(float(yaw) - float(start["yaw"])),
    )
    return (
        position_error <= float(position_tolerance_m)
        and abs(yaw_error) <= float(yaw_tolerance_rad)
    )


def spl(success: bool, shortest_path_m: float, executed_path_m: float) -> float:
    shortest = float(shortest_path_m)
    executed = float(executed_path_m)
    if shortest <= 0.0 or executed < 0.0:
        raise ValueError("path lengths must be positive/non-negative")
    return float(bool(success)) * shortest / max(shortest, executed)


@dataclass
class PathIntegrator:
    length_m: float = 0.0
    _last: tuple = None

    def reset(self, x: float, y: float):
        self.length_m = 0.0
        self._last = (float(x), float(y))

    def add(self, x: float, y: float) -> float:
        point = (float(x), float(y))
        if self._last is None:
            self._last = point
            return self.length_m
        self.length_m += math.hypot(
            point[0] - self._last[0], point[1] - self._last[1]
        )
        self._last = point
        return self.length_m


def safe_rate(count: int, elapsed_s: float):
    return float(count) / elapsed_s if elapsed_s > 0.0 else None


def timing_fps(timings_ms):
    available = [
        float(value) for value in timings_ms
        if value is not None and math.isfinite(float(value))
    ]
    total_s = sum(available) / 1000.0
    return safe_rate(len(available), total_s)


def episode_rates(
    duration_s,
    camera_frames,
    completed_steps,
    client_timings_ms,
    server_timings_ms,
    system1_timings_ms,
    system2_timings_ms,
    controller_ticks,
) -> dict:
    return {
        "camera_input_hz": safe_rate(camera_frames, duration_s),
        "model_updates_per_episode_s": safe_rate(completed_steps, duration_s),
        "client_request_response_hz": timing_fps(client_timings_ms),
        "server_compute_fps": timing_fps(server_timings_ms),
        "system1_fps": timing_fps(system1_timings_ms),
        "system2_fps": timing_fps(system2_timings_ms),
        "trajectory_control_hz": safe_rate(controller_ticks, duration_s),
    }


SUMMARY_METRICS = (
    "success_1m", "success_3m", "spl_1m", "spl_3m",
    "final_navigation_error_m", "duration_s", "model_step_count",
    "camera_input_hz", "model_updates_per_episode_s",
    "client_request_response_hz", "server_compute_fps", "system1_fps",
    "system2_fps", "trajectory_control_hz",
)


def _stats(rows):
    result = {}
    for key in SUMMARY_METRICS:
        values = [
            float(row[key]) for row in rows
            if row.get(key) is not None and math.isfinite(float(row[key]))
        ]
        result[key] = {
            "mean": statistics.fmean(values) if values else None,
            "std": statistics.pstdev(values) if values else None,
            "count": len(values),
        }
    return result


def summarize(rows) -> dict:
    by_route = {}
    by_backend = {}
    for row in rows:
        route_key = f"{row['route_id']}::{row['backend']}"
        by_route.setdefault(route_key, []).append(row)
        by_backend.setdefault(row["backend"], []).append(row)
    return {
        "per_route_backend": {
            key: _stats(values) for key, values in sorted(by_route.items())
        },
        "per_backend": {
            key: _stats(values) for key, values in sorted(by_backend.items())
        },
        "aggregate": _stats(rows),
    }
