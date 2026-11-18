"""Control panel logic that needs no ROS graph.

    cd control_panel && python3 -m pytest tests -q
"""
import asyncio
import struct
import sys
import time
import zlib
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "supervisor"))

from gateway.core import TeleopGuard, clean_name, encode_png, occupancy_to_rgba, quat_to_yaw, yaw_to_quat  # noqa: E402
from gateway.webserver import encode_frame, read_frame, ws_accept_key  # noqa: E402
import supervisor as sv  # noqa: E402


# ------------------------------------------------------------------ images
def _decode_png(data: bytes) -> np.ndarray:
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos, idat = 8, b""
    w = h = ctype = None
    while pos < len(data):
        (n,) = struct.unpack("!I", data[pos:pos + 4])
        tag, body = data[pos + 4:pos + 8], data[pos + 8:pos + 8 + n]
        (crc,) = struct.unpack("!I", data[pos + 8 + n:pos + 12 + n])
        assert crc == zlib.crc32(tag + body) & 0xFFFFFFFF
        if tag == b"IHDR":
            w, h, _, ctype = struct.unpack("!IIBB", body[:10])
        elif tag == b"IDAT":
            idat += body
        pos += 12 + n
    ch = 4 if ctype == 6 else 1
    raw = np.frombuffer(zlib.decompress(idat), np.uint8).reshape(h, 1 + w * ch)
    assert (raw[:, 0] == 0).all()
    return raw[:, 1:].reshape(h, w, ch) if ch > 1 else raw[:, 1:]


def test_png_round_trips():
    img = (np.arange(6 * 5 * 4) % 256).astype(np.uint8).reshape(6, 5, 4)
    assert np.array_equal(_decode_png(encode_png(img)), img)
    gray = np.arange(12, dtype=np.uint8).reshape(3, 4)
    assert np.array_equal(_decode_png(encode_png(gray)), gray)


def test_map_image_puts_the_top_of_the_map_first():
    # OccupancyGrid row 0 is the BOTTOM (y = origin). One wall cell there.
    data = [0] * 6
    data[0] = 100          # (row 0, col 0): bottom-left
    data[5] = -1           # (row 1, col 2): top-right, unknown
    rgba = occupancy_to_rgba(data, 3, 2)
    assert rgba.shape == (2, 3, 4)
    assert rgba[1, 0, 0] < 60 and rgba[1, 0, 3] == 255     # the wall: bottom-left of the image
    assert rgba[0, 2, 3] == 0                               # unknown: transparent
    assert rgba[0, 0, 0] > 200                              # free: light


def test_yaw_round_trip():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 3.1):
        assert quat_to_yaw(*yaw_to_quat(yaw)) == pytest.approx(yaw)


def test_map_names_are_file_safe():
    assert clean_name(" lab 1F/../x ") == "lab_1F____x"
    assert clean_name("..") == ""


# ------------------------------------------------------------------ teleop
def test_teleop_clamps_and_needs_fresh_commands():
    g = TeleopGuard(max_linear=0.3, max_angular=1.0, hold_s=0.4)
    assert g.tick(now=0.0) is None                          # nobody driving: say nothing
    g.command(5.0, -9.0, now=1.0)
    assert g.tick(now=1.1) == (0.3, -1.0)                   # clamped
    assert g.tick(now=1.39) == (0.3, -1.0)
    assert g.tick(now=1.5) == (0.0, 0.0)                    # stale -> one stop
    assert g.tick(now=1.6) is None                          # then quiet (Nav2 may drive)


def test_teleop_release_stops_at_once_and_nan_is_zero():
    g = TeleopGuard()
    g.command(0.2, 0.0, now=10.0)
    g.release()
    assert g.tick(now=10.01) == (0.0, 0.0)
    g.command(float("nan"), float("inf"), now=11.0)
    assert g.tick(now=11.0) == (0.0, 0.0)


# --------------------------------------------------------------- websocket
def test_websocket_accept_key_matches_rfc6455_example():
    assert ws_accept_key("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="


def test_websocket_reads_masked_client_frames():
    payload = b'{"type":"teleop","vx":0.1}' * 10   # > 125 bytes: 16-bit length
    mask = b"\x01\x02\x03\x04"
    masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    frame = bytes([0x81, 0x80 | 126]) + struct.pack("!H", len(payload)) + mask + masked

    async def go():
        r = asyncio.StreamReader()
        r.feed_data(frame)
        return await read_frame(r)
    assert asyncio.run(go()) == (1, payload)
    assert encode_frame(b"hi")[:2] == bytes([0x81, 2])


# --------------------------------------------------------------- supervisor
def _sup(tmp_path, units):
    cfg = {"setup": "true", "maps_dir": str(tmp_path / "maps"), "units": units,
           "stop_order": ["b", "a"]}
    return sv.Supervisor(cfg, ws=tmp_path, log_dir=tmp_path / "logs")


def _wait(pred, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


def test_requires_conflicts_and_args_are_checked(tmp_path):
    s = _sup(tmp_path, {
        "a": {"cmd": "sleep 30"},
        "b": {"cmd": "sleep 30", "requires": ["a"], "conflicts": ["c"]},
        "c": {"cmd": "sleep 30"},
        "loc": {"cmd": "sleep 30 # {maps_dir}/{map}", "args": ["map"]},
    })
    ok, msg = s.start("b")
    assert not ok and "start a first" in msg
    assert s.start("a")[0]
    assert s.start("c")[0]
    ok, msg = s.start("b")
    assert not ok and "stop c first" in msg
    s.stop("c")
    assert _wait(lambda: s.units["c"].state == "stopped")
    assert s.start("b")[0]
    ok, msg = s.start("loc", {"map": "../etc"})
    assert not ok
    ok, msg = s.start("loc", {"map": "lab"})
    assert not ok and "no saved map" in msg
    (s.maps_dir / "lab.posegraph").write_text("x")
    assert s.start("loc", {"map": "lab"})[0]
    assert [m["name"] for m in s.snapshot()["maps"]] == ["lab"]
    s.stop_all(wait=True)
    assert all(u.state == "stopped" for u in s.units.values())


def test_a_unit_that_dies_is_failed_with_its_output(tmp_path):
    s = _sup(tmp_path, {"a": {"cmd": "bash -c 'echo no lidar on /dev/ttyUSB0; exit 3'"}})
    assert s.start("a")[0]
    assert _wait(lambda: s.units["a"].state == "failed")
    snap = s.snapshot()["units"]["a"]
    assert snap["exit_code"] == 3
    assert any("no lidar" in line for line in snap["log"])
    assert (tmp_path / "logs" / "a.log").read_text().startswith("no lidar")


def test_stop_is_ctrl_c_to_the_whole_process_group(tmp_path):
    # A launch file's children must go too: the unit is a shell that starts a child.
    marker = tmp_path / "child.pid"
    s = _sup(tmp_path, {"a": {"cmd": f"bash -c 'sleep 60 & echo $! > {marker}; wait'"}})
    assert s.start("a")[0]
    assert _wait(marker.exists)
    child = int(marker.read_text())
    s.stop("a")
    # A background child of a non-interactive shell ignores SIGINT, so this one
    # needs the SIGTERM that follows the 8 s grace: the escalation is tested too.
    assert _wait(lambda: s.units["a"].state == "stopped", timeout=sum(sv.STOP_GRACE_S) + 3)
    assert _wait(lambda: not Path(f"/proc/{child}").exists() or
                 Path(f"/proc/{child}/stat").read_text().split()[2] == "Z")


def test_save_map_needs_mapping_and_a_clean_name(tmp_path):
    s = _sup(tmp_path, {"mapping": {"cmd": "sleep 30"}})
    assert not s.save_map("lab")[0]                         # mapping not running
    s.start("mapping")
    assert not s.save_map("bad name!")[0]
    calls = []

    def fake_run(argv, **kw):
        calls.append(argv[-1])
        (s.maps_dir / "lab.posegraph").write_text("x")
        return type("R", (), {"returncode": 0, "stdout": "", "stderr": ""})()
    ok, msg = s.save_map("lab", run=fake_run)
    assert ok, msg
    assert "serialize_map" in calls[0] and "map_saver_cli" in calls[1]
    s.stop_all(wait=True)


def test_the_shipped_units_file_parses_and_its_commands_format(tmp_path):
    import yaml
    for path in (ROOT / "supervisor" / "units.yaml", ROOT / "tests" / "fake_units.yaml"):
        cfg = yaml.safe_load(path.read_text())
        s = sv.Supervisor(cfg, ws=tmp_path, log_dir=tmp_path / "logs")
        for name, u in s.units.items():
            u.cmd.format(ws="W", maps_dir="M", slam_params="P", map="m")
            for req in u.requires:
                for alt in req.split("|"):
                    assert alt in s.units, f"{path.name}: {name} requires unknown {alt}"
        assert set(s.stop_order) == set(s.units), path.name


def test_a_launch_that_exits_and_leaves_a_node_behind_is_cleaned_up(tmp_path):
    # ros2 launch crashing while one of its nodes lives on: the leader is gone,
    # the child still holds the output pipe (so the pump never sees the end).
    marker = tmp_path / "orphan.pid"
    s = _sup(tmp_path, {"a": {"cmd": f"bash -c 'sleep 60 & echo $! > {marker}; exit 4'"}})
    assert s.start("a")[0]
    assert _wait(marker.exists)
    orphan = int(marker.read_text())
    assert _wait(lambda: (s.tick(), s.units["a"].state)[1] == "failed")
    assert s.units["a"].exit_code == 4
    gone = lambda: not Path(f"/proc/{orphan}").exists() or Path(f"/proc/{orphan}/stat").read_text().split()[2] == "Z"
    assert _wait(gone, timeout=sum(sv.STOP_GRACE_S) + 3)
