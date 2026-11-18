#!/usr/bin/env python3
"""End to end on the fake robot: supervisor + gateway + tools/fake_stretch.py,
driven through the gateway's WebSocket exactly as the page drives it.

    bash control_panel/tests/e2e_fake.sh          # starts everything, runs this, cleans up

Standard library only (a 40-line WebSocket client below), so it runs with the
same python3 as ROS.
"""
from __future__ import annotations

import base64
import json
import math
import os
import socket
import struct
import sys
import time
import urllib.request

PORT = int(os.environ.get("PANEL_PORT", "18080"))
MAPS = os.environ.get("PANEL_MAPS_DIR", "/tmp/panel_fake_maps")


class WS:
    def __init__(self, host="127.0.0.1", port=PORT):
        self.s = socket.create_connection((host, port), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        self.s.sendall((f"GET /ws HTTP/1.1\r\nHost: {host}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        head = b""
        while b"\r\n\r\n" not in head:
            head += self.s.recv(1)
        assert b" 101 " in head.split(b"\r\n")[0], head
        self.state, self.replies, self.buf, self.n = None, {}, b"", 0
        self.last_checks = []

    def _read(self, n):
        while len(self.buf) < n:
            chunk = self.s.recv(65536)
            if not chunk:
                raise ConnectionError("closed")
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def pump(self, timeout=0.3):
        self.s.settimeout(timeout)
        try:
            while True:
                b0, b1 = self._read(2)
                n = b1 & 0x7F
                if n == 126:
                    (n,) = struct.unpack("!H", self._read(2))
                elif n == 127:
                    (n,) = struct.unpack("!Q", self._read(8))
                m = json.loads(self._read(n))
                if m["type"] == "state":
                    self.state = m
                elif m["type"] == "osg_check":
                    self.last_checks = m["checks"]
                elif m["type"] == "reply":
                    self.replies[m["id"]] = m
                self.s.settimeout(0.01)
        except (socket.timeout, BlockingIOError):
            pass

    def send(self, obj):
        data = json.dumps(obj).encode()
        mask = os.urandom(4)
        hdr = bytes([0x81]) + (bytes([0x80 | len(data)]) if len(data) < 126 else bytes([0x80 | 126]) + struct.pack("!H", len(data)))
        self.s.sendall(hdr + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))

    def call(self, op, timeout=40, **kw):
        self.n += 1
        rid = self.n
        self.send(dict(type="call", id=rid, op=op, **kw))
        end = time.time() + timeout
        while time.time() < end:
            self.pump()
            if rid in self.replies:
                return self.replies.pop(rid)
        raise TimeoutError(op)

    def until(self, what, pred, timeout=30):
        end = time.time() + timeout
        while time.time() < end:
            self.pump()
            if self.state is not None and pred(self.state):
                return self.state
        raise TimeoutError(what)


def osg_unit(s, name):
    st = ((s.get("osg") or {}).get("runner") or {}).get("state") or {}
    return (st.get("units") or {}).get(name, {}).get("state")


def unit(s, name):
    sup = s.get("supervisor") or {}
    return (sup.get("units") or {}).get(name, {}).get("state")


def check(cond, what):
    print(("  ok  " if cond else "  FAIL ") + what, flush=True)
    if not cond:
        raise SystemExit(1)


def main():
    page = urllib.request.urlopen(f"http://127.0.0.1:{PORT}/").read().decode()
    check("Stretch 控制面板" in page, "GET / serves the page")
    ws = WS()
    ws.until("supervisor state", lambda s: s.get("supervisor"), 20)
    check(True, "the robot's supervisor is seen through ROS")

    r = ws.call("unit", action="start", unit="mapping")
    check(not r["ok"] and "start driver" in r["message"], f"mapping refused before the driver: {r['message']}")
    for name in ("driver", "lidar", "camera", "mapping", "navigation"):
        r = ws.call("unit", action="start", unit=name)
        check(r["ok"], f"start {name}: {r['message']}")
        ws.until(f"{name} running", lambda s: unit(s, name) == "running", 20)
    s = ws.until("map + pose in map + scan", lambda s: s.get("map") and s.get("pose") and
                 s["pose"]["frame"] == "map" and s.get("scan"), 20)
    check(len(s["scan"]) > 100, f"scan in the map frame: {len(s['scan'])} points")
    png = urllib.request.urlopen(f"http://127.0.0.1:{PORT}/api/map.png").read()
    check(png[:4] == b"\x89PNG", f"/api/map.png: {s['map']['width']}x{s['map']['height']} map, {len(png)} bytes")
    for _ in range(20):
        try:
            jpg = urllib.request.urlopen(f"http://127.0.0.1:{PORT}/api/camera.jpg").read()
            break
        except Exception:
            time.sleep(0.5)
    check(jpg[:2] == b"\xff\xd8", f"/api/camera.jpg relays the camera ({len(jpg)} bytes)")

    # ---- driver: modes and teleop
    r = ws.call("driver", action="mode_gamepad")
    check(r["ok"], f"switch to gamepad mode: {r['message']}")
    p0 = ws.until("gamepad mode", lambda s: s["robot"]["mode"] == "gamepad")["pose"]
    for _ in range(10):
        ws.send({"type": "teleop", "vx": 0.3, "wz": 0.0}); ws.pump(0.1)
    p1 = ws.state["pose"]
    check(math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"]) < 0.02, "no motion from the panel in gamepad mode")
    r = ws.call("driver", action="mode_navigation")
    ws.until("navigation mode", lambda s: s["robot"]["mode"] == "navigation")
    p0 = ws.state["pose"]
    for _ in range(15):
        ws.send({"type": "teleop", "vx": 0.3, "wz": 0.0}); ws.pump(0.1)
    ws.send({"type": "teleop_stop"})
    time.sleep(0.6); ws.pump()
    p1 = ws.state["pose"]
    moved = math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"])
    check(0.2 < moved < 0.8, f"teleop drove {moved:.2f} m in 1.5 s at 0.3 m/s")
    time.sleep(0.8); ws.pump()
    p2 = ws.state["pose"]
    check(math.hypot(p2["x"] - p1["x"], p2["y"] - p1["y"]) < 0.02, "and stopped when let go")

    # ---- Nav2
    r = ws.call("nav_goal", x=4.0, y=2.0, yaw=0.0)
    check(r["ok"], f"nav goal: {r['message']}")
    s = ws.until("arrived", lambda s: s["nav"]["status"] == "arrived", 60)
    check(math.hypot(s["pose"]["x"] - 4.0, s["pose"]["y"] - 2.0) < 0.3, f"arrived at ({s['pose']['x']}, {s['pose']['y']})")
    ws.call("nav_goal", x=1.0, y=5.5, yaw=0.0)
    ws.until("driving", lambda s: s["nav"]["status"] == "driving", 10)
    ws.send({"type": "teleop", "vx": 0.0, "wz": 0.5})
    s = ws.until("teleop takes over", lambda s: s["nav"]["status"] in ("canceled", "canceling"), 10)
    check(True, "a stick touch cancels the Nav2 goal")
    ws.send({"type": "teleop_stop"})

    # ---- maps
    r = ws.call("save_map", name="e2e lab", timeout=120)
    check(r["ok"], f"save map: {r['message']}")
    for ext in (".posegraph", ".pgm", ".yaml"):
        check(os.path.exists(f"{MAPS}/e2e_lab{ext}"), f"  wrote e2e_lab{ext}")
    ws.until("map listed", lambda s: any(m["name"] == "e2e_lab" for m in s["supervisor"]["maps"]))
    r = ws.call("unit", action="start", unit="localization", args={"map": "e2e_lab"})
    check(not r["ok"] and "stop mapping" in r["message"], f"localization refused while mapping: {r['message']}")
    ws.call("unit", action="stop", unit="navigation")
    ws.call("unit", action="stop", unit="mapping")
    ws.until("mapping stopped", lambda s: unit(s, "mapping") == "stopped" and unit(s, "navigation") == "stopped", 30)
    r = ws.call("unit", action="start", unit="localization", args={"map": "e2e_lab"})
    check(r["ok"], f"localization on e2e_lab: {r['message']}")
    ws.until("localization running", lambda s: unit(s, "localization") == "running", 20)
    r = ws.call("set_pose", x=6.0, y=1.5, yaw=1.57)
    s = ws.until("relocalised", lambda s: s.get("pose") and s["pose"]["frame"] == "map"
                 and math.hypot(s["pose"]["x"] - 6.0, s["pose"]["y"] - 1.5) < 0.1, 10)
    check(abs(s["pose"]["yaw"] - 1.57) < 0.1, f"set pose -> robot at ({s['pose']['x']}, {s['pose']['y']}, {s['pose']['yaw']})")

    # ---- OSG: start, check, map pass, search pass with inspection
    r = ws.call("unit", action="start", unit="navigation")
    ws.until("navigation back", lambda s: unit(s, "navigation") == "running", 20)
    s = ws.until("osg runner", lambda s: (s.get("osg") or {}).get("runner", {}).get("alive"), 15)
    check(True, "the OSG runner answers through the gateway")
    r = ws.call("osg_check", preset="stretch3_extinguisher")
    check(not r["ok"], f"pre-flight before the models: {r['message']}")
    checks = ws.last_checks
    check(any(c["key"].startswith("perception") and c["ok"] is False for c in checks), "  it names the perception servers")
    r = ws.call("osg", action="start", unit="perception")
    check(r["ok"], f"start perception: {r['message']}")
    ws.until("perception running", lambda s: osg_unit(s, "perception") == "running", 20)
    time.sleep(1.0)
    r = ws.call("osg_check", preset="stretch3_map")
    bad = [c["label"] + ": " + c["detail"] for c in ws.last_checks if c["ok"] is False and c["hard"]]
    check(r["ok"], f"pre-flight passes: {r['message']} {bad}")
    r = ws.call("osg", action="start", unit="map_run",
                args={"preset": "stretch3_map", "map_tag": "demo", "target": "chair; rm -rf /", "max_steps": "20"})
    check(not r["ok"], f"a target with shell syntax is refused: {r['message']}")
    r = ws.call("osg", action="start", unit="search_run",
                args={"preset": "stretch3_extinguisher", "map_tag": "nope", "target": "fire extinguisher #1"})
    check(not r["ok"] and "no saved map" in r["message"], f"search on a map that does not exist: {r['message']}")
    r = ws.call("osg", action="start", unit="map_run",
                args={"preset": "stretch3_extinguisher", "map_tag": "demo", "target": "fire extinguisher", "max_steps": "20"})
    check(r["ok"], f"map pass: {r['message']}")
    s = ws.until("graph", lambda s: ((s["osg"].get("graph") or {}).get("objects") or []), 20)
    check(True, f"scene graph on the page: {[o.get('name') or o['label'] for o in s['osg']['graph']['objects']]}")
    s = ws.until("map pass done", lambda s: osg_unit(s, "map_run") == "stopped", 60)
    u = s["osg"]["runner"]["state"]["units"]["map_run"]
    check(u["exit_code"] == 0 and any(m["name"] == "demo" for m in s["osg"]["runner"]["state"]["maps"]),
          f"map pass finished and saved 'demo' (log: {u['log'][-1]})")
    r = ws.call("osg", action="start", unit="search_run",
                args={"preset": "stretch3_extinguisher", "map_tag": "demo", "target": "fire extinguisher #1", "max_steps": "20"})
    check(r["ok"], f"search pass for #1: {r['message']}")
    s = ws.until("verdict", lambda s: ((s["osg"].get("mission") or {}).get("stage") == "verdict"), 60)
    m = s["osg"]["mission"]
    check(m["condition"] == "bad" and m["target"] == "fire extinguisher #1", f"verdict on the page: {m['condition']} {m['confidence']}")
    stages = [e["stage"] for e in s["osg"]["events"]]
    check(stages[:1] == ["searching"] and "inspecting" in stages, f"mission events: {stages}")
    png = urllib.request.urlopen(f"http://127.0.0.1:{PORT}/api/osg_view.png").read()
    check(png[:4] == b"\x89PNG", f"the inspected view is served ({len(png)} bytes)")
    ws.until("search pass done", lambda s: osg_unit(s, "search_run") == "stopped", 30)
    r = ws.call("osg", action="stop", unit="perception")
    ws.until("perception stopped", lambda s: osg_unit(s, "perception") == "stopped", 30)
    check(r["ok"], "perception stopped")

    # ---- E-stop
    r = ws.call("estop")
    check(r["ok"], f"E-stop: {r['message']}")
    ws.until("runstopped", lambda s: s["robot"]["runstopped"] is True)
    p0 = ws.state["pose"]
    for _ in range(10):
        ws.send({"type": "teleop", "vx": 0.3, "wz": 0.0}); ws.pump(0.1)
    ws.send({"type": "teleop_stop"})
    p1 = ws.state["pose"]
    check(math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"]) < 0.02, "no motion while runstopped")
    r = ws.call("runstop", on=False)
    ws.until("released", lambda s: s["robot"]["runstopped"] is False)
    check(r["ok"], "runstop released")

    r = ws.call("stop_all")
    s = ws.until("all stopped", lambda s: all(u["state"] in ("stopped",) for u in s["supervisor"]["units"].values()), 60)
    check(True, "stop all: every unit stopped")
    print("E2E PASSED")


if __name__ == "__main__":
    main()
