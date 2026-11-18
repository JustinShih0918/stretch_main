"""The control panel's gateway: a web page for phones and tablets, and the
one process that turns what the page asks for into ROS 2 on the robot.

    cd control_panel && python3 -m gateway [--port 8080]

Runs on the Thor (docker/thor/compose.yaml, service `panel`), on the same
CycloneDDS settings as everything else that talks to the Stretch. Open
http://<thor>:8080 from anything on the network.

One WebSocket per open page. Server -> page: `state` five times a second
(units, pose, map version, scan, Nav2), `reply` for each call. Page -> server:
`teleop` {vx, wz} while a finger is on the stick, and `call` {id, op, ...}.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import signal
import time
from pathlib import Path

from . import ros_io
from .core import clean_name
from .webserver import WebServer, WsClient

STATIC = Path(__file__).resolve().parent / "static"
STATE_HZ = 5.0
CALL_TIMEOUT_S = {"save_map": 120.0, "unit": 30.0}


def build(ros: "ros_io.RosIO") -> WebServer:
    def map_png(_q):
        png = ros.map_png
        return None if png is None else (200, "image/png", png)

    def camera_jpg(_q):
        ros.want_camera()
        jpg = ros.camera_jpeg
        return None if jpg is None else (200, "image/jpeg", jpg)

    def health(_q):
        body = json.dumps({"ok": True, "node": ros.get_name(),
                           "domain": os.environ.get("ROS_DOMAIN_ID", "0"),
                           "rmw": os.environ.get("RMW_IMPLEMENTATION", "")}).encode()
        return 200, "application/json", body

    async def call(client: WsClient, msg: dict) -> None:
        op = msg.get("op")
        rid = msg.get("id")
        try:
            ok, message = await dispatch(op, msg)
        except asyncio.TimeoutError:
            ok, message = False, f"{op}: no answer in time"
        except Exception as e:  # noqa: BLE001 -- the page shows it
            ok, message = False, f"{op}: {type(e).__name__}: {e}"
        await client.send({"type": "reply", "id": rid, "op": op, "ok": ok, "message": message})

    async def wait(fut, op):
        return await asyncio.wait_for(asyncio.wrap_future(fut), CALL_TIMEOUT_S.get(op, 10.0))

    async def dispatch(op: str, m: dict):
        if op == "unit":
            action = m.get("action")
            if action not in ("start", "stop", "restart"):
                return False, f"unit action {action!r}?"
            return await wait(ros.supervisor_async(
                {"op": action, "unit": str(m.get("unit", "")), "args": m.get("args") or {}}), "unit")
        if op == "stop_all":
            return await wait(ros.supervisor_async({"op": "stop_all"}), "unit")
        if op == "save_map":
            name = clean_name(m.get("name", ""))
            if not name:
                return False, "give the map a name"
            return await wait(ros.supervisor_async({"op": "save_map", "name": name}), "save_map")
        if op == "driver":
            return await wait(ros.driver_async(str(m.get("action", ""))), op)
        if op == "runstop":
            return await wait(ros.runstop_async(bool(m.get("on"))), op)
        if op == "estop":
            return await wait(ros.estop(), op)
        if op == "nav_goal":
            return await wait(ros.nav_goal_async(float(m["x"]), float(m["y"]), float(m.get("yaw", 0.0))), op)
        if op == "nav_cancel":
            ros.cancel_nav()
            return True, "cancelling the goal"
        if op == "set_pose":
            ros.set_pose(float(m["x"]), float(m["y"]), float(m.get("yaw", 0.0)))
            return True, "pose sent to /initialpose"
        return False, f"unknown op {op!r}"

    async def on_message(client: WsClient, msg: dict) -> None:
        kind = msg.get("type")
        if kind == "teleop":
            try:
                ros.drive(float(msg.get("vx", 0.0)), float(msg.get("wz", 0.0)))
            except (TypeError, ValueError):
                ros.teleop.release()
        elif kind == "teleop_stop":
            ros.teleop.release()
        elif kind == "call":
            asyncio.ensure_future(call(client, msg))

    async def on_open(client: WsClient) -> None:
        await client.send(ros.snapshot())

    async def on_close(client: WsClient) -> None:
        # A page that drove and then vanished must not leave the base moving.
        if not server.clients:
            ros.teleop.release()

    server = WebServer(STATIC, routes={"/api/map.png": map_png, "/api/camera.jpg": camera_jpg,
                                       "/api/health": health},
                       on_ws_open=on_open, on_ws_message=on_message, on_ws_close=on_close)
    return server


async def run(host: str, port: int, cfg: dict) -> None:
    ros = ros_io.start(cfg)
    server = build(ros)
    listener = await server.serve(host, port)
    # rclpy's own SIGINT/SIGTERM handler only shuts ROS down; without these the
    # web server would carry on, serving a page with no robot behind it and
    # holding the port. A signal ends the whole process.
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    print(f"[panel] http://{host}:{port}  (ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}, "
          f"RMW={os.environ.get('RMW_IMPLEMENTATION', 'default')})", flush=True)
    tick = 0
    while not stop.is_set():
        t0 = time.monotonic()
        if not ros.context.ok():
            print("[panel] ROS shut down: exiting", flush=True)
            break
        if server.clients:
            state = ros.snapshot(with_scan=(tick % 2 == 0))
            await server.broadcast(state)
        tick += 1
        try:
            await asyncio.wait_for(stop.wait(), max(0.0, 1.0 / STATE_HZ - (time.monotonic() - t0)))
        except asyncio.TimeoutError:
            pass
    if ros.context.ok() and ros.teleop.active:
        ros.cmd_vel.publish(ros_io.Twist())     # a page mid-drive when we went down
    listener.close()
    ros_io.stop(ros)


def main() -> None:
    ap = argparse.ArgumentParser(description="Stretch control panel gateway")
    ap.add_argument("--host", default=os.environ.get("PANEL_HOST", "0.0.0.0"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("PANEL_PORT", "8080")))
    ap.add_argument("--max-linear", type=float, default=float(os.environ.get("PANEL_MAX_LINEAR", "0.3")))
    ap.add_argument("--max-angular", type=float, default=float(os.environ.get("PANEL_MAX_ANGULAR", "1.0")))
    ap.add_argument("--cmd-vel", default=os.environ.get("PANEL_CMD_VEL", "/stretch/cmd_vel"))
    ap.add_argument("--scan", default=os.environ.get("PANEL_SCAN", "/scan_filtered"))
    ap.add_argument("--camera", default=os.environ.get("PANEL_CAMERA", "/camera/color/image_raw/compressed"))
    ap.add_argument("--base-frame", default=os.environ.get("PANEL_BASE_FRAME", "base_link"))
    o = ap.parse_args()
    cfg = {"max_linear": o.max_linear, "max_angular": o.max_angular, "cmd_vel_topic": o.cmd_vel,
           "scan_topic": o.scan, "camera_topic": o.camera, "base_frame": o.base_frame}
    try:
        asyncio.run(run(o.host, o.port, cfg))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
