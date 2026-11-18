#!/usr/bin/env python3
"""The panel's hands inside the Thor's OSG container: start the perception
servers and the map / search passes (`scripts/run_robot.py`) on request.

    python3 /panel/osg_runner/runner.py [--ws /workspace] [--port 18790]

The OSG container has no ROS on purpose (its bridge is a separate container,
OSG docs/THOR.md), so this answers plain HTTP instead of ROS topics -- on
127.0.0.1 only: the panel's gateway shares the host network and is the one
caller; nothing on the network reaches it.

    GET  /state      units, their last log lines, the saved OSG maps
    POST /request    {"op": start|stop|stop_all, "unit", "args"} -> {"ok", "message"}

The bookkeeping is the robot supervisor's (`supervisor/supervisor.py`), with
osg_units.yaml for units.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "supervisor"))
from supervisor import Supervisor  # noqa: E402


def make_handler(sup: Supervisor):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, obj) -> None:
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):  # noqa: N802
            if self.path.split("?")[0] == "/state":
                snap = sup.snapshot()
                snap["nvidia_api_key"] = bool(os.environ.get("NVIDIA_API_KEY"))
                self._send(200, snap)
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):  # noqa: N802
            if self.path != "/request":
                self._send(404, {"error": "not found"})
                return
            try:
                n = int(self.headers.get("Content-Length", "0"))
                req = json.loads(self.rfile.read(min(n, 65536)) or b"{}")
            except ValueError:
                self._send(400, {"ok": False, "message": "bad JSON"})
                return
            try:
                ok, message = sup.handle(req)
            except Exception as e:  # noqa: BLE001 -- the panel shows it
                ok, message = False, f"{type(e).__name__}: {e}"
            print(f"[osg_runner] {req.get('op')} {req.get('unit', '')}: {message}", flush=True)
            self._send(200, {"ok": ok, "message": message})

        def log_message(self, *_):
            pass

    return Handler


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--units", default=str(HERE / "osg_units.yaml"))
    ap.add_argument("--ws", default=os.environ.get("OSG_WS", "/workspace"), help="the OSG directory")
    ap.add_argument("--port", type=int, default=int(os.environ.get("OSG_RUNNER_PORT", "18790")))
    ap.add_argument("--host", default="127.0.0.1")
    opts = ap.parse_args()

    config = yaml.safe_load(Path(opts.units).read_text())
    log_dir = Path(os.environ.get("PANEL_LOG_DIR", str(Path(opts.ws) / "outputs" / "panel_logs")))
    sup = Supervisor(config, ws=Path(opts.ws), log_dir=log_dir)
    server = ThreadingHTTPServer((opts.host, opts.port), make_handler(sup))

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())

    def ticker():
        while not stop.is_set():
            sup.tick()
            time.sleep(0.2)
    threading.Thread(target=ticker, daemon=True).start()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[osg_runner] http://{opts.host}:{opts.port}  units {list(sup.units)}  ws {opts.ws}  "
          f"maps {sup.maps_dir}  logs {log_dir}", flush=True)
    while not stop.wait(0.5):
        pass
    print("[osg_runner] stopping every unit", flush=True)
    sup.stop_all(wait=True)
    server.shutdown()


if __name__ == "__main__":
    main()
