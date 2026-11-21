#!/usr/bin/env python3
"""The robot half of the control panel: start and stop the Stretch's units.

Runs ON THE STRETCH, because that is where the driver, the lidar and the
camera have to be launched -- they need the robot's USB devices and
hello-robot's install. ROS 2 cannot start a process on another machine, so
this is the one thing the Thor's gateway cannot do itself; everything else
(cmd_vel, the driver's services, Nav2, /initialpose) the gateway does over DDS
directly.

    python3 control_panel/supervisor/supervisor.py [--units units.yaml]

Talks over three ROS topics, JSON in std_msgs/String so no interface package
has to be built on either machine:

    /panel/supervisor/request  {"id", "op": start|stop|restart|stop_all|save_map|free_robot, "unit", "args"}
    /panel/supervisor/reply    {"id", "ok", "message"}
    /panel/supervisor/state    {"host", "units": {name: {...}}, "maps": [...]}   (latched, 1 Hz)

Each unit's output goes to <log_dir>/<unit>.log; the last lines ride along in
the state so the panel can show why something died.
"""
from __future__ import annotations

import argparse
import collections
import fcntl
import json
import os
import re
import shlex
import signal
import socket
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional

import yaml

HERE = Path(__file__).resolve().parent
WS_DEFAULT = HERE.parents[1]  # this stretch_main checkout
NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
# Free text an operator types (a search target: "fire extinguisher #1", "椅子"):
# word characters in any script, spaces, # . -  -- nothing a shell or Hydra
# would read as syntax. It is still quoted for both when substituted.
TEXT_RE = re.compile(r"^[\w #.\-]{1,60}$")
RUNNING_AFTER_S = 2.0          # alive this long after start counts as running
CHECK_TIMEOUT_S = 20.0         # a unit's `check` (ros2 node list is slow) before giving up on it
# The rtabmap unit's working database in maps_dir (units.yaml), which
# save_map copies out under the map's name.
RTABMAP_DB = "rtabmap_work.db"
STOP_GRACE_S = (8.0, 4.0)      # SIGINT, then SIGTERM, then SIGKILL


def expand(text: str) -> str:
    """$VAR / ${VAR:-default} / ~ in a config path."""
    def sub(m):
        name, default = m.group(1), m.group(2)
        return os.environ.get(name) or (default if default is not None else "")
    text = re.sub(r"\$\{([A-Za-z_][A-Za-z0-9_]*)(?::-([^}]*))?\}", sub, str(text))
    return os.path.expanduser(os.path.expandvars(text))


def single_instance(path: Path):
    """An exclusive lock on `path`, or None if another instance holds it.

    Two supervisors on the same robot both answer /panel/supervisor/request
    and both publish the latched state, so the panel shows one of them at
    random while a start button launches a driver in each -- two
    stretch_drivers on the same USB, invisible in the UI. The operator who
    runs the supervisor by hand after installing the systemd service
    (DEPLOY.md) is one keystroke from exactly that, so take a lock and say so.
    The returned file must stay referenced for as long as the lock is wanted.
    """
    lock = open(path, "a+")           # not "w": a refused instance must not
    try:                              # truncate the pid of the one holding it
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        lock.close()
        return None
    lock.seek(0)
    lock.truncate()
    lock.write(f"{os.getpid()}\n")
    lock.flush()
    return lock


@dataclass
class Unit:
    name: str
    cmd: str
    label: str = ""
    help: str = ""
    requires: List[str] = field(default_factory=list)
    conflicts: List[str] = field(default_factory=list)
    check: str = ""
    args: List[str] = field(default_factory=list)
    arg_specs: Dict[str, dict] = field(default_factory=dict)
    # runtime
    proc: Optional[subprocess.Popen] = None
    state: str = "stopped"          # stopped | starting | running | stopping | failed
    since: float = 0.0
    exit_code: Optional[int] = None
    started_with: Dict[str, str] = field(default_factory=dict)
    tail: collections.deque = field(default_factory=lambda: collections.deque(maxlen=40))
    stop_requested: bool = False


class Supervisor:
    """Process bookkeeping, free of ROS so it is testable on its own."""

    def __init__(self, config: dict, ws: Path = WS_DEFAULT, log_dir: Optional[Path] = None,
                 on_change: Optional[Callable[[], None]] = None):
        self.ws = Path(ws)
        # Only {ws} is substituted in these: the setup is bash, full of ${...}.
        self.setup = str(config.get("setup", "")).replace("{ws}", str(self.ws))
        self.maps_dir = Path(expand(str(config.get("maps_dir", "~/stretch_maps")).replace("{ws}", str(self.ws))))
        self.slam_params = expand(str(config.get("slam_params", "")).replace("{ws}", str(self.ws)))
        self.log_dir = Path(log_dir or expand("${PANEL_LOG_DIR:-~/.ros/panel_logs}"))
        self.stop_order = list(config.get("stop_order", []))
        # What counts as a saved map in maps_dir: slam_toolbox pose graphs on
        # the robot, OSG scene-graph maps (*.json) on the Thor.
        self.maps_ext = str(config.get("maps_ext", ".posegraph"))
        self.units: Dict[str, Unit] = {}
        for name, spec in (config.get("units") or {}).items():
            specs = self._arg_specs(spec.get("args"))
            self.units[name] = Unit(name=name, cmd=str(spec["cmd"]), label=str(spec.get("label", name)),
                                    help=str(spec.get("help", "")), requires=list(spec.get("requires", [])),
                                    conflicts=list(spec.get("conflicts", [])), check=str(spec.get("check", "")),
                                    args=list(specs), arg_specs=specs)
        self.on_change = on_change or (lambda: None)
        self._lock = threading.RLock()
        self.maps_dir.mkdir(parents=True, exist_ok=True)
        self.log_dir.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def _arg_specs(raw) -> Dict[str, dict]:
        """`args: [map]` (each a file-safe name) or `args: {target: text,
        steps: {type: int, min: 1, max: 2000}, preset: {type: choice, choices: [..]},
        map: {type: name, exists: true}}`. `map` as a plain name must exist."""
        out: Dict[str, dict] = {}
        if isinstance(raw, dict):
            for key, spec in raw.items():
                spec = {"type": spec} if isinstance(spec, str) else dict(spec or {})
                spec.setdefault("type", "name")
                out[str(key)] = spec
        else:
            for key in raw or []:
                out[str(key)] = {"type": "name", "exists": key == "map"}
        return out

    def _arg(self, unit: Unit, key: str, value: str) -> (Optional[str], str):
        """(the value as it goes into the command, or None) and why not."""
        spec = unit.arg_specs[key]
        kind = spec.get("type", "name")
        if value == "" and "default" in spec:
            value = str(spec["default"])
        if kind == "name":
            if not NAME_RE.match(value):
                return None, f"{unit.name} needs a {key} (letters, digits, - and _)"
            if spec.get("exists") and not (self.maps_dir / f"{value}{self.maps_ext}").exists():
                return None, f"no saved map {value!r} in {self.maps_dir}"
            return value, ""
        if kind == "text":
            if not TEXT_RE.match(value):
                return None, f"{unit.name}: {key} may be letters, digits, spaces, # . - (up to 60)"
            return shlex.quote(f"'{value}'"), ""   # a quoted Hydra string, quoted for bash
        if kind == "int":
            try:
                n = int(value)
            except ValueError:
                return None, f"{unit.name}: {key} must be a whole number"
            if not int(spec.get("min", -10**9)) <= n <= int(spec.get("max", 10**9)):
                return None, f"{unit.name}: {key} must be {spec.get('min')}..{spec.get('max')}"
            return str(n), ""
        if kind == "choice":
            if value not in [str(c) for c in spec.get("choices", [])]:
                return None, f"{unit.name}: {key} must be one of {spec.get('choices')}"
            return value, ""
        return None, f"{unit.name}: unknown argument type {kind!r}"

    # ---------------------------------------------------------- queries
    def is_up(self, name: str) -> bool:
        unit = self.units.get(name)
        return unit is not None and unit.state in ("starting", "running")

    def maps(self) -> List[dict]:
        out = []
        for pg in sorted(self.maps_dir.glob(f"*{self.maps_ext}")):
            name = pg.name[: -len(self.maps_ext)]
            out.append({"name": name, "mtime": pg.stat().st_mtime,
                        "image": (self.maps_dir / f"{name}.pgm").exists()})
        return sorted(out, key=lambda m: -m["mtime"])

    def snapshot(self) -> dict:
        with self._lock:
            units = {}
            for name, u in self.units.items():
                units[name] = {"label": u.label, "help": u.help, "state": u.state, "since": u.since,
                               "pid": u.proc.pid if u.proc and u.state != "stopped" else None,
                               "exit_code": u.exit_code, "args": u.started_with, "needs": u.args,
                               "arg_specs": u.arg_specs,
                               "requires": u.requires, "conflicts": u.conflicts,
                               "log": list(u.tail)[-8:]}
        return {"host": socket.gethostname(), "time": time.time(), "units": units,
                "maps": self.maps(), "maps_dir": str(self.maps_dir)}

    def already_running(self, unit: Unit, run=subprocess.run) -> (bool, str):
        """A unit's `check`: is an equivalent of it ALREADY up, started by
        something this supervisor cannot see?

        The process bookkeeping only knows what this instance launched. A
        supervisor that was SIGKILLed (or OOM-killed) leaves its units running
        -- they are in their own process groups -- and the fresh one shows
        them all stopped, so the next tap on a start button launches a second
        driver on the same USB, or a second Nav2 on the same topics. The same
        goes for a unit someone launched by hand, or from docker/deploy.

        Exit 0 from the check means "already up", and the start is refused
        with whatever the check printed. It FAILS OPEN: a check that errors or
        hangs must not be able to make the panel unable to start anything.
        """
        if not unit.check:
            return False, ""
        script = f"{self.setup}\n{unit.check}\n"
        try:
            r = run(["bash", "-c", script], capture_output=True, text=True, timeout=CHECK_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            return False, f"[supervisor] the {unit.name} check timed out; starting anyway"
        if r.returncode != 0:
            return False, ""
        found = " ".join((r.stdout or r.stderr).split())[:200] or "something is already there"
        return True, f"{unit.name} is already running outside the panel: {found}"

    # --------------------------------------------------------- commands
    def start(self, name: str, args: Optional[dict] = None) -> (bool, str):
        with self._lock:
            unit = self.units.get(name)
            if unit is None:
                return False, f"no unit {name!r}"
            if unit.state in ("starting", "running"):
                return True, f"{name} is already running"
            if unit.state == "stopping":
                return False, f"{name} is still stopping"
            for req in unit.requires:
                options = req.split("|")
                if not any(self.is_up(o) for o in options):
                    return False, f"start {' or '.join(options)} first"
            for other in unit.conflicts:
                if self.is_up(other):
                    return False, f"stop {other} first"
            args = {k: str(v).strip() for k, v in (args or {}).items() if k in unit.arg_specs}
            subst = {}
            for key in unit.args:
                value, why = self._arg(unit, key, args.get(key, ""))
                if value is None:
                    return False, why
                subst[key] = value
                if args.get(key, "") == "" and "default" in unit.arg_specs[key]:
                    args[key] = str(unit.arg_specs[key]["default"])
            fmt = dict(subst, ws=str(self.ws), maps_dir=str(self.maps_dir), slam_params=self.slam_params)
            try:
                cmd = unit.cmd.format(**fmt)
            except KeyError as e:
                return False, f"{name}: missing argument {e}"
            # Reserve the unit before running the check, which takes seconds
            # and must not hold the lock: `starting` with no process yet, so a
            # second tap gets "already running" rather than a second launch.
            unit.proc, unit.exit_code = None, None
            unit.state, unit.since = "starting", time.time()
            unit.started_with, unit.stop_requested = args, False
            unit.tail.clear()
            unit.tail.append(f"$ {cmd}")
        self.on_change()

        busy, why = self.already_running(unit)
        with self._lock:
            if why and not busy:
                unit.tail.append(why)          # a check that timed out; start anyway
            if busy or unit.stop_requested:
                unit.state, unit.since = "stopped", time.time()
                unit.tail.append(f"[supervisor] not started: {why or 'stopped while checking'}")
                self.on_change()
                return False, why or f"{name} was stopped before it started"
            log = open(self.log_dir / f"{name}.log", "wb")
            script = f"{self.setup}\nexec {cmd}\n"
            unit.proc = subprocess.Popen(["bash", "-c", script], stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                         start_new_session=True)
            unit.since = time.time()
            threading.Thread(target=self._pump, args=(unit, unit.proc, log), daemon=True).start()
        self.on_change()
        return True, f"starting {name}"

    def stop(self, name: str) -> (bool, str):
        with self._lock:
            unit = self.units.get(name)
            if unit is None:
                return False, f"no unit {name!r}"
            if unit.state == "starting" and unit.proc is None:
                unit.stop_requested = True     # still in its check; start() will drop it
                return True, f"{name} is still being checked; it will not be started"
            if unit.state not in ("starting", "running") or unit.proc is None:
                if unit.state == "failed":
                    unit.state = "stopped"
                    self.on_change()
                return True, f"{name} is not running"
            dependants = [u.name for u in self.units.values()
                          if u.name != name and self.is_up(u.name)
                          and any(name in r.split("|") and not any(self.is_up(o) for o in r.split("|") if o != name)
                                  for r in u.requires)]
            unit.state, unit.stop_requested = "stopping", True
            proc = unit.proc
        threading.Thread(target=self._kill, args=(proc,), daemon=True).start()
        self.on_change()
        note = f" ({', '.join(dependants)} still running without it)" if dependants else ""
        return True, f"stopping {name}{note}"

    def stop_all(self, wait: bool = False) -> None:
        order = [n for n in self.stop_order if n in self.units] + \
                [n for n in self.units if n not in self.stop_order]
        for name in order:
            if self.is_up(name):
                self.stop(name)
                if wait:
                    self.wait_stopped(name, sum(STOP_GRACE_S) + 2)

    def wait_stopped(self, name: str, timeout: float) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            if not self.is_up(name) and self.units[name].state != "stopping":
                return True
            time.sleep(0.05)
        return False

    def tick(self) -> None:
        """Promote starting -> running once a unit has stayed up a moment, and
        notice a unit whose leader died while children of it live on (the pump
        only sees the end of the output, which those children hold open)."""
        changed = False
        with self._lock:
            for u in self.units.values():
                if u.proc is not None and u.state in ("starting", "running") and u.proc.poll() is not None:
                    u.exit_code = u.proc.returncode
                    u.state = "stopped" if u.exit_code == 0 else "failed"
                    u.since = time.time()
                    u.tail.append(f"[supervisor] exited with {u.exit_code}; stopping what it left behind")
                    threading.Thread(target=self._kill, args=(u.proc,), daemon=True).start()
                    changed = True
                elif (u.state == "starting" and u.proc is not None
                      and time.time() - u.since >= RUNNING_AFTER_S):
                    u.state = "running"
                    changed = True
        if changed:
            self.on_change()

    # ---------------------------------------------------------- helpers
    def _pump(self, unit: Unit, proc: subprocess.Popen, log) -> None:
        with log:
            for raw in iter(proc.stdout.readline, b""):
                log.write(raw)
                log.flush()
                line = raw.decode("utf-8", "replace").rstrip()
                if line:
                    unit.tail.append(line[-300:])
        code = proc.wait()
        with self._lock:
            if unit.proc is proc:
                unit.exit_code = code
                unit.state = "stopped" if (unit.stop_requested or code == 0) else "failed"
                unit.since = time.time()
        self.on_change()

    @staticmethod
    def _group_alive(proc: subprocess.Popen) -> bool:
        proc.poll()                      # reap the leader, so a zombie does not count
        try:
            os.killpg(proc.pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    @classmethod
    def _kill(cls, proc: subprocess.Popen) -> None:
        """Ctrl-C, then SIGTERM, then SIGKILL to the unit's whole process group,
        until nothing in it is left. Waiting on the leader alone is not enough:
        a launch file that exits on SIGINT can leave a node behind that ignored
        it (and that node keeps the device, or the unit's output pipe, open)."""
        for sig, grace in ((signal.SIGINT, STOP_GRACE_S[0]), (signal.SIGTERM, STOP_GRACE_S[1]), (signal.SIGKILL, 2.0)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                return
            end = time.time() + grace
            while time.time() < end:
                if not cls._group_alive(proc):
                    return
                time.sleep(0.1)

    # ----------------------------------------------------------- maps
    def save_map(self, name: str, run=subprocess.run) -> (bool, str):
        """The running SLAM's own map -- slam_toolbox's pose graph (what
        localization loads) or RTAB-Map's database -- plus a .pgm/.yaml image of
        it (for looking at, or for AMCL), all into maps_dir."""
        if not NAME_RE.match(name or ""):
            return False, "map name: letters, digits, - and _ only"
        base = self.maps_dir / name
        image = ("image", f"ros2 run nav2_map_server map_saver_cli -f {base} "
                          f"--ros-args -p map_subscribe_transient_local:=true")
        if self.is_up("mapping"):
            steps = [("pose graph", f"ros2 service call /slam_toolbox/serialize_map "
                                    f"slam_toolbox/srv/SerializePoseGraph \"{{filename: '{base}'}}\""), image]
            made = base.with_suffix(".posegraph")
        elif self.is_up("rtabmap"):
            # backup: RTAB-Map writes its memory to the database, so the copy is whole.
            steps = [("database", f"ros2 service call /rtabmap/backup std_srvs/srv/Empty && "
                                  f"cp {self.maps_dir / RTABMAP_DB} {base}.db"), image]
            made = base.with_suffix(".db")
        else:
            return False, "start mapping (slam_toolbox or RTAB-Map) first: only a mapping session can be saved"
        for what, cmd in steps:
            r = run(["bash", "-c", f"{self.setup}\n{cmd}\n"], capture_output=True, text=True, timeout=60)
            if r.returncode != 0:
                return False, f"saving the {what} failed: {(r.stdout + r.stderr).strip()[-300:]}"
        if not made.exists():
            return False, f"the map was not written: no {made}"
        self.on_change()
        return True, f"saved map {name} to {self.maps_dir}"

    # ---------------------------------------------------------- robot
    def free_robot(self, run=subprocess.run) -> (bool, str):
        """hello-robot's stretch_free_robot_process.py: kill whatever holds the
        Stretch (the login-time gamepad teleop, a driver left in a terminal),
        the "Another process is already using Stretch" a driver start dies of.
        Our own driver unit goes first: the script would kill its stretch_driver
        and leave the launch -- and the unit's "running" -- behind."""
        if self.is_up("driver"):
            self.stop("driver")
            self.wait_stopped("driver", sum(STOP_GRACE_S) + 2)
        r = run(["bash", "-c", f"{self.setup}\nstretch_free_robot_process.py\n"],
                capture_output=True, text=True, timeout=30)
        out = (r.stdout + r.stderr).strip()[-300:]
        if r.returncode != 0:
            return False, f"stretch_free_robot_process.py failed: {out}"
        return True, f"robot freed, start the driver again{': ' + out if out else ''}"

    def handle(self, req: dict) -> (bool, str):
        op = req.get("op")
        if op == "start":
            return self.start(str(req.get("unit", "")), req.get("args") or {})
        if op == "stop":
            return self.stop(str(req.get("unit", "")))
        if op == "restart":
            name = str(req.get("unit", ""))
            unit = self.units.get(name)
            args = req.get("args") or (unit.started_with if unit else {})
            self.stop(name)
            self.wait_stopped(name, sum(STOP_GRACE_S) + 2)
            return self.start(name, args)
        if op == "stop_all":
            threading.Thread(target=self.stop_all, kwargs={"wait": True}, daemon=True).start()
            return True, "stopping everything"
        if op == "save_map":
            return self.save_map(str(req.get("name", "")))
        if op == "free_robot":
            return self.free_robot()
        return False, f"unknown op {op!r}"


# ------------------------------------------------------------------- ROS
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--units", default=str(HERE / "units.yaml"))
    ap.add_argument("--ws", default=str(WS_DEFAULT), help="stretch_main checkout ({ws})")
    ap.add_argument("--autostart", default=os.environ.get("PANEL_AUTOSTART", ""),
                    help="comma-separated units to start at once, e.g. driver,lidar")
    opts = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    config = yaml.safe_load(Path(opts.units).read_text())
    dirty = threading.Event()
    sup = Supervisor(config, ws=Path(opts.ws), on_change=dirty.set)
    lock = single_instance(sup.log_dir / "supervisor.lock")
    if lock is None:
        raise SystemExit(f"another panel supervisor already holds {sup.log_dir / 'supervisor.lock'} "
                         "(systemctl --user status stretch-panel-supervisor); "
                         "two of them would start two of every unit")
    rclpy.init()
    node = Node("panel_supervisor")
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state_pub = node.create_publisher(String, "/panel/supervisor/state", latched)
    reply_pub = node.create_publisher(String, "/panel/supervisor/reply", 20)

    def publish_state():
        state_pub.publish(String(data=json.dumps(sup.snapshot())))

    def on_request(msg: String):
        try:
            req = json.loads(msg.data)
        except ValueError:
            return

        def work():
            try:
                ok, message = sup.handle(req)
            except Exception as e:  # noqa: BLE001 -- the panel shows it
                ok, message = False, f"{type(e).__name__}: {e}"
            node.get_logger().info(f"{req.get('op')} {req.get('unit') or req.get('name') or ''}: {message}")
            reply_pub.publish(String(data=json.dumps({"id": req.get("id"), "ok": ok, "message": message})))
            dirty.set()
        threading.Thread(target=work, daemon=True).start()   # save_map / restart block

    node.create_subscription(String, "/panel/supervisor/request", on_request, 20)
    last = [0.0]

    def timer():
        sup.tick()
        if dirty.is_set() or time.time() - last[0] > 1.0:
            dirty.clear()
            last[0] = time.time()
            publish_state()
    node.create_timer(0.2, timer)

    for name in [n.strip() for n in opts.autostart.split(",") if n.strip()]:
        ok, message = sup.start(name)
        node.get_logger().info(f"autostart {name}: {message}")
        if ok:
            time.sleep(RUNNING_AFTER_S + 0.5)
            sup.tick()

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node.get_logger().info(f"panel supervisor up: {len(sup.units)} units, maps in {sup.maps_dir}, "
                           f"logs in {sup.log_dir}")
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    while not stop.wait(0.5):      # a timed wait, so the signal handler gets to run
        pass
    node.get_logger().info("stopping every unit before exiting")
    sup.stop_all(wait=True)
    publish_state()
    time.sleep(0.3)
    # In this order: shutting rclpy down under a spinning executor aborts the
    # process ("terminate called without an active exception").
    executor.shutdown(timeout_sec=2.0)
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
