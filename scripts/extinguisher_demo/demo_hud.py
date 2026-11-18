#!/usr/bin/env python3
"""The extinguisher demo's screen: one 1920x1080 window built around Isaac's viewport.

    DISPLAY=:6 python3 scripts/extinguisher_demo/demo_hud.py

Everything is read off the /osg/* topics the pipeline publishes
(src/semantic_nav/object_scene_graph: eval/debug_stream.py, scripts/run_robot.py):

    /osg/detections        the robot's head camera with YOLOE's boxes and masks
    /osg/costmap           the pipeline's own occupancy map
    /osg/scene_graph_json  rooms -> surfaces -> objects, the robot, its trajectory
    /osg/mission           searching | checking | resumed | inspecting | verdict | not_found
    /osg/inspection_view   the close-up the VLM is judging

VIEWPORT is left as an empty frame: run_extinguisher_demo.sh's ffmpeg overlays
Isaac's third-person viewport there while it records, so the two are composed
live into one video. A new mission ("searching") clears the screen.

With several extinguishers told apart ("#1", "#2"; OSG's objects/instances.py),
the decisions about which is which are written under the scene graph as they
happen, read off the same graph JSON: an instance whose track the presence filter
stops believing ("not at its place"), one whose index moves to another track ("found N m away:
moved"), and one re-anchored on the track at its own place after its estimate
drifted toward a neighbour. The map crosses out the place it is no longer
believed at and draws the move.

System python + system ROS (rclpy, python_qt_binding), on the demo's
ROS_DOMAIN_ID.
"""
import argparse
import json
import math
import re
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from python_qt_binding.QtCore import QPointF, QRectF, Qt, QTimer
from python_qt_binding.QtGui import (QBrush, QColor, QFont, QImage, QPainter,
                                     QPainterPath, QPen, QPolygonF)
from python_qt_binding.QtWidgets import QApplication, QWidget
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

W, H = 1920, 1080
# Panels: (x, y, w, h) including a 30 px header strip.
VIEWPORT = (16, 116, 1152, 678)      # ffmpeg overlays Isaac at (16, 146) 1152x648
MAP = (1184, 116, 720, 410)
CAMERA = (1184, 538, 300, 530)
VLM = (1496, 538, 408, 530)
GRAPH = (16, 806, 1152, 262)
HEADER = 30

BG = QColor("#0e1014")
PANEL = QColor("#171a21")
EDGE = QColor("#2a2f3a")
TEXT = QColor("#e8e8e8")
DIM = QColor("#9aa0ab")
FAINT = QColor("#5c6370")
FIRE = QColor("#ff6a1a")
RED = QColor("#e5484d")
GREEN = QColor("#30a46c")
CYAN = QColor("#3ec6ff")
YELLOW = QColor("#f5c542")
BLUE = QColor("#5b9cff")
# osg InstanceRegistry's missing_p (scene_graph.presence.min_presence): below this
# the presence filter no longer believes an instance is where its track is.
MISSING_P = 0.45

STAGES = ("Explore & build scene graph", "Fire extinguisher found", "Approach",
          "VLM condition inspection", "Verdict")
# A search run (ros2.map_mode=search) starts from a saved scene graph: the
# extinguisher is known before the robot moves, so the first two stages are lookups.
SEARCH_STAGES = ("Load saved scene graph", "Extinguisher located in graph", "Navigate & re-verify",
                 "VLM condition inspection", "Verdict")
APPROACH_STATES = {"APPROACH", "VERIFYING", "GOTO_VERIFY_VIEW", "CLOSE_LOOK", "DONE"}


def font(px, bold=False, mono=False):
    f = QFont("DejaVu Sans Mono" if mono else "DejaVu Sans")
    f.setPixelSize(px)
    f.setBold(bold)
    return f


def to_qimage(msg):
    h, w = int(msg.height), int(msg.width)
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)
    if msg.encoding.lower() == "bgr8":
        buf = buf[..., ::-1]
    buf = np.ascontiguousarray(buf)
    return QImage(buf.data, w, h, w * 3, QImage.Format_RGB888).copy()


def pretty(label):
    return re.sub(r"\s+", " ", str(label).replace("_", " ")).strip()


def name(o):
    """'fire extinguisher #1' for an object the scene graph tells apart by
    instance (osg/objects/instances.py), else its label."""
    return pretty(o.get("name") or o["label"])


def short(o):
    """On the map, where two extinguishers 2 m apart are 50 px apart: "#1"."""
    if "instance" in o:
        return f"#{o['instance']}" if o.get("instance") is not None else "?"
    return name(o)


def ghost(o):
    """An instance-class track nothing is believed to be at any more and no
    index is bound to: where an extinguisher used to be. Not drawn."""
    return "instance" in o and o.get("instance") is None and float(o.get("p", 1.0)) < 0.45


class Hud(QWidget):
    def __init__(self, node, model_name):
        super().__init__()
        self.setWindowTitle("Fire extinguisher inspection")
        self.setFixedSize(W, H)
        self.model_name = model_name
        self._reset()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        volatile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(Image, "/osg/detections", self._on_camera, volatile)
        node.create_subscription(Image, "/osg/inspection_view", self._on_view, volatile)
        node.create_subscription(OccupancyGrid, "/osg/costmap", self._on_grid, latched)
        node.create_subscription(String, "/osg/scene_graph_json", self._on_graph, latched)
        node.create_subscription(String, "/osg/mission", self._on_mission, latched)
        self._node = node
        timer = QTimer(self)
        timer.timeout.connect(self._tick)
        timer.start(40)

    # ------------------------------------------------------------------ state

    def _reset(self):
        self.camera = None
        self.view = None
        self.grid = None            # (QImage, origin_x, origin_y, resolution, w, h)
        self.graph = {}
        self.mission = {}
        self.map_mode = None        # from the mission's first message; later ones omit it
        self.mission_target = None  # likewise: "fire extinguisher #1" (the verdict says "object")
        self.t0 = None
        self.t_found = None
        self.t_inspect = None
        self.t_verdict = None
        self.t_end = None
        self.inst = {}              # instance name -> {"track", "xy", "missing"}
        self.events = []            # (elapsed s, step, text, colour), oldest first
        self.moves = []             # (name, from_xy, to_xy)

    def _tick(self):
        rclpy.spin_once(self._node, timeout_sec=0.0)
        self.update()

    def _on_camera(self, msg):
        self.camera = to_qimage(msg)

    def _on_view(self, msg):
        self.view = to_qimage(msg)

    def _on_grid(self, msg):
        w, h = int(msg.info.width), int(msg.info.height)
        data = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        rgb = np.empty((h, w, 3), np.uint8)
        rgb[:] = (34, 38, 46)                               # unknown
        rgb[data == 0] = (196, 200, 208)                    # free
        rgb[data > 50] = (20, 22, 26)                       # occupied
        rgb = np.ascontiguousarray(rgb[::-1])               # row 0 = top = max y
        img = QImage(rgb.data, w, h, w * 3, QImage.Format_RGB888).copy()
        known = np.argwhere(data >= 0)
        self.grid = (img, float(msg.info.origin.position.x), float(msg.info.origin.position.y),
                     float(msg.info.resolution), w, h, known)

    def _on_graph(self, msg):
        try:
            self.graph = json.loads(msg.data)
        except ValueError:
            return
        if self.t_found is None and any(o.get("target") for o in self.graph.get("objects", [])):
            self.t_found = time.time()
        self._follow_instances()

    def _event(self, text, colour=TEXT):
        self.events.append((self._elapsed(), int(self.graph.get("step", 0) or 0), text, colour))

    def _follow_instances(self):
        """Turn the graph's instance list into the decision log: where each
        instance is first placed, when the presence filter stops believing it
        is there, and when its index is handed to another track (a move)."""
        objs = {o["id"]: o for o in self.graph.get("objects", [])}
        for inst in self.graph.get("instances") or []:
            nm = pretty(inst["name"])
            o = objs.get(inst["track_id"])
            xy = list(o["xyz"][:2]) if o else None
            p_ = float(o.get("p", 1.0)) if o else None
            prev = self.inst.get(nm)
            moved = int(inst.get("moved", 0))
            if prev is None:
                self.inst[nm] = {"track": inst["track_id"], "xy": xy, "missing": False, "moved": moved}
                where = f"at ({xy[0]:.1f}, {xy[1]:.1f})" if xy else "indexed"
                self._event(f"{nm} {where}", TEXT)
                continue
            if inst["track_id"] != prev["track"]:
                d = math.dist(prev["xy"], xy) if prev["xy"] and xy else None
                if moved > prev["moved"]:
                    if prev["xy"] and xy:
                        self.moves.append((nm, prev["xy"], xy))
                    self._event(f"{nm} found {d:.1f} m from its old place: it was moved, keeps #{inst['index']}"
                                if d is not None else f"{nm} found on a new track", GREEN)
                else:
                    # Re-anchored (osg instances.py): its estimate had wandered off
                    # its place, toward a neighbour; the track at the place is it.
                    self._event(f"{nm} is the one still at its place: its estimate had drifted"
                                + (f" {d:.1f} m" if d is not None else "") + " toward a neighbour", CYAN)
                prev.update(track=inst["track_id"], xy=xy or prev["xy"], missing=False, moved=moved)
                continue
            if xy:
                prev["xy"] = xy
            missing = p_ is not None and p_ < MISSING_P
            if missing and not prev["missing"]:
                self._event(f"{nm} is not at its place (belief {p_:.2f}): looking for it elsewhere", YELLOW)
            elif prev["missing"] and not missing:
                self._event(f"{nm} seen at its place again", TEXT)
            prev["missing"] = missing

    def _on_mission(self, msg):
        try:
            m = json.loads(msg.data)
        except ValueError:
            return
        stage = m.get("stage")
        if stage == "searching":
            self._reset()
            self.map_mode = m.get("map_mode")
            self.mission_target = m.get("target")
        if self.t0 is None:
            self.t0 = time.time()
        if stage == "checking":
            # run_robot.py's stop check: does the detector name one where it stopped?
            if self.inst:
                who = pretty(m.get("target", "the target"))
                who = f"{who[:-1].strip()} no index explains yet" if who.endswith("?") else who
                self._event(f"stopped at {who}: looking at it", YELLOW)
        elif stage == "resumed":
            self.t_inspect = None
            where = m.get("where")
            at = f" at ({where[0]:.1f}, {where[1]:.1f})" if where else ""
            self._event(f"no fire extinguisher{at} after all: struck off, searching on", YELLOW)
        elif stage == "inspecting":
            self.t_inspect = time.time()
            if self.inst:
                who = pretty(m.get("target", "the target"))
                if who.endswith("?"):
                    # run_robot.py claims it at the stop and announces it again by name.
                    self._event(f"stopped at {who[:-1].strip()} no index explains yet: checking which one",
                                YELLOW)
                else:
                    self._event(f"stopped at {who}: asking the VLM", FIRE)
        elif stage in ("verdict", "not_found"):
            self.t_verdict = self.t_end = time.time()
            if stage == "not_found" and m.get("reason"):
                self._event(str(m["reason"]), RED)
            if self.inst and stage == "verdict":
                cond = str(m.get("condition", "unknown"))
                self._event(f"{pretty(m.get('object') or m.get('target', 'target'))}: {cond.upper()}",
                            GREEN if cond == "good" else RED if cond == "bad" else YELLOW)
        self.mission = m

    # ------------------------------------------------------------------ paint

    def paintEvent(self, _event):  # noqa: N802 (Qt)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        p.fillRect(0, 0, W, H, BG)
        self._top_bar(p)
        self._stages(p)
        self._panel(p, VIEWPORT, "ISAAC SIM  ·  THIRD-PERSON VIEW")
        self._camera(p)
        self._map(p)
        self._graph_panel(p)
        self._vlm(p)
        p.end()

    def _panel(self, p, rect, title, right=None, accent=None):
        x, y, w, h = rect
        p.setPen(QPen(accent or EDGE, 2 if accent else 1))
        p.setBrush(PANEL)
        p.drawRoundedRect(QRectF(x, y, w, h), 6, 6)
        p.setPen(DIM)
        p.setFont(font(13, bold=True))
        p.drawText(QRectF(x + 12, y, w - 24, HEADER), Qt.AlignVCenter | Qt.AlignLeft, title)
        if right:
            p.setFont(font(12))
            p.drawText(QRectF(x + 12, y, w - 24, HEADER), Qt.AlignVCenter | Qt.AlignRight, right)
        return QRectF(x + 1, y + HEADER, w - 2, h - HEADER - 1)

    def _elapsed(self):
        if self.t0 is None:
            return 0.0
        return max(0.0, (self.t_end or time.time()) - self.t0)

    def _top_bar(self, p):
        p.setBrush(FIRE)
        p.setPen(Qt.NoPen)
        p.drawEllipse(QRectF(20, 18, 18, 18))
        p.setPen(TEXT)
        p.setFont(font(26, bold=True))
        p.drawText(QRectF(50, 4, 900, 40), Qt.AlignVCenter | Qt.AlignLeft, "FIRE EXTINGUISHER INSPECTION")
        target = self.mission_target
        if target and "#" in str(target):
            # A mission for one of several: say which.
            x = 50 + p.fontMetrics().horizontalAdvance("FIRE EXTINGUISHER INSPECTION") + 24
            text = f"MISSION  ·  {pretty(target)}"
            p.setFont(font(17, bold=True))
            tw = p.fontMetrics().horizontalAdvance(text) + 28
            p.setPen(QPen(FIRE, 2))
            p.setBrush(QColor(255, 106, 26, 40))
            p.drawRoundedRect(QRectF(x, 10, tw, 30), 15, 15)
            p.setPen(TEXT)
            p.drawText(QRectF(x, 10, tw, 30), Qt.AlignCenter, text)
        p.setPen(DIM)
        p.setFont(font(15))
        p.drawText(QRectF(52, 36, 1200, 22), Qt.AlignVCenter | Qt.AlignLeft,
                   "Hello Robot Stretch 3  ·  Isaac Sim hospital  ·  object scene graph search  →  "
                   "VLM condition check")
        t = self._elapsed()
        p.setPen(TEXT)
        p.setFont(font(28, mono=True))
        p.drawText(QRectF(W - 330, 4, 310, 40), Qt.AlignVCenter | Qt.AlignRight,
                   f"T+ {int(t // 60):02d}:{int(t % 60):02d}")
        g = self.graph
        p.setPen(DIM)
        p.setFont(font(14))
        status = f"step {g.get('step', 0)}  ·  {str(g.get('state', 'waiting')).lower().replace('_', ' ')}"
        p.drawText(QRectF(W - 530, 38, 510, 20), Qt.AlignVCenter | Qt.AlignRight, status)

    def _stage_index(self):
        stage = self.mission.get("stage")
        if stage == "verdict":
            return 5
        if stage == "inspecting":
            return 3
        if stage == "checking":
            return 2
        if self.t_found is None:
            return 0
        return 2 if str(self.graph.get("state", "")) in APPROACH_STATES else 1

    def _stages(self, p):
        active = self._stage_index()
        stage = self.mission.get("stage")
        labels = SEARCH_STAGES if self.map_mode == "search" else STAGES
        n = len(labels)
        x0, y, gap = 16, 64, 28
        w = (W - 32 - gap * (n - 1)) / n
        pulse = 0.55 + 0.45 * math.sin(time.time() * 5)
        for i, label in enumerate(labels):
            x = x0 + i * (w + gap)
            if stage == "not_found" and i == 1:
                fill, edge, fg, mark = QColor("#3a1d20"), RED, RED, "✕"
            elif i < active:
                fill, edge, fg, mark = QColor("#16281f"), GREEN, TEXT, "✓"
            elif i == active and i < n:
                c = QColor(FIRE)
                c.setAlphaF(pulse)
                fill, edge, fg, mark = QColor("#2e1d12"), c, TEXT, str(i + 1)
            else:
                fill, edge, fg, mark = PANEL, EDGE, FAINT, str(i + 1)
            if i == 4 and stage == "verdict":
                good = self.mission.get("condition") == "good"
                col = GREEN if good else RED if self.mission.get("condition") == "bad" else YELLOW
                fill, edge, fg = QColor(col.red(), col.green(), col.blue(), 60), col, TEXT
                mark = "✓" if good else "!"
                label = f"Verdict: {str(self.mission.get('condition', '?')).upper()}"
            p.setPen(QPen(edge, 2))
            p.setBrush(fill)
            p.drawRoundedRect(QRectF(x, y, w, 40), 20, 20)
            p.setPen(Qt.NoPen)
            p.setBrush(edge if i <= active else EDGE)
            p.drawEllipse(QRectF(x + 8, y + 7, 26, 26))
            p.setPen(BG if i <= active else DIM)
            p.setFont(font(14, bold=True))
            p.drawText(QRectF(x + 8, y + 7, 26, 26), Qt.AlignCenter, mark)
            p.setPen(fg)
            p.setFont(font(15, bold=i == active))
            p.drawText(QRectF(x + 42, y, w - 50, 40), Qt.AlignVCenter | Qt.AlignLeft, label)
            if i < n - 1:
                p.setPen(QPen(FAINT, 2))
                ax = x + w + 6
                p.drawLine(QPointF(ax, y + 20), QPointF(ax + gap - 12, y + 20))
                p.drawLine(QPointF(ax + gap - 18, y + 14), QPointF(ax + gap - 12, y + 20))
                p.drawLine(QPointF(ax + gap - 18, y + 26), QPointF(ax + gap - 12, y + 20))

    def _fit(self, p, img, area):
        s = min(area.width() / img.width(), area.height() / img.height())
        w, h = img.width() * s, img.height() * s
        target = QRectF(area.x() + (area.width() - w) / 2, area.y() + (area.height() - h) / 2, w, h)
        p.drawImage(target, img)
        return target

    def _camera(self, p):
        area = self._panel(p, CAMERA, "ROBOT CAMERA  ·  YOLOE")
        if self.camera is None:
            p.setPen(FAINT)
            p.setFont(font(14))
            p.drawText(area, Qt.AlignCenter, "waiting for the robot…")
            return
        self._fit(p, self.camera, area.adjusted(6, 6, -6, -6))

    # -------------------------------------------------------------- the map

    def _map(self, p):
        objs = self.graph.get("objects", [])
        rooms = self.graph.get("rooms", [])
        area = self._panel(p, MAP, "SCENE GRAPH  ·  TOP-DOWN",
                           right=f"{len(rooms)} rooms · {len(objs)} objects")
        area = area.adjusted(8, 8, -8, -8)
        pts = [tuple(v[:2]) for v in self.graph.get("trajectory", [])]
        pts += [tuple(o["xyz"][:2]) for o in objs]
        if self.grid is not None and len(self.grid[6]):
            img, ox, oy, res, gw, gh, known = self.grid
            (r0, c0), (r1, c1) = known.min(0), known.max(0)
            pts += [(ox + c0 * res, oy + r0 * res), (ox + (c1 + 1) * res, oy + (r1 + 1) * res)]
        if not pts:
            p.setPen(FAINT)
            p.setFont(font(14))
            p.drawText(area, Qt.AlignCenter, "no map yet")
            return
        xs, ys = [q[0] for q in pts], [q[1] for q in pts]
        xmin, xmax, ymin, ymax = min(xs) - 1, max(xs) + 1, min(ys) - 1, max(ys) + 1
        s = min(area.width() / max(xmax - xmin, 1e-3), area.height() / max(ymax - ymin, 1e-3))
        cx = area.x() + (area.width() - (xmax - xmin) * s) / 2
        cy = area.y() + (area.height() - (ymax - ymin) * s) / 2

        def P(x, y):
            return QPointF(cx + (x - xmin) * s, cy + (ymax - y) * s)

        p.save()
        p.setClipRect(area)
        if self.grid is not None:
            img, ox, oy, res, gw, gh, _ = self.grid
            p.drawImage(QRectF(P(ox, oy + gh * res), P(ox + gw * res, oy)), img)
        room_xy = {r["id"]: r["xy"] for r in rooms}
        cont_xy = {c["id"]: c["xy"] for c in self.graph.get("containers", [])}
        # Scene-graph edges: object -> its surface, else its room; surface -> room.
        p.setPen(QPen(QColor(245, 197, 66, 70), 1))
        for c in self.graph.get("containers", []):
            if c.get("room_id") in room_xy:
                p.drawLine(P(*c["xy"]), P(*room_xy[c["room_id"]]))
        for o in objs:
            parent = cont_xy.get(o.get("container_id")) or room_xy.get(o.get("room_id"))
            if parent:
                p.drawLine(P(*o["xyz"][:2]), P(*parent))
        traj = self.graph.get("trajectory", [])
        if len(traj) > 1:
            p.setPen(QPen(FIRE, 2.5))
            p.drawPolyline(QPolygonF([P(*q) for q in traj]))
        goal = self.graph.get("goal")
        if goal:
            p.setPen(QPen(GREEN, 2))
            g = P(*goal)
            p.drawLine(g + QPointF(-6, -6), g + QPointF(6, 6))
            p.drawLine(g + QPointF(-6, 6), g + QPointF(6, -6))
        for c in self.graph.get("containers", []):
            q = P(*c["xy"])
            p.setPen(Qt.NoPen)
            p.setBrush(BLUE)
            p.drawRect(QRectF(q.x() - 4, q.y() - 4, 8, 8))
        p.setFont(font(10))
        for o in objs:
            if o.get("target") or ghost(o):
                continue
            q = P(*o["xyz"][:2])
            col = QColor(TEXT)
            col.setAlphaF(0.35 + 0.65 * float(o.get("p", 1.0)))
            p.setPen(Qt.NoPen)
            p.setBrush(col)
            p.drawEllipse(q, 3, 3)
            if o.get("name"):
                # The other extinguishers are named too: "#2" stays "#2".
                p.setPen(TEXT)
                p.setFont(font(12, bold=True))
                p.drawText(q + QPointF(8, -6), short(o))
                p.setFont(font(10))
        for r in rooms:
            q = P(*r["xy"])
            p.setPen(QPen(BG, 1))
            p.setBrush(YELLOW)
            p.drawEllipse(q, 5, 5)
            p.setPen(YELLOW)
            p.setFont(font(11, bold=True))
            p.drawText(q + QPointF(8, 4), pretty(r.get("label") or f"room {r['id']}"))
        robot = self.graph.get("robot")
        if robot:
            q = P(robot[0], robot[1])
            p.setPen(QPen(BG, 1.5))
            p.setBrush(CYAN)
            p.drawEllipse(q, 7, 7)
            if robot[2] is not None:
                a = math.radians(robot[2])
                p.setPen(QPen(CYAN, 2.5))
                p.drawLine(q, q + QPointF(16 * math.cos(a), -16 * math.sin(a)))
        for nm, a, b in self.moves:
            # Where an instance was moved from, to: a dashed arrow.
            qa, qb = P(*a), P(*b)
            p.setPen(QPen(GREEN, 2, Qt.DashLine))
            p.drawLine(qa, qb)
            ang = math.atan2(qb.y() - qa.y(), qb.x() - qa.x())
            for da in (2.6, -2.6):
                p.drawLine(qb, qb + QPointF(12 * math.cos(ang + da), 12 * math.sin(ang + da)))
            p.setPen(GREEN)
            p.setFont(font(11, bold=True))
            mid = (qa + qb) / 2
            p.drawText(mid + QPointF(8, 0), f"moved {math.dist(a, b):.1f} m")
        pulse = 10 + 6 * (0.5 + 0.5 * math.sin(time.time() * 4))
        for o in objs:
            if not o.get("target") or ghost(o):
                continue
            q = P(*o["xyz"][:2])
            if "instance" in o and float(o.get("p", 1.0)) < MISSING_P:
                # The target's believed place, which the robot has seen empty.
                p.setPen(QPen(YELLOW, 2, Qt.DashLine))
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(q, 12, 12)
                p.setPen(QPen(YELLOW, 2.5))
                p.drawLine(q + QPointF(-6, -6), q + QPointF(6, 6))
                p.drawLine(q + QPointF(-6, 6), q + QPointF(6, -6))
                p.setFont(font(12, bold=True))
                p.drawText(q + QPointF(16, -8), f"{short(o)} not here")
                continue
            p.setPen(QPen(RED, 2))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(q, pulse, pulse)
            p.setPen(Qt.NoPen)
            p.setBrush(RED)
            p.drawEllipse(q, 6, 6)
            p.setPen(RED)
            p.setFont(font(13, bold=True))
            p.drawText(q + QPointF(14, -10), short(o))
        p.restore()

    # ------------------------------------------------------- the hierarchy

    def _graph_panel(self, p):
        g = self.graph
        rooms = {r["id"]: r for r in g.get("rooms", [])}
        conts = {c["id"]: c for c in g.get("containers", [])}
        objs = g.get("objects", [])
        right = f"{len(rooms)} rooms  ·  {len(conts)} surfaces  ·  {len(objs)} objects"
        area = self._panel(p, GRAPH, "SCENE GRAPH  ·  ROOM → SURFACE → OBJECT", right=right)
        if not objs:
            p.setPen(FAINT)
            p.setFont(font(15))
            p.drawText(area, Qt.AlignCenter, "building the scene graph as the robot explores…")
            return
        by_room = {}
        for o in objs:
            by_room.setdefault(o.get("room_id", 0), []).append(o)
        target_rooms = {o.get("room_id", 0) for o in objs if o.get("target")}
        order = sorted(by_room, key=lambda rid: (rid not in target_rooms, -len(by_room[rid])))
        y = area.y() + 10
        x = area.x() + 14
        line_h = 30
        max_lines = int((area.height() - 14) // line_h)
        if self.events:
            # The hierarchy keeps up to three rows; the decisions get the rest.
            max_lines = min(max_lines, 3, len(order))
            top = area.y() + 10 + max_lines * line_h + 6
            self._decisions(p, QRectF(area.x(), top, area.width(), area.bottom() - top))
        lines = 0
        for rid in order:
            if lines >= max_lines:
                break
            room = rooms.get(rid)
            room_name = pretty(room.get("label") or f"room {rid}") if room else "not in a room yet"
            members = by_room[rid]
            p.setPen(YELLOW)
            p.setFont(font(15, bold=True))
            p.drawText(QRectF(x, y, 230, line_h), Qt.AlignVCenter | Qt.AlignLeft, f"▣ {room_name}")
            # Objects on a surface are listed with it; the rest as counts.
            on = {}
            loose = []
            for o in members:
                (on.setdefault(o["container_id"], []) if o.get("container_id") in conts else loose).append(o)
            cx = x + 240
            parts = []
            for cid, items in on.items():
                parts.append((f"{pretty(conts[cid]['label'])}: " + ", ".join(name(i) for i in items if not ghost(i)), BLUE, False))
            counts = {}
            for o in loose:
                if ghost(o):
                    continue
                key = name(o)
                counts.setdefault(key, [0, False])
                counts[key][0] += 1
                counts[key][1] |= bool(o.get("target"))
            for label, (k, is_t) in sorted(counts.items(), key=lambda kv: (not kv[1][1], -kv[1][0])):
                parts.append((("★ " if is_t else "") + label + (f" ×{k}" if k > 1 else ""), RED if is_t else TEXT, is_t))
            for text, col, bold in parts:
                p.setFont(font(15, bold=bold))
                tw = p.fontMetrics().horizontalAdvance(text) + 22
                if cx + tw > area.right() - 10:
                    p.setPen(FAINT)
                    p.drawText(QRectF(cx, y, 40, line_h), Qt.AlignVCenter | Qt.AlignLeft, "…")
                    break
                p.setPen(col)
                p.drawText(QRectF(cx, y, tw, line_h), Qt.AlignVCenter | Qt.AlignLeft, text)
                cx += tw
            y += line_h
            lines += 1

    def _decisions(self, p, area):
        """The last few instance decisions, newest at the bottom."""
        p.setPen(QPen(EDGE, 1))
        p.drawLine(QPointF(area.x() + 14, area.y()), QPointF(area.right() - 14, area.y()))
        p.setPen(DIM)
        p.setFont(font(12, bold=True))
        p.drawText(QRectF(area.x() + 14, area.y() + 4, 600, 20), Qt.AlignVCenter | Qt.AlignLeft,
                   "WHICH EXTINGUISHER IS WHICH  ·  DECISIONS")
        row = 25
        rows = max(1, int((area.height() - 28) // row))
        y = area.y() + 26
        for t, step, text, col in self.events[-rows:]:
            p.setPen(FAINT)
            p.setFont(font(13, mono=True))
            p.drawText(QRectF(area.x() + 14, y, 170, row), Qt.AlignVCenter | Qt.AlignLeft,
                       f"{int(t // 60):02d}:{int(t % 60):02d} · step {step}")
            p.setPen(col)
            p.setFont(font(15, bold=col is not TEXT))
            p.drawText(QRectF(area.x() + 190, y, area.width() - 204, row), Qt.AlignVCenter | Qt.AlignLeft,
                       p.fontMetrics().elidedText(text, Qt.ElideRight, int(area.width() - 204)))
            y += row

    # ------------------------------------------------------------ the VLM

    def _vlm(self, p):
        m = self.mission
        stage = m.get("stage")
        model = m.get("model") or self.model_name
        accent = None
        if stage == "verdict":
            accent = GREEN if m.get("condition") == "good" else RED if m.get("condition") == "bad" else YELLOW
        elif stage == "inspecting":
            accent = FIRE
        area = self._panel(p, VLM, "VLM CONDITION INSPECTION", right="NVIDIA API", accent=accent)
        inner = area.adjusted(10, 8, -10, -34)
        p.setPen(FAINT)
        p.setFont(font(12, mono=True))
        p.drawText(QRectF(area.x() + 10, area.bottom() - 28, area.width() - 20, 22),
                   Qt.AlignVCenter | Qt.AlignLeft, model)
        if stage not in ("inspecting", "verdict"):
            p.setPen(FAINT)
            p.setFont(font(15))
            msg = ((m.get("reason") or "target not found within the step budget") if stage == "not_found"
                   else "waiting for the robot to reach the fire extinguisher…")
            p.drawText(inner, Qt.AlignCenter, msg)
            return
        thumb = QRectF(inner.x(), inner.y(), 150, 210)
        if self.view is not None:
            self._fit(p, self.view, thumb)
        else:
            p.setPen(QPen(EDGE, 1, Qt.DashLine))
            p.setBrush(Qt.NoBrush)
            p.drawRect(thumb)
        tx = thumb.right() + 14
        text_w = inner.right() - tx
        below = thumb.bottom() + 16
        if stage == "inspecting":
            dots = "." * (1 + int(time.time() * 2) % 3)
            p.setPen(FIRE)
            p.setFont(font(26, bold=True))
            p.drawText(QRectF(tx, inner.y() + 10, text_w, 40), Qt.AlignVCenter | Qt.AlignLeft, f"Analysing{dots}")
            waited = time.time() - (self.t_inspect or time.time())
            p.setPen(DIM)
            p.setFont(font(14))
            p.drawText(QRectF(tx, inner.y() + 56, text_w, 80), Qt.TextWordWrap | Qt.AlignTop | Qt.AlignLeft,
                       f"the robot's best close-ups of the extinguisher, sent to the VLM\n{waited:4.1f} s")
            p.setFont(font(15))
            p.setPen(DIM)
            p.drawText(QRectF(inner.x(), below, inner.width(), 24), Qt.AlignVCenter | Qt.AlignLeft, "checking:")
            checks = ["paint & corrosion", "hose & nozzle", "handle, pin & gauge",
                      "upright in its stand", "powder residue, blocked access"]
            hot = int(time.time() * 2) % len(checks)
            for i, c in enumerate(checks):
                p.setPen(TEXT if i == hot else FAINT)
                p.drawText(QRectF(inner.x() + 12, below + 30 + i * 30, inner.width() - 12, 26),
                           Qt.AlignVCenter | Qt.AlignLeft, ("▸ " if i == hot else "◦ ") + c)
            return
        cond = str(m.get("condition", "unknown"))
        col = GREEN if cond == "good" else RED if cond == "bad" else YELLOW
        p.setPen(col)
        p.setFont(font(54, bold=True))
        head = cond.upper() if cond in ("good", "bad") else "NONE"
        p.drawText(QRectF(tx, inner.y(), text_w, 70), Qt.AlignVCenter | Qt.AlignLeft, head)
        p.setPen(TEXT)
        p.setFont(font(16))
        if cond in ("good", "bad"):
            sub = f"{float(m.get('confidence', 0)):.0%} confidence\nanswered in {float(m.get('latency_s', 0)):.1f} s"
            if m.get("fallback"):
                sub += "\n(reasoning ran long: direct answer)"
        else:
            sub = str(m.get("error") or "no verdict")
        p.drawText(QRectF(tx, inner.y() + 74, text_w, 120), Qt.TextWordWrap | Qt.AlignTop | Qt.AlignLeft, sub)
        y = below
        issues = m.get("issues") or (["no defects visible"] if cond == "good" else [])
        p.setFont(font(15))
        fm = p.fontMetrics()
        for issue in issues[:4]:
            rect = fm.boundingRect(0, 0, int(inner.width() - 22), 200, Qt.TextWordWrap, str(issue))
            p.setPen(col)
            p.drawText(QRectF(inner.x(), y, 20, 22), Qt.AlignVCenter | Qt.AlignLeft, "●")
            p.setPen(TEXT)
            p.drawText(QRectF(inner.x() + 20, y + 1, inner.width() - 22, rect.height() + 2),
                       Qt.TextWordWrap | Qt.AlignTop | Qt.AlignLeft, str(issue))
            y += rect.height() + 8
        p.setPen(DIM)
        p.setFont(font(13))
        p.drawText(QRectF(inner.x(), y + 4, inner.width(), inner.bottom() - y - 4),
                   Qt.TextWordWrap | Qt.AlignTop | Qt.AlignLeft, str(m.get("summary", "")))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--geometry", default=f"{W}x{H}+0+0")
    ap.add_argument("--model", default="nvidia/nemotron-3.5-super-vl-120b-a12b",
                    help="shown before the first inspection names its own")
    args, qt_args = ap.parse_known_args(argv)
    m = re.fullmatch(r"(\d+)x(\d+)\+(\d+)\+(\d+)", args.geometry)
    if not m:
        ap.error("--geometry must look like 1920x1080+0+0")
    rclpy.init(args=None)
    node = rclpy.create_node("extinguisher_demo_hud")
    app = QApplication([sys.argv[0]] + qt_args)
    hud = Hud(node, args.model)
    hud.move(int(m.group(3)), int(m.group(4)))
    hud.show()
    try:
        return app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
