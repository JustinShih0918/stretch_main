"""Telling objects of one class apart: "fire extinguisher #1", "#2".

The rest of the pipeline is category-level: every track of the target's label
is a candidate, ranked by its belief. With two extinguishers in a building
that cannot answer "inspect #1", and when #1 is moved it goes wrong in a
specific way: the robot finds #1's place empty, the presence filter retires
that track, and the next best candidate is #2, which is still where it was --
so #2 is inspected and reported as the one that was asked for, and the
extinguisher that actually moved is never looked for.

An instance is an index bound to the track that currently IS that object.
`InstanceRegistry.update` re-applies five rules every keyframe, per label in
`labels`, over the confirmed tracks (named by the detector, `min_obs`
observations, not blacklisted):

  fragment  an unbound track within `fragment_m` of an instance's track, and
            never detected beside it in one frame, is a second ellipsoid of
            the same object. It answers to that index and never becomes an
            object of its own. (Two real extinguishers cannot stand closer than
            their own width, ~0.35 m; two tracks fed by separate parts of one
            frame are two objects however close their estimates put them.)
  missing   an instance is missing when its track and all its fragments are
            disbelieved by the presence filter (p < `missing_p`, the
            candidate gate's `min_presence`): the robot looked where the map
            says it is and it was not there.
  moved     an unbound, believed track that nothing explains is claimed by a
            missing instance of its label, nearest first. The old track stays
            in the map as what it is -- a place the object used to be.
  new       an unbound, believed track is given the next index only when it
            cannot be one of the known instances moved: none of them is
            missing, and every one has been seen in this run where its track
            says. Otherwise it stays unindexed ("?") until the robot has
            looked.
  drifted   the one exception to "never": an instance whose believed track has
            been pulled more than `drift_m` off the place the instance was put
            (its anchor: where it was indexed, moved to, or found on reload)
            while another believed track of its label, held by no other
            instance, sits within `fragment_m` of that place and nearer it. The
            track at the place is the instance; the one that wandered took a
            neighbour's detections (or a false one's: #2's track pulled 0.46 m
            toward the cart beside it, #2's own detections began a track 0.24 m
            from its place, and that was indexed "#3", outputs/20261113_162238). #1 put 0.65 m from #2 and seen alone from across the
            lobby joined #2's track, whose centre the refit then pulled 0.45 m
            toward #1; #2's own detections started a fresh track at #2's place,
            which nothing explained, so missing #1 claimed it and #2 was
            inspected as #1 (outputs/20261113_154556). The track it leaves
            holds both objects' detections and sits between them: it is
            retired, never indexed, claimed, or a fragment ("#3" midway
            between them otherwise, outputs/20261113_160353).
  never     otherwise, an instance whose track is believed is never re-bound.
            A second extinguisher seen where it should be cannot become the
            first one.

Nothing here moves a track or changes a belief: the registry is a view over
the object layer, like the scene graph, and dropping it leaves every other
module's behaviour as it was.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

from ..core.labels import normalize_label
from ..mapping.costmap import HEIGHT_AXIS, PLANE


@dataclass
class Instance:
    label: str
    index: int
    track_id: int
    # Where it has been moved from and to, as the registry saw it happen.
    moves: List[dict] = field(default_factory=list)
    # Ground-plane point where it was put: indexed, moved to, re-anchored, or
    # its track's centre when a map was loaded. Only those events move it.
    anchor: Optional[np.ndarray] = field(default=None, repr=False, compare=False)

    @property
    def name(self) -> str:
        return f"{self.label} #{self.index}"


def detection_xyz(det, frame, min_valid: int = 8) -> Optional[np.ndarray]:
    """World point of a detection: the median valid depth of its mask (its box
    when it has none), at the mask's centroid. None without enough depth."""
    depth = getattr(frame, "depth", None)
    if depth is None:
        return None
    mask = getattr(det, "mask", None)
    if mask is not None and np.asarray(mask).shape == depth.shape:
        ys, xs = np.nonzero(mask)
    else:
        x1, y1, x2, y2 = (int(round(float(v))) for v in det.bbox_xyxy)
        h, w = depth.shape[:2]
        x1, x2 = max(0, x1), min(w, x2)
        y1, y2 = max(0, y1), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        ys, xs = np.mgrid[y1:y2, x1:x2]
        ys, xs = ys.ravel(), xs.ravel()
    if ys.size == 0:
        return None
    d = depth[ys, xs]
    valid = d > 1e-3
    if int(valid.sum()) < int(min_valid):
        return None
    from ..core.geometry import backproject_pixel

    return backproject_pixel(float(xs[valid].mean()), float(ys[valid].mean()),
                             float(np.median(d[valid])), frame.intrinsics, frame.T_wc)


def split_target(target: str) -> Tuple[str, int]:
    """'fire extinguisher #2' -> ('fire extinguisher', 2); no '#k' -> index 0 (any)."""
    text = str(target).strip()
    head, sep, tail = text.rpartition("#")
    if sep and tail.strip().isdigit() and head.strip():
        return head.strip(), int(tail.strip())
    return text, 0


class InstanceRegistry:
    def __init__(self, labels: Iterable[str], fragment_m: float = 0.4,
                 missing_p: float = 0.45, min_obs: int = 2,
                 drift_m: Optional[float] = None, max_height_m: float = 0.0) -> None:
        self.labels = {normalize_label(l) for l in labels if str(l).strip()}
        self.fragment_m = float(fragment_m)
        # Half the fragment radius: a track this close to an instance's place is
        # at it, and one this far off has left it.
        self.drift_m = float(drift_m) if drift_m is not None else self.fragment_m / 2
        # Tracks centred higher than this above `floor_y` are not of the class
        # (a wall sign, for a class that stands on the floor). 0 disables; the
        # agent sets `floor_y` from its map.
        self.max_height_m = float(max_height_m or 0.0)
        self.floor_y: Optional[float] = None
        self.missing_p = float(missing_p)
        self.min_obs = int(min_obs)
        self._by_label: Dict[str, Dict[int, Instance]] = {}
        self._fragment_of: Dict[int, Tuple[str, int]] = {}
        # Tracks the drifted rule took an instance off: blends of two objects.
        self._retired: set = set()
        self.events: List[dict] = []

    # ------------------------------------------------------------- queries

    def covers(self, label: str) -> bool:
        return normalize_label(label) in self.labels

    def instances(self, label: Optional[str] = None) -> List[Instance]:
        labels = [normalize_label(label)] if label is not None else sorted(self._by_label)
        out = []
        for lab in labels:
            out += [self._by_label.get(lab, {})[k] for k in sorted(self._by_label.get(lab, {}))]
        return out

    def get(self, label: str, index: int) -> Optional[Instance]:
        return self._by_label.get(normalize_label(label), {}).get(int(index))

    def instance_of(self, track_id: int) -> Optional[Instance]:
        """The instance a track IS (its bound track) or is a fragment of."""
        tid = int(track_id)
        for insts in self._by_label.values():
            for inst in insts.values():
                if inst.track_id == tid:
                    return inst
        ref = self._fragment_of.get(tid)
        return self._by_label.get(ref[0], {}).get(ref[1]) if ref else None

    def index_of(self, track_id: int) -> Optional[int]:
        inst = self.instance_of(track_id)
        return None if inst is None else inst.index

    def name_of(self, track) -> str:
        """'fire extinguisher #1', 'fire extinguisher ?' (a covered label not yet
        indexed), or the plain label for a class this registry does not cover."""
        if not self.covers(track.label):
            return str(track.label)
        k = self.index_of(track.id)
        return f"{track.label} #{k}" if k is not None else f"{track.label} ?"

    def belongs(self, xyz, track_id: int, layer, match_m: float) -> bool:
        """Is an object seen at world point `xyz` the one `track_id` is, rather
        than another instance of its label? Within `match_m` of that track (on
        the ground plane), and no nearer to any other instance's track.

        This is what "the target is in view" has to mean once there are two of
        them: walking up to the place #1 was taken from, #2 two metres beyond it
        is in the frame, and counting it as the target cancels the absence
        reading -- the robot stops at the empty place and the inspection
        judges #2."""
        track = layer.get(int(track_id))
        if track is None or xyz is None:
            return False
        p = np.asarray(xyz, dtype=float)[list(PLANE)]
        own = float(np.linalg.norm(self._xy(layer, track) - p))
        if own > float(match_m):
            return False
        mine = self.instance_of(track.id)
        for inst in self._by_label.get(normalize_label(track.label), {}).values():
            if mine is not None and inst.index == mine.index:
                continue
            other = layer.get(inst.track_id)
            if other is None or other.id == track.id or self._track_missing(other):
                continue
            if float(np.linalg.norm(self._xy(layer, other) - p)) < own:
                return False
        return True

    def claim(self, label: str, index: int, track_id: int, layer, step=None,
              why: str = "") -> Optional[dict]:
        """Bind instance `index` to `track_id` on evidence the rules above do not
        see -- the inspection's own frames at the stop. Only a missing instance
        can be claimed, and only by a track no other instance holds. Returns
        the "moved" event, or None when the claim is refused."""
        inst = self.get(label, index)
        track = layer.get(int(track_id))
        if inst is None or track is None or not self._missing(inst, layer) or self._too_high(layer, track):
            return None
        holder = self.instance_of(track.id)
        if holder is not None and holder.index != inst.index:
            return None
        old = layer.get(inst.track_id)
        d = (float(np.linalg.norm(self._xy(layer, track) - self._xy(layer, old)))
             if old is not None else 0.0)
        move = {"step": None if step is None else int(step), "from_track": int(inst.track_id),
                "from_xy": self._ros_xy(layer, old), "to_track": int(track.id),
                "to_xy": self._ros_xy(layer, track), "distance_m": round(d, 2), "why": why}
        inst.moves.append(move)
        self._bind(inst, track, layer)
        self._fragment_of.pop(int(track.id), None)
        event = dict(self._event("moved", inst, layer, step), **move)
        self.events.append(event)
        return event

    def same_as(self, track, label: str, index: int, layer) -> bool:
        """Is `track` instance `index` of `label`? Its bound track or a fragment;
        or a track the rules never bind (the region proposer's, one not yet
        confirmed) within `fragment_m` of the instance's believed track and
        nearer it than any other instance's. A stop can come to rest on such a
        second track of the very object the instance was just found as
        (outputs/20261113_175735: #1 claimed at the chairs, the stop on a
        proposer track 0.15 m from it, inspected and reported as "?")."""
        inst = self.get(label, index)
        if inst is None or track is None or self._too_high(layer, track):
            return False
        k = self.index_of(track.id)
        if k is not None:
            return k == inst.index
        held = layer.get(inst.track_id)
        if held is None or self._track_missing(held):
            return False
        xy = self._xy(layer, track)
        d = float(np.linalg.norm(xy - self._xy(layer, held)))
        if d > self.fragment_m:
            return False
        for other in self._by_label.get(inst.label, {}).values():
            o = layer.get(other.track_id)
            if other.index == inst.index or o is None or self._track_missing(o):
                continue
            if float(np.linalg.norm(xy - self._xy(layer, o))) < d:
                return False
        return True

    def is_track_missing(self, track) -> bool:
        """The presence filter no longer believes this track's object is there."""
        return self._track_missing(track)

    def is_missing(self, label: str, index: int, layer) -> bool:
        inst = self.get(label, index)
        return inst is not None and self._missing(inst, layer)

    def order_for(self, candidates: List, label: str, index: int, layer) -> List:
        """The candidate list for a mission that names instance `index`.

        The instance's own track (and its fragments) first, in the order they
        came. Unindexed tracks of the label only when the instance is missing --
        then one of them is where it went -- or when no such instance is known
        yet. Every other instance's tracks never: going to #2 is not a way of
        finding #1.
        """
        label = normalize_label(label)
        own, free = [], []
        for t in candidates:
            if normalize_label(t.label) != label:
                continue
            k = self.index_of(t.id)
            if k == int(index):
                own.append(t)
            elif k is None and int(t.id) not in self._retired and not self._too_high(layer, t):
                free.append(t)
        inst = self.get(label, index)
        if inst is not None and not self._missing(inst, layer):
            return own
        return own + free

    # -------------------------------------------------------------- update

    def bootstrap(self, layer) -> List[dict]:
        """Index a map that carries no instances (one saved before they existed):
        every confirmed track of a covered label, in creation order, fragments
        folded into the instance they belong to."""
        events = []
        for label in sorted(self.labels):
            insts = self._by_label.setdefault(label, {})
            for t in sorted(self._confirmed(layer, label), key=lambda t: t.id):
                if self.instance_of(t.id) is not None or self._fragment_host(t, layer) is not None:
                    self._refresh_fragments(layer, label)
                    continue
                k = max(insts, default=0) + 1
                insts[k] = Instance(label=label, index=k, track_id=int(t.id))
                self._bind(insts[k], t, layer)
                events.append(self._event("indexed", insts[k], layer, step=None))
                self._refresh_fragments(layer, label)
        self.events += events
        return events

    def update(self, layer, step: Optional[int] = None) -> List[dict]:
        events: List[dict] = []
        for label in sorted(self.labels):
            insts = self._by_label.setdefault(label, {})
            self._refresh_fragments(layer, label)
            for inst in insts.values():
                if inst.anchor is None:
                    self._bind(inst, layer.get(inst.track_id), layer)
            # drifted: back to the track at the instance's place.
            for inst in insts.values():
                event = self._undrift(inst, layer, step)
                if event is not None:
                    events.append(event)
                    self._refresh_fragments(layer, label)
            # A missing instance whose own fragment is still believed was never
            # gone: its first ellipsoid was the poor one. Re-anchor on the
            # fragment rather than calling it moved.
            for inst in insts.values():
                if self._track_missing(layer.get(inst.track_id)):
                    alive = [tid for tid, ref in self._fragment_of.items()
                             if ref == (label, inst.index)
                             and not self._track_missing(layer.get(tid))]
                    if alive:
                        self._bind(inst, layer.get(int(min(alive))), layer)
                        events.append(self._event("reanchored", inst, layer, step))
            self._refresh_fragments(layer, label)
            free = [t for t in self._confirmed(layer, label)
                    if self.instance_of(t.id) is None
                    and not self._track_missing(t)]
            if not free:
                continue
            missing = [inst for inst in insts.values() if self._missing(inst, layer)]
            # moved: nearest (missing instance, unexplained track) pairs first.
            pairs = []
            for inst in missing:
                old = self._xy(layer, layer.get(inst.track_id))
                for t in free:
                    d = float(np.linalg.norm(self._xy(layer, t) - old)) if old is not None else 0.0
                    pairs.append((d, inst.index, t.id))
            claimed_inst, claimed_track = set(), set()
            for d, k, tid in sorted(pairs):
                if k in claimed_inst or tid in claimed_track:
                    continue
                inst = insts[k]
                old_track = layer.get(inst.track_id)
                move = {
                    "step": None if step is None else int(step),
                    "from_track": int(inst.track_id),
                    "from_xy": self._ros_xy(layer, old_track),
                    "to_track": int(tid),
                    "to_xy": self._ros_xy(layer, layer.get(tid)),
                    "distance_m": round(d, 2),
                }
                inst.moves.append(move)
                self._bind(inst, layer.get(tid), layer)
                claimed_inst.add(k)
                claimed_track.add(tid)
                events.append(dict(self._event("moved", inst, layer, step), **move))
            free = [t for t in free if t.id not in claimed_track]
            if not free:
                continue
            # new: only when no known instance could be this one, moved.
            accounted = all(
                not self._missing(inst, layer) and self._seen_live(layer.get(inst.track_id))
                for inst in insts.values()
            ) and not (set(i.index for i in missing) - claimed_inst)
            if not accounted:
                continue
            for t in sorted(free, key=lambda t: t.id):
                if self.instance_of(t.id) is not None or self._fragment_host(t, layer) is not None:
                    continue
                k = max(insts, default=0) + 1
                insts[k] = Instance(label=label, index=k, track_id=int(t.id))
                self._bind(insts[k], t, layer)
                events.append(self._event("indexed", insts[k], layer, step))
                self._refresh_fragments(layer, label)
        self._refresh_all_fragments(layer)
        self.events += events
        return events

    # -------------------------------------------------------- persistence

    def to_dict(self) -> dict:
        return {
            "labels": sorted(self.labels),
            "instances": [
                {"label": i.label, "index": int(i.index), "track_id": int(i.track_id),
                 "moves": list(i.moves)}
                for i in self.instances()
            ],
        }

    def load_dict(self, blob: dict, layer) -> int:
        """Restore instances whose track is in `layer`; returns how many."""
        self._by_label = {}
        self._fragment_of = {}
        self._retired = set()
        n = 0
        for rec in (blob or {}).get("instances", []):
            label = normalize_label(rec["label"])
            if label not in self.labels or layer.get(int(rec["track_id"])) is None:
                continue
            inst = Instance(label=label, index=int(rec["index"]), track_id=int(rec["track_id"]),
                            moves=list(rec.get("moves", [])))
            self._bind(inst, layer.get(inst.track_id), layer)
            self._by_label.setdefault(label, {})[inst.index] = inst
            n += 1
        self._refresh_all_fragments(layer)
        return n

    # ------------------------------------------------------------ helpers

    def _bind(self, inst: Instance, track, layer) -> None:
        """Bind `inst` to `track` and put its anchor where that track is."""
        if track is None:
            return
        inst.track_id = int(track.id)
        xy = self._xy(layer, track)
        inst.anchor = None if xy is None else np.array(xy, dtype=float)

    def _undrift(self, inst: Instance, layer, step) -> Optional[dict]:
        """The "drifted" rule for one instance: its "reanchored" event, or None."""
        track = layer.get(inst.track_id)
        if inst.anchor is None or track is None or self._track_missing(track):
            return None
        off = float(np.linalg.norm(self._xy(layer, track) - inst.anchor))
        if off <= self.drift_m:
            return None
        best, best_d = None, min(self.fragment_m, off)
        for t in self._confirmed(layer, inst.label):
            if t.id == track.id or self._track_missing(t):
                continue
            holder = self.instance_of(t.id)
            if holder is not None and holder.index != inst.index:
                continue
            d = float(np.linalg.norm(self._xy(layer, t) - inst.anchor))
            if d < best_d:
                best, best_d = t, d
        if best is None:
            return None
        old = int(inst.track_id)
        anchor = inst.anchor
        self._retired.add(old)
        self._fragment_of.pop(old, None)
        self._fragment_of.pop(int(best.id), None)
        self._bind(inst, best, layer)
        inst.anchor = anchor           # the place itself has not moved
        return dict(self._event("reanchored", inst, layer, step), from_track=old,
                    drift_m=round(off, 2),
                    why=f"its track drifted {off:.2f} m off its place; the track at the place is it")

    def _confirmed(self, layer, label: str) -> List:
        return [
            t for t in layer.tracks()
            if normalize_label(t.label) == label
            and not bool(getattr(t, "blacklisted", False))
            and not bool(getattr(t, "proposal_only", False))
            and int(getattr(t, "n_obs", 0)) >= self.min_obs
            and int(t.id) not in self._retired
            and not self._too_high(layer, t)
        ]

    def plausible(self, tracks: List, layer) -> List:
        """`tracks` without those of a covered class centred too high to be one."""
        return [t for t in tracks if not (self.covers(t.label) and self._too_high(layer, t))]

    def _too_high(self, layer, track) -> bool:
        if self.max_height_m <= 0.0 or self.floor_y is None or track is None:
            return False
        centre = np.asarray(layer.center_of(track), dtype=float)
        return float(centre[HEIGHT_AXIS]) - float(self.floor_y) > self.max_height_m

    def _track_missing(self, track) -> bool:
        if track is None or bool(getattr(track, "blacklisted", False)):
            return True
        pres = getattr(track, "presence", None)
        return pres is not None and float(pres.p) < self.missing_p

    def _missing(self, inst: Instance, layer) -> bool:
        if not self._track_missing(layer.get(inst.track_id)):
            return False
        return not any(ref == (inst.label, inst.index) and not self._track_missing(layer.get(tid))
                       for tid, ref in self._fragment_of.items())

    @staticmethod
    def _seen_live(track) -> bool:
        return bool(getattr(track, "seen_live", True)) if track is not None else False

    @staticmethod
    def _xy(layer, track) -> Optional[np.ndarray]:
        if track is None:
            return None
        return np.asarray(layer.center_of(track), dtype=float)[list(PLANE)]

    def _ros_xy(self, layer, track) -> Optional[List[float]]:
        xy = self._xy(layer, track)
        # Pipeline ground plane (x, z) -> ROS map (x, y = -z).
        return None if xy is None else [round(float(xy[0]), 2), round(float(-xy[1]), 2)]

    def _fragment_host(self, track, layer) -> Optional[Instance]:
        """The instance whose track lies within `fragment_m` of `track`, if any."""
        label = normalize_label(track.label)
        xy = self._xy(layer, track)
        best, best_d = None, self.fragment_m
        for inst in self._by_label.get(label, {}).values():
            if inst.track_id == track.id:
                return inst
            host = layer.get(inst.track_id)
            if host is None or self._co_observed(host, track):
                continue
            d = float(np.linalg.norm(self._xy(layer, host) - xy))
            if d < best_d:
                best, best_d = inst, d
        return best

    # Two detections in one frame are two objects when their centres are this
    # many summed standard deviations apart along the line joining them. Two
    # extinguishers standing touching: centres one width apart, each mask's
    # std across it ~0.29 width, so the ratio is ~1.7. A second box on the same
    # object (or one partly hidden behind its neighbour, seen along the wall)
    # overlaps the first and stays under 1.
    SEPARATE_SIGMAS = 1.2

    @classmethod
    def _co_observed(cls, a, b) -> bool:
        """Both tracks took a detection from the same frame, in separate parts of
        the image -- so they are two objects, however close their centres are
        estimated. The associator gives each detection to one track; but two
        heavily overlapping detections (a duplicate box, an object half hidden
        behind its neighbour) are not evidence of a second object, and a track
        made from one sat 0.23 m from extinguisher #2 and was indexed "#3"
        (outputs/20261113_012324)."""
        obs_a = {int(o.frame_id): o for o in (getattr(a, "observations", None) or []) if o is not None}
        for ob in (getattr(b, "observations", None) or []):
            if ob is None or int(ob.frame_id) not in obs_a:
                continue
            oa = obs_a[int(ob.frame_id)]
            mu_a, mu_b = getattr(oa, "mu", None), getattr(ob, "mu", None)
            if mu_a is None or mu_b is None:
                return True        # nothing to tell them apart by: trust the associator
            d = np.asarray(mu_b, dtype=float) - np.asarray(mu_a, dtype=float)
            dist = float(np.linalg.norm(d))
            if dist < 1e-9:
                continue
            u = d / dist
            spread = sum(float(np.sqrt(max(u @ np.asarray(o.cov, dtype=float) @ u, 0.0)))
                         for o in (oa, ob))
            if dist > cls.SEPARATE_SIGMAS * spread:
                return True
        return False

    def _refresh_fragments(self, layer, label: str) -> None:
        for tid in [tid for tid, ref in self._fragment_of.items() if ref[0] == label]:
            del self._fragment_of[tid]
        bound = {i.track_id for i in self._by_label.get(label, {}).values()}
        for t in self._confirmed(layer, label):
            if t.id in bound:
                continue
            host = self._fragment_host(t, layer)
            if host is not None:
                self._fragment_of[int(t.id)] = (label, host.index)

    def _refresh_all_fragments(self, layer) -> None:
        for label in self.labels:
            self._refresh_fragments(layer, label)

    def _event(self, kind: str, inst: Instance, layer, step) -> dict:
        return {"event": kind, "step": None if step is None else int(step),
                "label": inst.label, "index": int(inst.index), "name": inst.name,
                "track_id": int(inst.track_id),
                "xy": self._ros_xy(layer, layer.get(inst.track_id))}
