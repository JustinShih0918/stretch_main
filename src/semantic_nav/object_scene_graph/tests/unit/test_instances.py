"""Instances of one class: "fire extinguisher #1" and "#2" (objects/instances.py).

The scenario these pin: two extinguishers mapped, #1 moved, the robot asked for
#1. It must look where #1 was, find it gone, take the extinguisher it cannot
otherwise explain as #1, and never take #2 -- which is still where it was --
for #1.
"""
import math
from types import SimpleNamespace

import numpy as np

from osg.objects.association import ObjectTrack
from osg.objects.ellipsoid import Ellipsoid
from osg.objects.instances import InstanceRegistry, split_target
from osg.objects.linking import relink
from osg.objects.presence import PresenceState

EXT = "fire extinguisher"
A, B, C = (-7.5, 5.12), (-9.6, 5.12), (-4.0, 1.0)


def _logodds(p):
    return math.log(p / (1.0 - p))


def track(tid, xy, label=EXT, frames=(1, 2, 3), p=0.9):
    # ROS (x, y) -> pipeline (x, y-up, z = -y)
    t = ObjectTrack(id=tid, label=label,
                    ellipsoid=Ellipsoid(center=np.array([xy[0], 0.3, -xy[1]]),
                                        axes=np.array([0.15, 0.3, 0.15]), R=np.eye(3)))
    t.observations = [SimpleNamespace(frame_id=f) for f in frames]
    t.presence = PresenceState(log_odds=_logodds(p))
    return t


class Layer:
    def __init__(self, *tracks):
        self._t = {t.id: t for t in tracks}

    def add(self, t):
        self._t[t.id] = t
        return t

    def tracks(self, include_blacklisted=False):
        return [t for t in self._t.values() if include_blacklisted or not t.blacklisted]

    def get(self, tid):
        return self._t.get(int(tid))

    def center_of(self, t):
        return t.ellipsoid.center


def registry():
    return InstanceRegistry([EXT], fragment_m=0.4, missing_p=0.45, min_obs=2)


def prior_map():
    """Day 1's map: #1 at A, #2 at B, both from a previous session (negative frames)."""
    layer = Layer(track(10, A, frames=(-900, -899)), track(11, B, frames=(-880, -879)))
    for t in layer.tracks():
        t.from_prior = True
    reg = registry()
    reg.bootstrap(layer)
    return layer, reg


def test_a_map_run_indexes_each_extinguisher_once_in_the_order_it_was_found():
    layer, reg = Layer(), registry()
    layer.add(track(5, A))
    layer.add(track(6, B, frames=(7,)))            # one sighting: not confirmed yet
    reg.update(layer, step=1)
    assert reg.index_of(5) == 1 and reg.index_of(6) is None
    layer.get(6).observations.append(SimpleNamespace(frame_id=8))
    reg.update(layer, step=2)
    assert reg.index_of(6) == 2
    assert [i.name for i in reg.instances(EXT)] == ["fire extinguisher #1", "fire extinguisher #2"]


def test_a_second_ellipsoid_of_the_same_extinguisher_is_not_a_third_one():
    layer, reg = Layer(track(5, A, frames=(1, 2))), registry()
    reg.update(layer)
    layer.add(track(6, (A[0] + 0.1, A[1]), frames=(5, 6)))   # never seen with track 5
    reg.update(layer)
    assert reg.index_of(6) == 1
    assert len(reg.instances(EXT)) == 1


def _obs(frame_id, u, w=80.0, h=250.0):
    """An observation whose mask is a w x h px box centred at column u."""
    return SimpleNamespace(frame_id=frame_id, mu=np.array([u, 600.0]),
                           cov=np.diag([w * w / 12.0, h * h / 12.0]))


def test_two_tracks_seen_side_by_side_in_one_frame_are_two_objects_however_close():
    layer, reg = Layer(track(5, A, frames=(1,))), registry()
    layer.get(5).observations = [_obs(1, 300.0), _obs(2, 300.0)]
    reg.update(layer)
    t = layer.add(track(6, (A[0] + 0.1, A[1])))
    t.observations = [_obs(2, 385.0), _obs(3, 380.0)]       # frame 2: beside it, touching
    reg.update(layer)
    assert reg.index_of(6) == 2


def test_two_boxes_on_one_object_in_one_frame_are_still_one_object():
    """Seen along the wall, the one behind is half hidden by the one in front:
    the two boxes overlap, and that is not evidence of a second object."""
    layer, reg = Layer(track(5, A, frames=(1,))), registry()
    layer.get(5).observations = [_obs(1, 300.0), _obs(2, 300.0)]
    reg.update(layer)
    t = layer.add(track(6, (A[0] + 0.2, A[1])))
    t.observations = [_obs(2, 320.0, w=50.0), _obs(3, 310.0)]   # frame 2: overlapping
    reg.update(layer)
    assert reg.index_of(6) == 1 and len(reg.instances(EXT)) == 1


def test_a_new_extinguisher_waits_until_the_known_ones_are_checked():
    layer, reg = prior_map()
    assert (reg.index_of(10), reg.index_of(11)) == (1, 2)
    layer.add(track(20, C, frames=(30, 31)))       # seen today, nowhere a known one is
    reg.update(layer, step=31)
    # It may be #1 or #2, moved: not indexed, and not a candidate for #1 while
    # #1 is still believed where it was.
    assert reg.index_of(20) is None
    cands = [layer.get(10), layer.get(20), layer.get(11)]
    assert [t.id for t in reg.order_for(cands, EXT, 1, layer)] == [10]
    assert [t.id for t in reg.order_for(cands, EXT, 2, layer)] == [11]


def test_the_moved_extinguisher_is_claimed_by_the_one_that_went_missing():
    layer, reg = prior_map()
    layer.add(track(20, C, frames=(30, 31)))
    reg.update(layer, step=31)
    layer.get(10).presence = PresenceState(log_odds=_logodds(0.2))   # looked at A: gone
    events = reg.update(layer, step=40)
    assert reg.index_of(20) == 1
    assert reg.get(EXT, 1).track_id == 20 and reg.get(EXT, 2).track_id == 11
    [moved] = [e for e in events if e["event"] == "moved"]
    assert moved["from_track"] == 10 and moved["to_track"] == 20
    assert moved["from_xy"] == [A[0], A[1]] and moved["to_xy"] == [C[0], C[1]]
    # The old place is no longer #1, and #2 is never offered for #1.
    assert reg.index_of(10) is None
    cands = [layer.get(11), layer.get(20)]
    assert [t.id for t in reg.order_for(cands, EXT, 1, layer)] == [20]
    assert [t.id for t in reg.order_for(cands, EXT, 2, layer)] == [11]


def test_an_extinguisher_beside_the_second_one_is_still_the_first_one():
    layer, reg = prior_map()
    layer.get(10).presence = PresenceState(log_odds=_logodds(0.2))
    beside = (B[0] + 0.65, B[1])
    layer.add(track(21, beside, frames=(50, 51)))
    reg.update(layer, step=51)
    assert reg.index_of(21) == 1 and reg.get(EXT, 2).track_id == 11


def test_an_estimate_drifting_onto_the_second_one_does_not_merge_them_when_seen_together():
    layer, reg = prior_map()
    layer.get(10).presence = PresenceState(log_odds=_logodds(0.2))
    layer.get(11).observations.append(SimpleNamespace(frame_id=60))
    layer.add(track(21, (B[0] + 0.3, B[1]), frames=(60, 61)))   # inside fragment_m of #2
    reg.update(layer, step=61)
    assert reg.index_of(21) == 1 and reg.get(EXT, 2).track_id == 11


def _move_track(t, xy):
    t.ellipsoid.center = np.array([xy[0], 0.3, -xy[1]])


def test_the_second_ones_track_pulled_toward_the_first_does_not_give_away_its_place():
    """outputs/20261113_154556: #1 put 0.65 m from #2 and seen alone joined #2's
    track, whose refit pulled it 0.52 m toward #1. #2's own detections then began a
    fresh track at #2's place and #1's a third at its own; nothing explained the one
    at #2's place, so missing #1 claimed it and #2 was inspected as #1."""
    layer, reg = prior_map()
    _move_track(layer.get(11), (B[0] + 0.51, B[1] - 0.11))          # drifted off B
    layer.get(11).observations.append(SimpleNamespace(frame_id=30))
    layer.add(track(53, (B[0] + 0.64, B[1] + 0.05), frames=(31, 32)))  # where #1 stands
    layer.add(track(55, (B[0] + 0.15, B[1]), frames=(40, 41)))         # at #2's place
    layer.get(10).presence = PresenceState(log_odds=_logodds(0.06))   # A looked at: empty
    events = reg.update(layer, step=41)
    assert reg.get(EXT, 2).track_id == 55
    assert reg.get(EXT, 1).track_id == 53
    assert [e["event"] for e in events] == ["reanchored", "moved"]
    assert events[0]["from_track"] == 11 and events[0]["drift_m"] > 0.4
    # The inspection's views follow: #1's are its own, #2's are not #1's.
    assert reg.belongs(np.array([B[0] + 0.64, 0.3, -(B[1] + 0.05)]), 53, layer, 1.0)
    assert not reg.belongs(np.array([B[0], 0.3, -B[1]]), 53, layer, 1.0)


def test_the_track_a_drifted_instance_leaves_is_not_a_third_extinguisher():
    """outputs/20261113_160353: after #2 was re-anchored and #1 found, the track
    #2 left -- between the two, fed by both -- was indexed "#3"."""
    layer, reg = prior_map()
    _move_track(layer.get(11), (B[0] + 0.51, B[1] - 0.11))
    layer.get(11).observations += [SimpleNamespace(frame_id=f) for f in (31, 40)]   # beside both
    layer.add(track(53, (B[0] + 0.64, B[1] + 0.05), frames=(31, 32)))
    layer.add(track(55, (B[0] + 0.15, B[1]), frames=(40, 41)))
    layer.get(10).presence = PresenceState(log_odds=_logodds(0.06))
    reg.update(layer, step=41)
    reg.update(layer, step=42)
    assert [i.track_id for i in reg.instances(EXT)] == [53, 55]
    assert reg.index_of(11) is None
    cands = [layer.get(11), layer.get(53), layer.get(55)]
    assert [t.id for t in reg.order_for(cands, EXT, 1, layer)] == [53]


def test_a_drifted_instance_takes_back_its_own_detections_near_its_place():
    """outputs/20261113_162238: #2's track pulled 0.46 m off its place, #2's own
    detections began a track 0.24 m from it, and with every instance accounted
    for that was indexed "#3"."""
    layer, reg = prior_map()
    for t in layer.tracks():
        t.observations.append(SimpleNamespace(frame_id=20))
    _move_track(layer.get(11), (B[0] + 0.46, B[1] - 0.09))
    layer.add(track(73, (B[0] - 0.24, B[1] + 0.01), frames=(40, 41)))
    events = reg.update(layer, step=41)
    assert reg.get(EXT, 2).track_id == 73
    assert [e["event"] for e in events] == ["reanchored"]
    assert len(reg.instances(EXT)) == 2 and reg.index_of(11) is None


def test_a_track_nearer_than_the_drifted_one_but_not_at_the_place_is_not_taken():
    layer, reg = prior_map()
    _move_track(layer.get(11), (B[0] + 0.3, B[1]))
    layer.add(track(74, (B[0] - 0.45, B[1]), frames=(40, 41)))   # beyond fragment_m of B
    reg.update(layer, step=41)
    assert reg.get(EXT, 2).track_id == 11


def test_a_track_settling_with_nothing_else_at_its_place_keeps_its_instance():
    layer, reg = prior_map()
    _move_track(layer.get(11), (B[0] + 0.5, B[1]))   # refined 0.5 m: nothing at B now
    events = reg.update(layer, step=5)
    assert reg.get(EXT, 2).track_id == 11 and events == []


def test_a_drifted_instance_never_takes_another_instances_track():
    layer, reg = prior_map()
    _move_track(layer.get(11), (B[0] + 0.5, B[1]))
    _move_track(layer.get(10), (B[0] + 0.1, B[1]))   # #1's own track, now at B
    reg.update(layer, step=5)
    assert reg.get(EXT, 2).track_id == 11 and reg.get(EXT, 1).track_id == 10


def test_a_second_track_on_the_one_just_found_is_that_one():
    """outputs/20261113_175735: #1 claimed at the chairs, the stop on the region
    proposer's track 0.15 m from it."""
    layer, reg = prior_map()
    layer.add(track(20, C, frames=(30, 31)))
    layer.get(10).presence = PresenceState(log_odds=_logodds(0.2))
    reg.update(layer, step=40)
    assert reg.get(EXT, 1).track_id == 20
    prop = layer.add(track(1000001, (C[0] + 0.15, C[1]), frames=(41, 42)))
    prop.n_proposal_obs = 2                        # every observation a region proposal
    assert prop.proposal_only and reg.index_of(prop.id) is None
    assert reg.same_as(prop, EXT, 1, layer) and not reg.same_as(prop, EXT, 2, layer)
    beside_two = layer.add(track(1000002, (B[0] + 0.15, B[1]), frames=(42,)))
    assert not reg.same_as(beside_two, EXT, 1, layer)
    assert reg.same_as(layer.get(11), EXT, 2, layer) and not reg.same_as(layer.get(11), EXT, 1, layer)


def test_a_sign_high_on_the_wall_is_never_an_extinguisher():
    """outputs/20261113_184240: the hospital's FIRE EXTINGUISHER wall signs were
    detected as extinguishers, tracks centred 1.36-1.51 m up; missing #1 claimed
    one as its new place."""
    layer = Layer(track(10, A, frames=(-900, -899)), track(11, B, frames=(-880, -879)))
    reg = InstanceRegistry([EXT], fragment_m=0.4, missing_p=0.45, min_obs=2, max_height_m=1.0)
    reg.floor_y = 0.0
    reg.bootstrap(layer)
    sign = layer.add(track(88, (-23.07, 5.28), frames=(30, 31, 32, 33)))
    sign.ellipsoid.center[1] = 1.37
    floor = layer.add(track(89, C, frames=(40, 41)))
    layer.get(10).presence = PresenceState(log_odds=_logodds(0.2))
    events = reg.update(layer, step=41)
    assert reg.get(EXT, 1).track_id == 89 and reg.index_of(88) is None
    assert [e["to_track"] for e in events if e["event"] == "moved"] == [89]
    assert reg.plausible([sign, floor], layer) == [floor]
    assert reg.claim(EXT, 2, 88, layer) is None
    assert [t.id for t in reg.order_for([sign, floor], EXT, 1, layer)] == [89]
    off = InstanceRegistry([EXT], max_height_m=0.0)
    off.floor_y = 0.0
    assert off.plausible([sign], layer) == [sign]


def test_a_believed_instance_is_never_rebound():
    layer, reg = prior_map()
    for t in layer.tracks():                       # both checked today, both there
        t.observations.append(SimpleNamespace(frame_id=70))
    layer.add(track(22, C, frames=(71, 72)))
    reg.update(layer, step=72)
    assert reg.index_of(22) == 3
    assert reg.get(EXT, 1).track_id == 10 and reg.get(EXT, 2).track_id == 11


def test_a_missing_instance_with_a_believed_fragment_is_re_anchored_not_moved():
    layer, reg = prior_map()
    layer.add(track(23, (A[0] + 0.15, A[1]), frames=(80, 81)))
    reg.update(layer, step=81)
    assert reg.index_of(23) == 1                    # a fragment of #1
    layer.get(10).presence = PresenceState(log_odds=_logodds(0.2))
    events = reg.update(layer, step=82)
    assert reg.get(EXT, 1).track_id == 23
    assert [e["event"] for e in events] == ["reanchored"]


def test_instances_survive_a_save_and_load():
    layer, reg = prior_map()
    layer.add(track(20, C, frames=(30, 31)))
    layer.get(10).presence = PresenceState(log_odds=_logodds(0.2))
    reg.update(layer, step=40)
    again = registry()
    assert again.load_dict(reg.to_dict(), layer) == 2
    assert again.index_of(20) == 1 and again.index_of(11) == 2
    assert again.get(EXT, 1).moves[0]["from_track"] == 10


def test_other_classes_are_left_alone():
    layer, reg = Layer(track(5, A, label="chair")), registry()
    reg.update(layer)
    assert reg.instances() == [] and reg.name_of(layer.get(5)) == "chair"


def test_a_target_names_its_instance_with_a_hash():
    assert split_target("fire extinguisher #2") == ("fire extinguisher", 2)
    assert split_target("fire extinguisher") == ("fire extinguisher", 0)
    assert split_target("#3") == ("#3", 0)


def test_instance_labels_are_never_linked_into_one_object():
    a = track(1, B, frames=(1, 2))
    b = track(2, (B[0] + 0.65, B[1]), frames=(1, 2))   # seen together, 0.65 m apart
    relink([a, b], link_dist_m=1.0, max_frame_gap=50)
    assert b.id in a.linked_ids                         # what linking does to a sofa
    relink([a, b], link_dist_m=1.0, max_frame_gap=50, skip_labels={EXT})
    assert not a.linked_ids and not b.linked_ids


def _pt(xy, h=0.3):
    return np.array([xy[0], h, -xy[1]])


def test_seeing_the_second_one_is_not_seeing_the_first():
    """Walking up to where #1 was, #2 two metres on is in the frame. That
    sighting must not count as #1, or the absence reading never happens and
    the robot stops at the empty place."""
    layer, reg = prior_map()
    assert reg.belongs(_pt((A[0] + 0.2, A[1] - 0.1)), 10, layer, match_m=1.0)   # #1, where it was
    assert not reg.belongs(_pt(B), 10, layer, match_m=1.0)                    # #2
    assert not reg.belongs(_pt(C), 10, layer, match_m=1.0)                    # far from #1
    # Within reach of #1's track, but nearer #2's: it is #2.
    between = (A[0] - 1.2, A[1])
    assert reg.belongs(_pt(between), 11, layer, match_m=1.5)
    assert not reg.belongs(_pt(between), 10, layer, match_m=1.5)
    assert not reg.belongs(None, 10, layer, match_m=1.0)                      # no depth: not counted


def test_the_inspection_judges_the_views_of_the_one_it_reached():
    from osg.inspection import TargetViewBuffer

    class Det:
        def __init__(self, box, score):
            self.label, self.score, self.bbox_xyxy, self.mask = EXT, score, np.array(box, float), None

    def frame_at(depth_m):
        # Camera at the origin looking down +z (OpenCV): a box's centre column
        # u and depth d place it at x = (u - cx) / fx * d, z = d.
        K = SimpleNamespace(fx=500.0, fy=500.0, cx=360.0, cy=640.0)
        depth = np.full((1280, 720), depth_m, dtype=np.float32)
        return SimpleNamespace(depth=depth, intrinsics=K, T_wc=np.eye(4))

    buf = TargetViewBuffer(EXT, keep=2)
    rgb = np.zeros((1280, 720, 3), np.uint8)
    # Three big, confident views of an extinguisher 2 m away, one small one of
    # another 4 m away: by quality alone the far one is evicted.
    for step in (1, 2, 3):
        buf.observe(rgb, [Det([260, 300, 460, 1000], 0.95)], step, frame=frame_at(2.0))
    buf.observe(rgb, [Det([340, 500, 380, 700], 0.6)], 4, frame=frame_at(4.0))
    near = buf.best(5)
    far_xyz = np.array([0.0, 0.0, 4.0])
    picked = buf.best(5, near=far_xyz, max_m=0.5)
    assert [v.step for v in picked] == [4]                  # kept, and the only view of it
    assert len(near) == 3                                   # 2 of the near one + 1 of the far


def test_the_views_of_the_one_beside_it_are_not_judged():
    from osg.inspection import TargetView, TargetViewBuffer

    layer, reg = prior_map()
    layer.get(10).presence = PresenceState(log_odds=_logodds(0.2))
    layer.add(track(21, (B[0] + 0.65, B[1]), frames=(50, 51)))      # #1, moved beside #2
    reg.update(layer, step=51)
    buf = TargetViewBuffer(EXT, keep=6)
    rgb = np.zeros((10, 10, 3), np.uint8)
    for step, xy, q in ((1, B, 0.9), (2, (B[0] + 0.65, B[1]), 0.5)):   # #2's view is the better one
        buf.views.append(TargetView(rgb=rgb, bbox_xyxy=np.zeros(4), score=q, step=step,
                                    truncated=False, quality=q, xyz=_pt(xy)))
    accept = lambda xyz: reg.belongs(xyz, 21, layer, 1.0)
    assert [v.step for v in buf.best(2, accept=accept)] == [2]
    assert [v.step for v in buf.best(2, near=layer.center_of(layer.get(21)), max_m=1.0)] == [1, 2]


def test_an_extinguisher_put_down_beside_another_does_not_take_its_track():
    """Matched in 3D, nearest first: #2's own detection keeps #2's track even
    when #2's estimate leans toward the newcomer, and the newcomer starts its
    own track instead of pulling #2's toward it."""
    from osg.objects.association import DataAssociator

    K = SimpleNamespace(fx=500.0, fy=500.0, cx=360.0, cy=640.0, K=lambda: np.array(
        [[500.0, 0, 360.0], [0, 500.0, 640.0], [0, 0, 1]]))
    frame = SimpleNamespace(depth=np.full((1280, 720), 2.9, np.float32), intrinsics=K,
                            T_wc=np.eye(4), T_cw=np.eye(4))

    def det(x):                     # a box whose depth places it at (x, 0, 2.9)
        u = 360.0 + x * 500.0 / 2.9
        return SimpleNamespace(label=EXT, score=0.9, mask=None,
                               bbox_xyxy=np.array([u - 20, 500, u + 20, 780]))

    two = ObjectTrack(id=32, label=EXT, ellipsoid=Ellipsoid(
        center=np.array([-0.05, 0.0, 3.0]), axes=np.array([0.15, 0.3, 0.15]), R=np.eye(3)))
    assoc = DataAssociator(category_gate=True)
    seen_both = dict(assoc.associate([det(0.35), det(-0.3)], frame, [two],
                                     xyz_labels={EXT}, xyz_gate_m=0.45))
    assert seen_both == {0: None, 1: 32}                   # #2 keeps its track
    alone = dict(assoc.associate([det(0.6)], frame, [two], xyz_labels={EXT}, xyz_gate_m=0.45))
    assert alone == {0: None}                              # out of reach: a track of its own


def test_the_context_frame_outlines_the_one_being_judged():
    from osg.inspection.condition import _outline, build_prompt

    rgb = np.zeros((640, 360, 3), np.uint8)
    out = _outline(rgb, [100, 200, 180, 500])
    assert out[200, 140].tolist() == [0, 220, 0] and out[350, 100].tolist() == [0, 220, 0]
    assert out[350, 140].tolist() == [0, 0, 0] and not rgb.any()      # inside clear; input untouched
    assert "outlined in green" in build_prompt(EXT, [], 2)[1]


def test_the_approach_stops_short_to_look_when_the_one_it_wants_is_not_there():
    """With the object gone, nothing stops an approach whose goal is the
    object's centre until the robot is at the wall where it stood -- in Isaac
    the Stretch pushed into it and fell over. At `terminal_stop_m` from the
    track, unseen, the approach takes its reading there instead."""
    from osg.agent.approach import ApproachPolicy

    nav = SimpleNamespace(cfg=SimpleNamespace(agent=SimpleNamespace(terminal_stop_m=1.4)),
                          instances=registry(), target=EXT,
                          _target_obj_xy=np.array([-7.5, -5.29]))
    ap = ApproachPolicy(nav)
    assert not ap._at_standoff_unseen(np.array([-7.5, -3.0]))     # 2.3 m out: keep going
    assert ap._at_standoff_unseen(np.array([-7.5, -4.0]))         # 1.3 m, never seen: look
    ap.last_good_xy = np.array([-7.5, -3.5])                      # seen on the way: depth stop's job
    assert not ap._at_standoff_unseen(np.array([-7.5, -4.0]))
    ap.last_good_xy, ap.standoff_checked = None, True             # once per approach
    assert not ap._at_standoff_unseen(np.array([-7.5, -4.0]))
    nav.instances, ap.standoff_checked = None, False              # not an instance class: as before
    assert not ap._at_standoff_unseen(np.array([-7.5, -4.0]))


def test_a_missing_instance_can_be_claimed_on_evidence_at_the_stop_but_never_a_held_one():
    layer, reg = prior_map()
    prop = layer.add(track(1000001, C, frames=(90, 91)))
    prop.n_proposal_obs = 2                     # the region proposer's track: the rules skip it
    reg.update(layer, step=91)
    assert reg.index_of(1000001) is None
    assert reg.claim(EXT, 1, 1000001, layer, step=95) is None        # #1 not missing yet
    layer.get(10).presence = PresenceState(log_odds=_logodds(0.2))
    ev = reg.claim(EXT, 1, 1000001, layer, step=96, why="named by the detector at the stop")
    assert ev["event"] == "moved" and reg.index_of(1000001) == 1 and reg.get(EXT, 2).track_id == 11
    assert reg.claim(EXT, 2, 1000001, layer) is None                   # #2 is not missing


def _approaching(layer, reg, committed, k=1):
    """A nav agent mid-approach to `committed` for "#k" (CandidatePolicy's view of it)."""
    from osg.agent.candidate import CandidatePolicy
    from osg.agent.state import State

    started = []
    nav = SimpleNamespace(instances=reg, target=EXT, target_instance=k, object_layer=layer,
                          state=State.APPROACH, _candidate_id=committed, _target_obj_xy=None,
                          _goal_xy=None, _current_path=None, stats={},
                          _start_approach=lambda xy: started.append(np.asarray(xy, float)))
    policy = CandidatePolicy(nav)
    policy._log_goal_commit = lambda track: None
    return nav, policy, started


def test_an_approach_to_a_track_that_turns_out_to_be_the_second_one_is_dropped():
    """outputs/20261113_200152: sent for #1, #1 gone from its place, the agent
    committed to the track at #2's place -- #2's own had drifted 0.36 m off it, so
    nothing held it -- and two steps later the registry re-anchored #2 onto it.
    The approach drove on and #2 was inspected for the #1 mission."""
    from osg.agent.state import State

    layer, reg = prior_map()
    _move_track(layer.get(11), (B[0] + 0.51, B[1] - 0.11))          # #2's track drifted
    layer.get(11).observations.append(SimpleNamespace(frame_id=30))
    layer.get(10).presence = PresenceState(log_odds=_logodds(0.06))   # #1's place empty
    layer.add(track(55, (B[0] + 0.15, B[1]), frames=(40, 41)))         # at #2's place
    nav, policy, started = _approaching(layer, reg, committed=55)
    before = layer.get(55).presence.log_odds
    reg.update(layer, step=41)
    assert reg.get(EXT, 2).track_id == 55
    policy.follow_registry()
    assert nav.state is State.EXPLORE and nav._candidate_id is None and not started
    assert layer.get(55).presence.log_odds == before                  # nothing held against #2
    assert nav.stats["approach_dropped_other_instance"] == 1


def test_an_approach_follows_the_instance_onto_the_track_the_registry_chose():
    """outputs/20261113_203830: committed to a track by the moved #1, then #1 was
    claimed on its neighbour 0.56 m away; the approach stopped at the first, a "?"."""
    layer, reg = prior_map()
    layer.get(10).presence = PresenceState(log_odds=_logodds(0.06))
    layer.add(track(146, (C[0], C[1] - 0.56), frames=(70, 71)))
    layer.add(track(149, C, frames=(72, 73)))
    nav, policy, started = _approaching(layer, reg, committed=146)
    reg.claim(EXT, 1, 149, layer, step=73)
    policy.follow_registry()
    assert nav._candidate_id == 149
    assert np.allclose(started[-1], [C[0], -C[1]])                     # pipeline (x, z = -y)
    policy.follow_registry()                                           # settled: nothing more
    assert len(started) == 1


def test_an_approach_to_the_ones_own_track_or_for_any_one_is_left_alone():
    layer, reg = prior_map()
    nav, policy, started = _approaching(layer, reg, committed=10)
    policy.follow_registry()
    assert nav._candidate_id == 10 and not started
    nav, policy, started = _approaching(layer, reg, committed=11, k=0)  # "a fire extinguisher"
    policy.follow_registry()
    assert nav._candidate_id == 11 and not started


def test_the_loopback_robot_backs_up():
    from osg.ros2.loopback import LoopbackRobot, parse_args

    robot = LoopbackRobot(parse_args(["--width", "32", "--height", "24"]))
    start = robot.pose[:2].copy()
    robot.execute("move_forward", 0.5, 0.0)
    robot.execute("move_backward", 0.5, 0.0)
    assert np.allclose(robot.pose[:2], start, atol=1e-6)
