"""Map snapshots: the map survives the episode that built it (Phase 2).

The dynamic benchmark is two passes -- explore the static layout, move the
objects, then navigate the moved world WITH THE OLD MAP. The staleness is the
experiment, so it is the snapshot that has to be faithful: a track that loses
its observations is a candidate the pipeline silently ignores, and a belief that
resets to 0.82 erases the evidence the whole comparison is about.

Habitat-free: a snapshot is JSON plus grids.
"""
import numpy as np
import pytest

from osg.graph.map_store import MapStoreError, apply_map, load_map, save_map
from osg.mapping.costmap import FREE, OCCUPIED, UNKNOWN, Costmap2D
from osg.mapping.map_layer import MapLayer
from osg.mapping.value_map import ValueMap2D
from osg.objects.association import Observation, ObjectTrack
from osg.objects.ellipsoid import Ellipsoid
from osg.objects.object_layer import ObjectLayer


class _Agent:
    def __init__(self, object_layer, costmap, room_labels=None, scene_graph=None):
        self.object_layer = object_layer
        self.map = MapLayer(costmap=costmap, segmenter=None, room_labels=room_labels)
        self.scene_graph = scene_graph or _NullGraph()

    @property
    def costmap(self):
        return self.map.costmap

    @property
    def _room_labels(self):
        return self.map.room_labels

    @_room_labels.setter
    def _room_labels(self, value):
        self.map.room_labels = value


class _NullGraph:
    def __init__(self):
        self.rebuilt = 0

    def rebuild(self, *a, **kw):
        self.rebuilt += 1


def track(tid=1, label="mug", center=(1.0, 0.8, 2.0), n_obs=3):
    t = ObjectTrack(
        id=tid, label=label,
        ellipsoid=Ellipsoid(center=np.array(center, float),
                            axes=np.array([0.1, 0.2, 0.3]),
                            R=np.eye(3)),
    )
    t.best_score = 0.77
    t.best_bbox_px = 4321.0
    t.evidence = 2.5
    t.best_cam_xy = np.array([0.5, 0.25])
    t.first_cam_xy = np.array([0.1, 0.2])
    for i in range(n_obs):
        t.observations.append(
            Observation(frame_id=i, mu=np.array([10.0 + i, 20.0]),
                        cov=np.eye(2) * 3.0, K=np.eye(3), T_cw=np.eye(4),
                        mean_depth=1.5 + i)
        )
    return t


def agent_with(tracks, *, size_m=20.0, room_labels=None):
    layer = ObjectLayer()
    for t in tracks:
        layer._tracks[t.id] = t
    layer._next_id = max((t.id for t in tracks), default=-1) + 1
    cm = Costmap2D(resolution=0.05, size_m=size_m)
    return _Agent(layer, cm, room_labels)


def roundtrip(agent, tmp_path, into=None):
    save_map(tmp_path / "m.json", agent, scene="s1", layout_id="static")
    blob = load_map(tmp_path / "m.json")
    target = into or agent_with([])
    apply_map(target, blob)
    return target


# ---------------------------------------------------------------- fidelity


def test_tracks_survive_with_geometry_and_score(tmp_path):
    out = roundtrip(agent_with([track(1), track(2, label="bowl")]), tmp_path)
    restored = {t.id: t for t in out.object_layer.tracks()}
    assert set(restored) == {1, 2}
    assert restored[2].label == "bowl"
    assert restored[1].ellipsoid.center == pytest.approx([1.0, 0.8, 2.0])
    assert restored[1].ellipsoid.axes == pytest.approx([0.1, 0.2, 0.3])
    assert restored[1].best_score == pytest.approx(0.77)


def test_observations_survive_so_candidates_still_qualify(tmp_path):
    """n_obs gates candidate selection; a snapshot that drops observations
    loads tracks the pipeline then refuses to propose."""
    out = roundtrip(agent_with([track(1, n_obs=4)]), tmp_path)
    restored = out.object_layer.get(1)
    assert restored.n_obs == 4
    assert restored.observations[2].mean_depth == pytest.approx(3.5)
    assert restored.observations[0].T_cw.shape == (4, 4)
    assert out.object_layer.candidates("mug", min_obs=3) == [restored]


def test_presence_belief_survives(tmp_path):
    """The whole experiment is about what the map still believes. Resetting the
    belief on load would erase exactly the thing being measured."""
    t = track(1)
    t.presence.log_odds = -3.5
    t.presence.n_missed = 7
    t.presence.last_seen_kf = 42
    out = roundtrip(agent_with([t]), tmp_path)
    restored = out.object_layer.get(1)
    assert restored.presence.log_odds == pytest.approx(-3.5)
    assert restored.presence.n_missed == 7
    assert restored.presence.p < 0.05


def test_links_survive(tmp_path):
    """Linking reunites fragments of one object that a single ellipsoid cannot
    cover; losing it splits an L-shaped sofa back into two tracks."""
    a, b = track(1), track(2)
    a.linked_ids, b.linked_ids = {2}, {1}
    out = roundtrip(agent_with([a, b]), tmp_path)
    assert out.object_layer.get(1).linked_ids == {2}


def test_the_blacklist_does_not_survive_a_reload(tmp_path):
    """It is an EPISODE-scoped device -- "this attempt already tried that
    candidate" -- and persisting it makes an episode's rejection a permanent
    strike-off in every later session. Measured: one track in a 587-track map was
    saved blacklisted, and it was the pitcher's only correct track (0.00 m from
    the authored pose, score 0.68, belief 0.82), which made every pitcher episode
    of the next benchmark unwinnable before it started. No state may be
    absorbing; `min_presence` over a belief is the recoverable version of this.
    """
    a = track(1)
    a.blacklisted = True
    out = roundtrip(agent_with([a, track(2)]), tmp_path)
    assert out.object_layer.get(1).blacklisted is False
    assert [t.id for t in out.object_layer.tracks()] == [1, 2]


def test_next_track_id_does_not_collide_after_load(tmp_path):
    """A reused id would merge a new object into an old one's history."""
    out = roundtrip(agent_with([track(7)]), tmp_path)
    assert out.object_layer._next_id > 7


def test_occupancy_and_rooms_survive(tmp_path):
    src = agent_with([track(1)])
    src.costmap.grid[3, 4] = OCCUPIED
    src.costmap.grid[5, 6] = FREE
    labels = np.zeros(src.costmap.grid.shape, dtype=np.int32)
    labels[10:20, 10:20] = 3
    src._room_labels = labels
    out = roundtrip(src, tmp_path)
    assert out.costmap.grid[3, 4] == OCCUPIED
    assert out.costmap.grid[5, 6] == FREE
    assert out.costmap.grid[0, 0] == UNKNOWN
    assert out._room_labels[15, 15] == 3


def test_scene_graph_is_rebuilt_not_restored(tmp_path):
    """Derived structure must be recomputed, or a snapshot freezes yesterday's
    container rule into today's run."""
    src = agent_with([track(1)])
    src._room_labels = np.ones(src.costmap.grid.shape, dtype=np.int32)
    out = roundtrip(src, tmp_path)
    assert out.scene_graph.rebuilt == 1


# ------------------------------------------------------------------ guards


def test_the_floor_height_survives_so_both_passes_band_alike(tmp_path):
    src = agent_with([track(1)])
    src.map.floor_y = 0.12
    save_map(tmp_path / "m.json", src, scene="s1")
    blob = load_map(tmp_path / "m.json")
    assert blob["schema_version"] == 3
    dst = agent_with([])
    apply_map(dst, blob, initial_floor_y=0.9)
    assert dst.map.floor_y == pytest.approx(0.12)


def test_optional_value_map_evidence_survives_roundtrip(tmp_path):
    src = agent_with([track(1)])
    src.map.value_map = ValueMap2D(src.costmap)
    src.map.value_map.value[4, 5] = 0.73
    src.map.value_map.conf[4, 5] = 0.91
    src.map.value_map.n_updates = 7
    out = roundtrip(src, tmp_path)
    vm = out.map.value_map
    assert vm is not None
    assert vm.value[4, 5] == pytest.approx(0.73)
    assert vm.conf[4, 5] == pytest.approx(0.91)
    assert vm.n_updates == 7


def test_schema_v1_still_loads(tmp_path):
    import json

    src = agent_with([track(1)])
    np.savez_compressed(tmp_path / "legacy.npz", grid=src.costmap.grid,
                        origin=src.costmap.origin)
    (tmp_path / "legacy.json").write_text(json.dumps({
        "schema_version": 1,
        "resolution": 0.05,
        "grids": "legacy.npz",
        "tracks": [],
    }))
    dst = agent_with([])
    apply_map(dst, load_map(tmp_path / "legacy.json"), initial_floor_y=0.3)
    assert dst.costmap.grid.shape == src.costmap.grid.shape
    # v1 recorded no floor height: the episode's own first frame stands in.
    assert dst.map.floor_y == pytest.approx(0.3)


def _v2_blob(tmp_path, floors, grids):
    import json

    np.savez_compressed(tmp_path / "v2.npz", **grids)
    (tmp_path / "v2.json").write_text(json.dumps({
        "schema_version": 2, "resolution": 0.05, "grids": "v2.npz",
        "floors": floors, "tracks": [],
    }))
    return tmp_path / "v2.json"


def test_a_single_floor_v2_snapshot_still_loads(tmp_path):
    cm = Costmap2D(resolution=0.05)
    cm.grid[7, 8] = OCCUPIED
    path = _v2_blob(
        tmp_path,
        [{"key": 4, "height_y": 0.2, "resolution": 0.05, "prefix": "floor_4_"}],
        {"floor_4_grid": cm.grid, "floor_4_origin": cm.origin},
    )
    dst = agent_with([])
    apply_map(dst, load_map(path))
    assert dst.costmap.grid[7, 8] == OCCUPIED
    assert dst.map.floor_y == pytest.approx(0.2)


def test_a_multi_floor_v2_snapshot_is_refused(tmp_path):
    cm = Costmap2D(resolution=0.05)
    path = _v2_blob(
        tmp_path,
        [{"key": 0, "height_y": 0.0, "resolution": 0.05, "prefix": "floor_0_"},
         {"key": 1, "height_y": 2.8, "resolution": 0.05, "prefix": "floor_1_"}],
        {"floor_0_grid": cm.grid, "floor_0_origin": cm.origin,
         "floor_1_grid": cm.grid, "floor_1_origin": cm.origin},
    )
    with pytest.raises(MapStoreError, match="single-floor"):
        load_map(path)


def test_corrupt_snapshot_and_missing_v2_arrays_fail_loudly(tmp_path):
    import json

    (tmp_path / "broken.json").write_text("{not json")
    with pytest.raises(MapStoreError, match="corrupt"):
        load_map(tmp_path / "broken.json")

    np.savez_compressed(tmp_path / "empty.npz")
    (tmp_path / "empty.json").write_text(json.dumps({
        "schema_version": 2,
        "grids": "empty.npz",
        "floors": [{"key": 4, "height_y": 0.0, "resolution": 0.05,
                    "prefix": "floor_4_"}],
    }))
    with pytest.raises(MapStoreError, match="no occupancy grid"):
        load_map(tmp_path / "empty.json")


def test_a_grown_costmap_loads_into_a_fresh_one(tmp_path):
    """The costmap grows as the agent explores, so a snapshot is routinely
    larger than the 20 m grid a fresh agent starts with. Refusing that would
    reject every real mapping run."""
    src = agent_with([track(1)], size_m=40.0)
    src.costmap.grid[7, 8] = OCCUPIED
    dst = agent_with([], size_m=20.0)
    save_map(tmp_path / "m.json", src, scene="s1")
    apply_map(dst, load_map(tmp_path / "m.json"))
    assert dst.costmap.grid.shape == src.costmap.grid.shape
    assert dst.costmap.grid[7, 8] == OCCUPIED


def test_resolution_mismatch_is_refused(tmp_path):
    """Different resolution IS a real incompatibility: the same array indexes
    different world coordinates."""
    save_map(tmp_path / "m.json", agent_with([track(1)]), scene="s1")
    blob = load_map(tmp_path / "m.json")
    coarse = agent_with([])
    coarse.costmap.resolution = 0.1
    with pytest.raises(MapStoreError, match="resolution"):
        apply_map(coarse, blob)


def test_missing_snapshot_names_the_path(tmp_path):
    with pytest.raises(MapStoreError, match="no map snapshot"):
        load_map(tmp_path / "absent.json")


def test_schema_version_mismatch_is_refused(tmp_path):
    import json
    save_map(tmp_path / "m.json", agent_with([track(1)]), scene="s1")
    blob = json.loads((tmp_path / "m.json").read_text())
    blob["schema_version"] = 999
    (tmp_path / "m.json").write_text(json.dumps(blob))
    with pytest.raises(MapStoreError, match="schema"):
        load_map(tmp_path / "m.json")


# ------------------------------------------------------ ghosting on reload


def test_a_carried_belief_cannot_assert_certainty(tmp_path):
    """The map was built in another session and the world had every chance to
    change. A belief restored at p=0.998 needs seven clean misses to unwind, so
    the agent commits to a stale goal on step 1 and the episode ends before the
    evidence arrives."""
    t = track(1)
    t.presence.log_odds = 6.0
    out = roundtrip(agent_with([t]), tmp_path)
    assert out.object_layer.get(1).presence.log_odds == pytest.approx(1.5)


def test_disbelief_is_carried_across_unchanged(tmp_path):
    """An object already known to be gone has not become more likely by sitting
    in a file -- the cap limits confidence, not doubt."""
    t = track(1)
    t.presence.log_odds = -4.0
    out = roundtrip(agent_with([t]), tmp_path)
    assert out.object_layer.get(1).presence.log_odds == pytest.approx(-4.0)


def test_restored_observations_are_marked_as_a_previous_session(tmp_path):
    """relink must be able to tell "seen a moment ago" from "seen before the
    world changed"; raw frame ids restart each episode and would collide."""
    out = roundtrip(agent_with([track(1, n_obs=3)]), tmp_path)
    assert all(o.frame_id < 0 for o in out.object_layer.get(1).observations)


# ---------------------------------------------------- C5: absence reporting


def test_verify_absence_reports_per_category_and_never_guesses():
    """A failed call must return None -- 'no information'. Treating a network
    error as evidence of absence would quietly delete objects."""
    import numpy as np
    from osg.verification.verifier import VLMVerifier

    class _Client:
        def __init__(self, reply):
            self.reply, self.calls = reply, 0

        def chat(self, system, user, images=None, json_response=False):
            self.calls += 1
            if isinstance(self.reply, Exception):
                raise self.reply
            return self.reply

    img = np.zeros((40, 40, 3), np.uint8)
    bbox = np.array([5.0, 5.0, 30.0, 30.0])

    v = VLMVerifier(_Client({"visible": "an empty table", "present": []}))
    assert v.verify_absence(img, bbox, ["mug"]) == {"mug": False}

    v = VLMVerifier(_Client({"visible": "a mug and a book", "present": ["mug"]}))
    assert v.verify_absence(img, bbox, ["mug", "bowl"]) == {"mug": True, "bowl": False}

    v = VLMVerifier(_Client(RuntimeError("network down")))
    assert v.verify_absence(img, bbox, ["mug"]) is None
    assert v.n_errors == 1


def test_verify_absence_truncates_the_category_list():
    """Enumerating a long list is where VLMs are least reliable."""
    import numpy as np
    from osg.verification.verifier import VLMVerifier

    captured = {}

    class _Client:
        def chat(self, system, user, images=None, json_response=False):
            captured["user"] = user
            return {"present": []}

    v = VLMVerifier(_Client())
    cats = ["mug", "bowl", "plate", "kettle", "colander", "zucchini"]
    out = v.verify_absence(np.zeros((40, 40, 3), np.uint8), np.array([1.0, 1.0, 20.0, 20.0]),
                           cats, max_categories=3)
    assert list(out) == ["mug", "bowl", "plate"]
    assert "zucchini" not in captured["user"] and "colander" not in captured["user"]


def test_verify_still_there_is_a_forced_choice_and_blocked_means_nothing():
    """Measured: asking "which categories are present" scored 11/20 because the
    model answered about plausibility, not pixels. Forced choice on a zoomed
    crop scored 17/20. "blocked" must map to None -- an obstructed view is not
    evidence of absence, and treating it as such deletes objects behind doors."""
    import numpy as np
    from osg.verification.verifier import VLMVerifier

    class _Client:
        def __init__(self, choice):
            self.choice = choice
            self.seen_user = None

        def chat(self, system, user, images=None, json_response=False):
            self.seen_user = user
            return {"seen": "a surface", "choice": self.choice}

    img = np.zeros((80, 80, 3), np.uint8)
    bbox = np.array([20.0, 20.0, 60.0, 60.0])
    assert VLMVerifier(_Client("bowl")).verify_still_there(img, bbox, "bowl") is True
    assert VLMVerifier(_Client("bare")).verify_still_there(img, bbox, "bowl") is False
    assert VLMVerifier(_Client("blocked")).verify_still_there(img, bbox, "bowl") is None
    assert VLMVerifier(_Client("")).verify_still_there(img, bbox, "bowl") is None

    c = _Client("bare")
    VLMVerifier(c).verify_still_there(img, bbox, "bowl")
    assert "bare" in c.seen_user and "blocked" in c.seen_user


def test_the_identity_crop_survives_so_the_vlm_gate_is_not_a_no_op(tmp_path):
    """`verify()` judges a candidate from a picture of it, and a restored track
    used to have neither the frame nor the crop -- so it fell through to
    `_ask(None)`, which fails OPEN. The VLM candidate gate was therefore a silent
    no-op on exactly the tracks that produce most false-positive commits (62 of
    72 in the 96-episode run came from the prior map)."""
    import numpy as np

    t = track(1)
    crop = np.zeros((40, 30, 3), dtype=np.uint8)
    crop[10:20, 5:15] = (200, 30, 30)
    t.best_crop = crop
    out = roundtrip(agent_with([t]), tmp_path)
    restored = out.object_layer.get(1).best_crop
    assert restored is not None
    assert restored.shape == crop.shape
    assert restored[15, 10, 0] > 150 and restored[0, 0, 0] < 50


def test_an_oversized_crop_is_stored_small(tmp_path):
    """A snapshot holds hundreds of tracks; the crop is a thumbnail for a VLM,
    not an archive."""
    import numpy as np

    from osg.graph.map_store import CROP_MAX_PX

    t = track(1)
    t.best_crop = np.full((900, 600, 3), 120, dtype=np.uint8)
    out = roundtrip(agent_with([t]), tmp_path)
    restored = out.object_layer.get(1).best_crop
    assert restored is not None and max(restored.shape[:2]) <= CROP_MAX_PX


def test_a_track_with_no_crop_still_round_trips(tmp_path):
    out = roundtrip(agent_with([track(1)]), tmp_path)
    assert out.object_layer.get(1).best_crop is None


# ------------------------------------------- scene graph without its occupancy

def test_scene_graph_only_keeps_tracks_but_not_occupancy(tmp_path):
    """`map_in_occupancy=false`: the objects survive, the occupancy does not,
    and this run rebuilds its own.
    """
    agent = agent_with([track(1), track(2, label="bowl")])
    agent.costmap.grid[10:20, 10:20] = 0
    agent.costmap.grid[30:34, 30:34] = 100
    save_map(tmp_path / "m.json", agent, scene="s1", layout_id="static")
    blob = load_map(tmp_path / "m.json")

    with_occ = agent_with([])
    apply_map(with_occ, blob, restore_occupancy=True)
    assert (with_occ.costmap.grid != -1).any(), "occupancy should have loaded"
    assert (with_occ.costmap.grid == 100).any()

    without = agent_with([])
    n = apply_map(without, blob, restore_occupancy=False)

    assert n == 2, "the object tracks must still load"
    assert {t.label for t in without.object_layer.tracks()} == {"mug", "bowl"}
    assert (without.costmap.grid == -1).all(), "occupancy leaked in"
