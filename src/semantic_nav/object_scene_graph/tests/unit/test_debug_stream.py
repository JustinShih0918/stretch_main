"""DebugStream: the simulator's debug view pushed through the bridge.

Two properties matter more than the pixels. It publishes the rotated frame and
the panel on the /osg/* topics scripts/ros2/osg.rviz shows, and a bridge that
refuses an image switches the stream off rather than ending the run -- a debug
view must never cost a step on a robot.
"""
from types import SimpleNamespace

import numpy as np

from osg.eval.debug_stream import DebugStream
from osg.mapping.costmap import Costmap2D


class _Transport:
    def __init__(self, fail=False):
        self.calls, self.markers, self.fail = [], [], fail

    def publish_image(self, topic, image, encoding):
        if self.fail:
            raise ConnectionError("bridge gone")
        self.calls.append((topic, np.asarray(image).shape, encoding))

    def publish_markers(self, topic, markers, frame_id):
        if self.fail:
            raise ConnectionError("bridge gone")
        self.markers.append((topic, frame_id, list(markers)))

    def publish_grid(self, topic, grid, origin, resolution, frame_id, z):
        if self.fail:
            raise ConnectionError("bridge gone")


class _NoDetections:
    def detect(self, rgb):
        return []


DET = _NoDetections()


def _env(tr):
    return SimpleNamespace(transport=tr, cfg=SimpleNamespace())


def _frame(h=64, w=48):
    return SimpleNamespace(rgb=np.zeros((h, w, 3), np.uint8),
                           camera_position=np.array([1.0, 1.3, 2.0]))


def _panel_agent():
    return SimpleNamespace(costmap=Costmap2D(resolution=0.1, size_m=4.0))


def test_publishes_the_rotated_frame_and_the_panel():
    tr = _Transport()
    stream = DebugStream(_env(tr))
    stream.write(_frame(), _panel_agent(), "cup", detector=DET)
    topics = [c[0] for c in tr.calls]
    assert topics == [DebugStream.RGB_TOPIC, DebugStream.DET_TOPIC, DebugStream.PANEL_TOPIC]
    assert tr.calls[0][1:] == ((64, 48, 3), "rgb8")   # the frame the pipeline consumes, as-is
    assert tr.calls[2][2] == "bgr8"                    # cv2-drawn panel is BGR


def test_a_refusing_bridge_turns_the_stream_off_without_raising():
    tr = _Transport(fail=True)
    stream = DebugStream(_env(tr))
    stream.write(_frame(), _panel_agent(), "cup", detector=DET)   # must not raise
    assert stream._dead is True
    tr.fail = False
    stream.write(_frame(), _panel_agent(), "cup", detector=DET)
    assert tr.calls == []   # stays off: one failure is a bridge that cannot take images


def test_the_stream_is_only_a_sink():
    # Nothing to close, nothing to flush: the pictures already went to RViz.
    assert DebugStream(_env(_Transport())).close() is None


class _Track:
    def __init__(self, id, label, centre, p, blacklisted=False):
        self.id, self.label, self.blacklisted = id, label, blacklisted
        self.ellipsoid = SimpleNamespace(center=np.array(centre, float), axes=np.array([0.2, 0.1, 0.3]))
        self.presence = SimpleNamespace(p=p)


def _graph_agent():
    layer = SimpleNamespace(tracks=lambda include_blacklisted=False: [
        _Track(3, "cup", (1.0, 0.8, 2.0), 0.9), _Track(4, "chair", (0.0, 3.4, -1.0), 0.2, blacklisted=True)])
    return SimpleNamespace(
        costmap=Costmap2D(resolution=0.1, size_m=4.0),
        object_layer=layer,
        scene_graph=SimpleNamespace(
            containers={7: SimpleNamespace(id=7, label="table", center=np.array([1.0, 0.7, 2.0]))},
            rooms={1: SimpleNamespace(id=1, label="kitchen", centroid_xy=np.array([1.0, 2.0]))}),
        _current_path=np.array([[0.0, 0.0], [1.0, 2.0]]), _goal_xy=np.array([1.0, 2.0]),
        _current_frontier=SimpleNamespace(centroid_xy=np.array([1.5, 2.5])), state="EXPLORE")


def test_the_scene_graph_goes_out_as_markers_in_the_map_frame():
    tr = _Transport()
    DebugStream(_env(tr)).write(_frame(), _graph_agent(), "cup", detector=DET)
    (topic, frame_id, ms), = tr.markers
    assert topic == DebugStream.GRAPH_TOPIC and frame_id == "map"
    by_ns = {}
    for m in ms:
        by_ns.setdefault(m["ns"], []).append(m)
    assert set(by_ns) >= {"objects", "labels", "containers", "rooms", "path", "frontier", "goal", "status"}
    cup = next(m for m in by_ns["objects"] if m["id"] == 3)
    # pipeline (x, y-up, z) = (1.0, 0.8, 2.0) -> ROS (x, -z, y) = (1.0, -2.0, 0.8)
    assert cup["xyz"] == (1.0, -2.0, 0.8)
    assert cup["rgba"][1] > cup["rgba"][0]        # believed: more green than red
    chair = next(m for m in by_ns["objects"] if m["id"] == 4)
    assert chair["rgba"][:3] == (0.5, 0.5, 0.5)   # blacklisted: grey


def _logging_agent(**logs):
    agent = _graph_agent()
    for k, v in logs.items():
        setattr(agent, k, v)
    return agent


def test_stream_jsonl_carries_new_agent_log_entries_each_step(tmp_path):
    import json

    tr = _Transport()
    agent = _logging_agent(state_log=[(0, "explore")], frontier_select_log=[])
    stream = DebugStream(_env(tr), log_path=tmp_path / "stream.jsonl")
    stream.write(_frame(), agent, "cup", detector=DET)
    agent.state_log.append((3, "approach"))
    agent.frontier_select_log.append({"step": 3, "xy": np.array([1.0, 2.0]), "score": np.float32(0.5)})
    stream.write(_frame(), agent, "cup", detector=DET)
    stream.write(_frame(), agent, "cup", detector=DET)
    stream.close()

    lines = [json.loads(l) for l in (tmp_path / "stream.jsonl").read_text().splitlines()]
    assert [l["step"] for l in lines] == [1, 2, 3]
    assert lines[0]["state_log"] == [[0, "explore"]]
    assert lines[1]["state_log"] == [[3, "approach"]]          # only what was added
    assert lines[1]["frontier_select_log"][0]["xy"] == [1.0, 2.0]   # numpy made JSON
    assert "state_log" not in lines[2]                          # nothing new: no key


def test_log_keeps_coming_after_the_bridge_refuses_markers(tmp_path):
    tr = _Transport()
    orig = tr.publish_markers

    def refuse(*a):
        raise ConnectionError("bridge gone")
    tr.publish_markers = refuse
    stream = DebugStream(_env(tr), log_path=tmp_path / "s.jsonl")
    stream.write(_frame(), _graph_agent(), "cup", detector=DET)
    assert stream._dead is True
    stream.write(_frame(), _graph_agent(), "cup", detector=DET)
    stream.close()
    assert len((tmp_path / "s.jsonl").read_text().splitlines()) == 2
    tr.publish_markers = orig


def test_on_step_is_called_with_the_step_and_may_not_end_the_run():
    seen = []

    def hook(step):
        seen.append(step)
        if step == 2:
            raise RuntimeError("disk full")
    stream = DebugStream(_env(_Transport()), on_step=hook)
    for _ in range(3):
        stream.write(_frame(), _panel_agent(), "cup", detector=DET)
    assert seen == [1, 2, 3]


def test_a_pose_jump_is_said_out_loud_and_yaw_is_logged(tmp_path, capsys):
    import json

    stream = DebugStream(_env(_Transport()), log_path=tmp_path / "s.jsonl")
    f1 = _frame(); f1.T_wc = np.eye(4); f1.T_wc[:3, 3] = [0.0, 1.3, 0.0]; f1.camera_position = np.array([0.0, 1.3, 0.0])
    f2 = _frame(); f2.T_wc = np.eye(4); f2.T_wc[:3, 3] = [4.0, 1.3, -3.0]; f2.camera_position = np.array([4.0, 1.3, -3.0])
    stream.write(f1, _graph_agent(), "cup", detector=DET)
    stream.write(f2, _graph_agent(), "cup", detector=DET)
    stream.close()
    assert "POSE JUMP: 5.00 m" in capsys.readouterr().out
    lines = [json.loads(l) for l in (tmp_path / "s.jsonl").read_text().splitlines()]
    assert lines[0]["yaw_deg"] == -90.0   # identity T_wc looks down pipeline +z = ROS -y
