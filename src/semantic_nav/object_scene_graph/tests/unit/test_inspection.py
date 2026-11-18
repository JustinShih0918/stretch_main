"""The condition inspection that runs after the search (osg/inspection/).

What has to hold, without a model or a robot:

  the view buffer keeps the views that SHOW the target -- a whole object over
  one cut off at the image edge, a closer one over a distant one -- and only
  the target's;
  the inspector reports a failed call, an empty answer or a non-verdict as
  condition "unknown" with the reason, never as a verdict;
  the scene-graph JSON and the detections hook reach a display through the
  debug stream, and a bridge without `publish_text` switches only that off.
"""
import json
from types import SimpleNamespace

import numpy as np
import pytest

from osg.core.config import InspectionConfig
from osg.core.types import Detection
from osg.eval.debug_stream import DebugStream
from osg.inspection import (ConditionInspector, TargetViewBuffer, bearing_deg, build_prompt,
                            parse_verdict, render_card)
from osg.mapping.costmap import Costmap2D

H, W = 1280, 720


def _det(label, box, score=0.8, source="detector"):
    return Detection(label, score, np.asarray(box, float), np.zeros((H, W), bool), source=source)


def _rgb():
    return np.zeros((H, W, 3), np.uint8)


# ------------------------------------------------------------------ views

def test_keeps_only_the_targets_views():
    buf = TargetViewBuffer("fire extinguisher")
    assert buf.observe(_rgb(), [_det("chair", [100, 400, 300, 900])], 1) is None
    assert buf.observe(_rgb(), [_det("fire_extinguisher", [100, 400, 300, 900])], 2) is not None
    assert [v.step for v in buf.best(5)] == [2]
    assert buf.n_seen == 1


def test_a_proposal_is_not_a_view_of_the_target():
    buf = TargetViewBuffer("fire extinguisher")
    assert buf.observe(_rgb(), [_det("fire extinguisher", [100, 400, 300, 900], source="proposal")], 1) is None


def test_a_whole_object_beats_one_cut_off_at_the_edge():
    buf = TargetViewBuffer("fire extinguisher")
    buf.observe(_rgb(), [_det("fire extinguisher", [100, 700, 300, H - 1], score=0.95)], 1)   # base cut off
    buf.observe(_rgb(), [_det("fire extinguisher", [100, 500, 250, 900], score=0.80)], 2)
    best = buf.best(1)[0]
    assert best.step == 2 and not best.truncated


def test_closer_beats_farther_until_the_object_fills_enough_of_the_frame():
    buf = TargetViewBuffer("fire extinguisher")
    buf.observe(_rgb(), [_det("fire extinguisher", [300, 600, 340, 780])], 1)    # 14% of the height
    buf.observe(_rgb(), [_det("fire extinguisher", [250, 400, 400, 900])], 2)    # 39%
    assert buf.best(1)[0].step == 2


def test_too_small_to_judge_is_not_kept():
    buf = TargetViewBuffer("fire extinguisher", min_h_frac=0.12)
    assert buf.observe(_rgb(), [_det("fire extinguisher", [300, 600, 320, 650])], 1) is None
    assert buf.best(1) == [] and buf.n_seen == 1


def test_a_worse_view_after_better_ones_is_compared_not_crashed():
    """`observe` checks whether the new view survived the cut; with the
    dataclass's generated __eq__ that compared numpy frames and raised
    (the 2026-11-10 map take died in its final look)."""
    buf = TargetViewBuffer("fire extinguisher", keep=2)
    for step, top in enumerate([400, 300, 500, 450]):
        buf.observe(_rgb(), [_det("fire extinguisher", [200, top, 300, 900])], step)
    assert [v.step for v in buf.best(2)] == [1, 0]


def test_keeps_at_most_keep_views():
    buf = TargetViewBuffer("fire extinguisher", keep=3)
    for step in range(10):
        buf.observe(_rgb(), [_det("fire extinguisher", [200, 400, 300, 400 + 30 * step + 200])], step)
    assert len(buf.best(10)) == 3


def test_crop_pads_and_clips_to_the_frame():
    buf = TargetViewBuffer("fire extinguisher")
    view = buf.observe(_rgb(), [_det("fire extinguisher", [0, 400, 100, 800])], 1)
    crop = view.crop(0.5)
    assert crop.shape == (800, 150, 3)          # 400 + 2*200 high; left side clipped at 0


def test_crop_widens_a_narrow_box_to_the_minimum_aspect():
    buf = TargetViewBuffer("fire extinguisher")
    view = buf.observe(_rgb(), [_det("fire extinguisher", [300, 400, 340, 600])], 1)
    assert view.crop(0.0).shape == (200, 40, 3)
    crop = view.crop(0.0, min_aspect=0.8)       # widened about the box centre, x 240..400
    assert crop.shape == (200, 160, 3)
    assert view.crop(0.0, min_aspect=0.1).shape == (200, 40, 3)   # already wide enough


def test_small_crops_are_enlarged_to_crop_upscale_px():
    buf = TargetViewBuffer("fire extinguisher")
    view = buf.observe(_rgb(), [_det("fire extinguisher", [300, 400, 340, 600])], 1)
    cfg = InspectionConfig(enabled=True, crop_pad=0.0, crop_upscale_px=400)
    crop, whole = ConditionInspector(cfg, "fire extinguisher", client=object()).images([view])
    assert crop.shape == (400, 80, 3)            # 200x40 doubled
    assert whole.shape == view.rgb.shape         # the context frame is left alone
    cfg = InspectionConfig(enabled=True, crop_pad=0.0)
    assert ConditionInspector(cfg, "fire extinguisher", client=object()).images([view])[0].shape == (200, 40, 3)


def test_a_dark_view_is_lifted_before_the_model_sees_it():
    """An unlit corner hid the rust: the corroded extinguisher was judged "good"
    from crops at median lightness 0.08 and 0.14, and "bad" once lifted
    (outputs/20261113_202629, _203830)."""
    import cv2

    from osg.inspection.condition import _brighten

    def median_l(im):
        return float(np.median(cv2.cvtColor(im, cv2.COLOR_RGB2LAB)[..., 0])) / 255.0

    dark = np.zeros((40, 40, 3), np.uint8)
    dark[:] = (40, 14, 10)                                   # dark red
    dark[::2, ::2] = (30, 10, 8)
    lifted = _brighten(dark, 0.35, 0.45)
    assert median_l(dark) < 0.2 and abs(median_l(lifted) - 0.45) < 0.03
    r, g, b = (lifted[..., i].mean() for i in range(3))
    assert r > g and r > b                                  # still red: only lightness moves
    lit = np.full((40, 40, 3), (200, 70, 60), np.uint8)
    assert _brighten(lit, 0.35, 0.45) is lit                # bright enough: as is
    assert _brighten(dark, 0.0, 0.45) is dark               # off

    buf = TargetViewBuffer("fire extinguisher")
    frame = np.zeros((H, W, 3), np.uint8)
    frame[:] = (30, 12, 10)
    view = buf.observe(frame, [_det("fire extinguisher", [300, 400, 340, 600])], 1)
    cfg = InspectionConfig(enabled=True, crop_pad=0.0, brighten_below=0.35)
    crop, whole = ConditionInspector(cfg, "fire extinguisher", client=object()).images([view])
    assert median_l(crop) > 0.4 and median_l(whole) > 0.4
    cfg = InspectionConfig(enabled=True, crop_pad=0.0)
    crop, _ = ConditionInspector(cfg, "fire extinguisher", client=object()).images([view])
    assert median_l(crop) < 0.2


def _camera_facing_ros_x():
    """A camera at the origin whose optical axis is ROS +x (pipeline +x)."""
    T = np.eye(4)
    T[:3, :3] = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]]).T  # columns: x, y, z(optical)
    return T


@pytest.mark.parametrize("ros_xy, turn", [((2, 0), 0), ((0, 2), 90), ((0, -2), -90), ((-2, 0.001), 180)])
def test_bearing_is_the_left_turn_to_face_the_target(ros_xy, turn):
    target = np.array([ros_xy[0], 0.4, -ros_xy[1]])          # ROS (x, y) -> pipeline (x, y-up, -y)
    assert bearing_deg(_camera_facing_ros_x(), target) == pytest.approx(turn, abs=0.1)


# ---------------------------------------------------------------- verdict

def test_parse_verdict_normalises():
    v = parse_verdict('Sure: {"condition": "BAD", "confidence": 1.7, "issues": "rust", "summary": " x "}')
    assert v == {"condition": "bad", "confidence": 1.0, "issues": ["rust"], "summary": "x"}


def test_parse_verdict_refuses_a_non_verdict():
    with pytest.raises(ValueError):
        parse_verdict('{"condition": "fine"}')


def test_prompt_names_the_target_and_the_checklist():
    system, user = build_prompt("fire extinguisher", ["leaks: water pooling"], 2)
    assert "fire extinguisher" in system and "leaks: water pooling" in user and "Images 1-2" in user


class _Client:
    """The slice of an OpenAI client the inspector calls."""

    def __init__(self, content=None, reasoning="", exc=None, finish="stop"):
        self.kwargs = None
        msg = SimpleNamespace(content=content, reasoning_content=reasoning, model_extra={})
        self._resp = SimpleNamespace(choices=[SimpleNamespace(message=msg, finish_reason=finish)])
        self._exc = exc
        self.chat = SimpleNamespace(completions=SimpleNamespace(create=self._create))

    def _create(self, **kwargs):
        self.kwargs = kwargs
        self.calls = getattr(self, "calls", []) + [kwargs]
        if self._exc:
            raise self._exc
        return self._resp


def _views(n=2):
    buf = TargetViewBuffer("fire extinguisher")
    for step in range(n):
        buf.observe(_rgb(), [_det("fire extinguisher", [200, 400, 330, 800 + 10 * step])], step)
    return buf.best(n)


def test_inspect_returns_the_verdict_and_sends_crops_then_the_frame():
    client = _Client('{"condition": "bad", "confidence": 0.9, "issues": ["rust"], "summary": "rusty"}',
                     reasoning="looks corroded")
    cfg = InspectionConfig(enabled=True, thinking=False)
    report = ConditionInspector(cfg, "fire extinguisher", client=client).inspect(_views(2))
    assert (report.condition, report.confidence, report.issues) == ("bad", 0.9, ["rust"])
    assert report.reasoning == "looks corroded" and report.error is None
    content = client.kwargs["messages"][1]["content"]
    assert [c["type"] for c in content] == ["text", "image_url", "image_url", "image_url"]
    assert client.kwargs["extra_body"] == {"chat_template_kwargs": {"enable_thinking": False}}
    assert client.kwargs["max_tokens"] == cfg.max_tokens


@pytest.mark.parametrize("client, says", [
    (_Client(exc=TimeoutError("slow")), "TimeoutError"),
    (_Client(content=None, finish="length"), "max_tokens"),
    (_Client(content="it looks fine to me"), "unparseable"),
])
def test_a_failed_inspection_is_unknown_not_good(client, says):
    report = ConditionInspector(InspectionConfig(enabled=True), "fire extinguisher", client=client).inspect(_views(1))
    assert report.condition == "unknown" and says in report.error


class _OutOfBudgetThenAnswer(_Client):
    """Reasoning runs out of tokens; the no-thinking retry answers."""

    def _create(self, **kwargs):
        self.calls = getattr(self, "calls", []) + [kwargs]
        thinking = kwargs["extra_body"]["chat_template_kwargs"]["enable_thinking"]
        content = None if thinking else '{"condition": "good", "confidence": 0.9}'
        msg = SimpleNamespace(content=content, reasoning_content="wait, look again..." if thinking else "",
                              model_extra={})
        return SimpleNamespace(choices=[SimpleNamespace(message=msg, finish_reason="length" if thinking else "stop")])


def test_reasoning_out_of_budget_falls_back_to_a_direct_answer():
    client = _OutOfBudgetThenAnswer()
    report = ConditionInspector(InspectionConfig(enabled=True), "fire extinguisher", client=client).inspect(_views(1))
    assert report.condition == "good" and report.fallback and report.error is None
    assert report.reasoning == "wait, look again..."
    assert [c["extra_body"]["chat_template_kwargs"]["enable_thinking"] for c in client.calls] == [True, False]


def test_no_fallback_when_it_is_off():
    cfg = InspectionConfig(enabled=True, fallback_without_thinking=False)
    report = ConditionInspector(cfg, "fire extinguisher", client=_OutOfBudgetThenAnswer()).inspect(_views(1))
    assert report.condition == "unknown" and report.fallback is None


def test_no_views_is_reported_not_asked():
    client = _Client('{"condition": "good"}')
    report = ConditionInspector(InspectionConfig(enabled=True), "fire extinguisher", client=client).inspect([])
    assert report.condition == "unknown" and client.kwargs is None


def test_card_is_an_image():
    report = ConditionInspector(InspectionConfig(), "fire extinguisher",
                                client=_Client('{"condition": "good", "confidence": 0.8}')).inspect(_views(1))
    card = render_card(report, _views(1)[0].crop(0.3))
    assert card.dtype == np.uint8 and card.ndim == 3 and card.shape[2] == 3


# ------------------------------------------------------- debug stream wiring

class _Transport:
    def __init__(self, with_text=True):
        self.texts = []
        if with_text:
            self.publish_text = lambda topic, text: self.texts.append((topic, json.loads(text)))

    def publish_image(self, topic, image, encoding):
        pass

    def publish_markers(self, topic, markers, frame_id):
        pass

    def publish_grid(self, topic, grid, origin, resolution, frame_id, z):
        pass


class _Detector:
    def detect(self, rgb):
        return [Detection("fire extinguisher", 0.8, np.array([10.0, 10.0, 30.0, 60.0]), np.zeros(rgb.shape[:2], bool))]


def _write(stream):
    frame = SimpleNamespace(rgb=np.zeros((64, 48, 3), np.uint8), camera_position=np.array([1.0, 1.3, 2.0]),
                            T_wc=np.eye(4))
    room = SimpleNamespace(id=2, label="lobby", centroid_xy=np.array([1.0, 2.0]))
    obj = SimpleNamespace(track_id=5, label="fire_extinguisher", center=np.array([3.0, 0.4, -4.0]),
                          room_id=2, container_id=None, n_obs=3)
    agent = SimpleNamespace(costmap=Costmap2D(resolution=0.1, size_m=4.0),
                            scene_graph=SimpleNamespace(rooms={2: room}, containers={}, objects=[obj]),
                            object_layer=None)
    stream.write(frame, agent, "fire extinguisher", detector=_Detector())


def test_scene_graph_json_reaches_the_display_in_ros_coordinates():
    tr = _Transport()
    seen = []
    _write(DebugStream(SimpleNamespace(transport=tr), on_detections=lambda rgb, dets, step: seen.append(step)))
    topic, graph = tr.texts[-1]
    assert topic == DebugStream.GRAPH_JSON_TOPIC
    assert graph["rooms"] == [{"id": 2, "label": "lobby", "xy": [1.0, -2.0]}]   # ROS y = -z
    (obj,) = graph["objects"]
    assert obj["target"] and obj["xyz"] == [3.0, 4.0, 0.4] and obj["room_id"] == 2
    assert seen == [1]


def test_the_graph_can_be_published_again_between_steps():
    tr = _Transport()
    stream = DebugStream(SimpleNamespace(transport=tr))
    _write(stream)
    agent = SimpleNamespace(scene_graph=SimpleNamespace(rooms={}, containers={}, objects=[]), object_layer=None)
    stream.publish_graph(agent, "fire extinguisher")
    topic, graph = tr.texts[-1]
    assert topic == DebugStream.GRAPH_JSON_TOPIC and graph["step"] == 1 and graph["objects"] == []


def test_a_bridge_without_publish_text_loses_only_the_json():
    tr = _Transport(with_text=False)
    stream = DebugStream(SimpleNamespace(transport=tr))
    _write(stream)
    _write(stream)
    assert stream._text_broken and not stream._dead
