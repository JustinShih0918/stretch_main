"""`inspection` group: what the robot does AFTER it has found the target.

The search ends with a STOP beside the target. With this group on, the run then
asks a vision-language model what CONDITION that object is in -- stretch_main's
fire-extinguisher inspection demo (scripts/extinguisher_demo/) -- from the best views of
it the search collected on the way in (`osg/inspection/views.py`) plus a last
look from where it stopped.

It is a different model from the verifier's on purpose: the verifier answers
"is this the target", which a small fast VLM does well; this answers "is it
damaged", which needs a model that looks harder.

Off by default. Nothing in the search reads it, so no benchmark arm changes.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class InspectionConfig:
    enabled: bool = False

    # ------------------------------------------------------------- the model
    # Any OpenAI-compatible endpoint. The key is the yaml's business
    # (configs/inspection/nim.yaml reads NVIDIA_API_KEY), so it never lands in
    # a run's `.hydra/overrides.yaml`.
    base_url: str = "https://integrate.api.nvidia.com/v1"
    model: str = "nvidia/nemotron-3.5-super-vl-120b-a12b"
    api_key: str = ""
    timeout_s: float = 120.0
    # Nemotron 3.5 Super VL is a reasoning model. With thinking on, its
    # reasoning comes out of this budget BEFORE the answer: at 1024 tokens a
    # probe on the demo's first target, a fire hydrant, spent all of it
    # thinking and returned no answer at all (2026-11-09); answered calls used
    # ~650 tokens in all. A clean hydrant can
    # set it second-guessing indefinitely -- one took the whole of 4096 in 79 s
    # and never answered (2026-11-10) -- so the budget is a time limit as much
    # as a size, and `fallback_without_thinking` covers running out of it.
    max_tokens: int = 2048
    # Reasoning on: ~15 s per inspection against ~2 s off (2026-11-09, one
    # hydrant image, both verdicts right). The robot is parked by then, so the
    # slower, more careful answer is the right trade.
    thinking: bool = True
    # When the reasoning uses the whole budget and gives no answer, ask once
    # more with thinking off (~2 s; right on both hydrants in the 2026-11-09
    # probe). The report says so (`fallback`), and keeps the partial reasoning.
    fallback_without_thinking: bool = True
    temperature: float = 0.2
    # Longest image side sent; the crop is what the model judges, so it must
    # stay large enough to show rust and a missing part.
    max_image_px: int = 1024

    # --------------------------------------------------------- the evidence
    # Views of the target the search saw, ranked by `views.TargetViewBuffer`:
    # a box that touches the image edge is a partial view, and one smaller
    # than `min_bbox_h_frac` of the image height shows too little to judge.
    keep_views: int = 6
    n_views: int = 2
    border_px: int = 6
    min_bbox_h_frac: float = 0.12
    # Context added around the box on each side, as a fraction of its size:
    # the floor around an object (a leak, spilled powder) is part of its
    # condition.
    crop_pad: float = 0.35
    # Widen a crop to at least this width/height ratio (0 = as padded). A
    # 0.66 m extinguisher seen from 1.9 m padded to a 146x457 sliver that cut
    # off the powder spilled beside it (2026-11-10).
    crop_min_aspect: float = 0.0
    # Enlarge a crop whose long side is under this many pixels (0 = never).
    # A model looks at a small crop through few patches: from 1.9 m an
    # extinguisher crop was 179x297 px, and the VLM missed rust it named once
    # the same crop was enlarged (2026-11-10).
    crop_upscale_px: int = 0
    # Lift an image whose median lightness (CIELAB L, 0-1) is below this to
    # `brighten_to` with a gamma on L, before the model sees it (0 = off). An
    # unlit corner hides rust: the corroded extinguisher, found 16 m and 14 m
    # from its place, was judged "good" twice from crops at median L 0.08 and
    # 0.14 ("the lighting is dim, making it hard to see ... rust"); the same
    # images lifted were judged "bad" (rust, peeling paint). The same lift kept
    # every verdict it was tried on that was already right (3) and found no
    # defect on the intact one darkened to L 0.08 and 0.14 (2026-11-13).
    brighten_below: float = 0.0
    brighten_to: float = 0.45
    # Fresh frames taken from the stop pose before asking, on top of the
    # search's own.
    final_frames: int = 3
    # Turn in place to face the target before those frames. The approach can
    # stop with the target off to the side -- the 2026-11-10 search take
    # stopped 1.5 m from the target (then a fire hydrant) looking at the wall
    # beside it, so its last look saw no target at all.
    face_target: bool = True
    face_min_deg: float = 8.0

    # What an inspector checks for, as short phrases appended to the prompt.
    # Empty = a generic visual condition check of the target.
    checklist: List[str] = field(default_factory=list)
