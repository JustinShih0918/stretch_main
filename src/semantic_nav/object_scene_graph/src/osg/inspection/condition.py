"""Ask a vision-language model what condition the found object is in.

The search answers WHERE the target is; this answers what state it is in --
for the extinguisher demo, whether a fire extinguisher is still fit for use. The
model sees close-up crops of the best views the search collected plus one
whole frame for context, and must return a fixed JSON verdict.

A failure is reported as a failure. A model that timed out, ran out of tokens
while still reasoning, or returned something that is not the verdict gives
condition "unknown" with the reason in `error` -- never a default "good",
which on an inspection robot is the one answer that sends nobody out.
"""
from __future__ import annotations

import base64
import time
from dataclasses import asdict, dataclass, field
from typing import List, Optional

import numpy as np

from ..llm.client import extract_json
from .views import TargetView

CONDITIONS = ("good", "bad")


@dataclass
class InspectionReport:
    target: str
    condition: str                     # good | bad | unknown
    confidence: float = 0.0
    issues: List[str] = field(default_factory=list)
    summary: str = ""
    model: str = ""
    latency_s: float = 0.0
    reasoning: str = ""                # the model's own reasoning, when it shows it
    n_images: int = 0
    view_steps: List[int] = field(default_factory=list)
    # Set when the verdict came from the no-thinking retry, saying why.
    fallback: Optional[str] = None
    error: Optional[str] = None

    def to_dict(self) -> dict:
        return asdict(self)


def _enlarge(rgb: np.ndarray, min_px: int) -> np.ndarray:
    """`rgb` scaled up so its long side is `min_px`, if it is shorter (0 = as is)."""
    h, w = rgb.shape[:2]
    if min_px <= 0 or max(h, w) >= min_px or min(h, w) == 0:
        return rgb
    import cv2

    scale = min_px / max(h, w)
    return cv2.resize(rgb, (int(round(w * scale)), int(round(h * scale))), interpolation=cv2.INTER_CUBIC)


def _brighten(rgb: np.ndarray, below: float, to: float) -> np.ndarray:
    """`rgb` with its lightness lifted so the median reaches `to`, if the median
    (CIELAB L, 0-1) is under `below` (0 = as is). A gamma on L alone: hue and
    saturation stay, so rust stays the colour it was."""
    if below <= 0.0 or rgb.size == 0:
        return rgb
    import cv2

    lab = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2LAB)
    light = lab[..., 0].astype(np.float32) / 255.0
    median = float(np.median(light))
    if median >= below or median <= 1e-3 or not 0.0 < to < 1.0:
        return rgb
    gamma = float(np.log(to) / np.log(median))
    lab[..., 0] = np.clip(np.power(light, gamma) * 255.0, 0, 255).astype(np.uint8)
    return cv2.cvtColor(lab, cv2.COLOR_LAB2RGB)


def _jpeg_b64(rgb: np.ndarray, max_px: int, quality: int = 90) -> str:
    import cv2

    h, w = rgb.shape[:2]
    scale = max_px / max(h, w)
    if scale < 1.0:
        rgb = cv2.resize(rgb, (int(round(w * scale)), int(round(h * scale))),
                         interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", np.ascontiguousarray(rgb[..., ::-1]),
                           [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError("jpeg encode failed")
    return base64.b64encode(buf.tobytes()).decode()


def _outline(rgb: np.ndarray, box, color=(0, 220, 0)) -> np.ndarray:
    """A copy of `rgb` with `box` (x1, y1, x2, y2) drawn round. With a second
    extinguisher in the whole view, the model judged that one: shown both, it
    reported "a second extinguisher ... rusty and lying on the floor" as the
    inspected one's defects (outputs/20261113_013326)."""
    out = np.array(rgb, dtype=np.uint8, copy=True)
    h, w = out.shape[:2]
    t = max(3, int(round(min(h, w) / 160)))
    x1, y1, x2, y2 = (int(round(float(v))) for v in box)
    x1, x2 = max(0, min(w - 1, x1)), max(0, min(w - 1, x2))
    y1, y2 = max(0, min(h - 1, y1)), max(0, min(h - 1, y2))
    out[y1:y1 + t, x1:x2 + 1] = color
    out[max(y1, y2 - t + 1):y2 + 1, x1:x2 + 1] = color
    out[y1:y2 + 1, x1:x1 + t] = color
    out[y1:y2 + 1, max(x1, x2 - t + 1):x2 + 1] = color
    return out


def build_prompt(target: str, checklist, n_crops: int) -> tuple:
    """(system, user) text. The images follow the user text in this order:
    `n_crops` close-ups, best first, then the whole frame of the best view."""
    system = (
        f"You are a fire-safety inspector reviewing a {target} through a mobile "
        "robot's camera. Judge only what is visible in the images; do not assume "
        "defects you cannot see, and do not excuse ones you can.")
    checks = "\n".join(f"- {c}" for c in checklist) if checklist else (
        f"- any visible damage, wear, corrosion, missing parts, misalignment or leaks on the {target}")
    crops = ("Image 1 is a close-up" if n_crops == 1
             else f"Images 1-{n_crops} are close-ups from different moments of the approach")
    user = (
        f"{crops} of the {target}; the last image is the robot's whole view, for context, "
        f"with the {target} being inspected outlined in green. Other objects like it may be "
        "in view: judge only the outlined one.\n\n"
        f"Inspect the {target}'s physical condition. Check:\n{checks}\n\n"
        "Answer \"bad\" if anything you see would justify a maintenance work order, "
        "\"good\" if nothing does. Reply with ONLY this JSON object:\n"
        '{"condition": "good" or "bad", "confidence": a number from 0 to 1, '
        '"issues": ["each defect you can see, in a few words"], '
        '"summary": "one sentence for the maintenance log"}')
    return system, user


def parse_verdict(text: str) -> dict:
    """The model's JSON, normalised; raises ValueError if it is not a verdict."""
    raw = extract_json(text or "")
    condition = str(raw.get("condition", "")).strip().lower()
    if condition not in CONDITIONS:
        raise ValueError(f"condition {raw.get('condition')!r} is not one of {CONDITIONS}")
    try:
        confidence = float(raw.get("confidence", 0.0))
    except (TypeError, ValueError):
        confidence = 0.0
    issues = raw.get("issues") or []
    if isinstance(issues, str):
        issues = [issues]
    return {
        "condition": condition,
        "confidence": float(min(1.0, max(0.0, confidence))),
        "issues": [str(i).strip() for i in issues if str(i).strip()],
        "summary": str(raw.get("summary", "")).strip(),
    }


class ConditionInspector:
    """One VLM call per inspection, over an OpenAI-compatible endpoint."""

    def __init__(self, cfg, target: str, client=None) -> None:
        self.cfg = cfg
        self.target = str(target)
        if client is None:
            from openai import OpenAI

            client = OpenAI(base_url=str(cfg.base_url), api_key=str(cfg.api_key),
                            timeout=float(cfg.timeout_s))
        self._client = client

    def images(self, views: List[TargetView]) -> List[np.ndarray]:
        """What the model is shown: the crops, best first, then one whole frame."""
        if not views:
            return []
        crops = [_enlarge(v.crop(float(self.cfg.crop_pad), float(self.cfg.crop_min_aspect)),
                          int(self.cfg.crop_upscale_px)) for v in views]
        images = crops + [_outline(views[0].rgb, views[0].bbox_xyxy)]
        below = float(getattr(self.cfg, "brighten_below", 0.0) or 0.0)
        to = float(getattr(self.cfg, "brighten_to", 0.45) or 0.45)
        return [_brighten(im, below, to) for im in images]

    def inspect(self, views: List[TargetView]) -> InspectionReport:
        report = InspectionReport(target=self.target, condition="unknown",
                                  model=str(self.cfg.model),
                                  view_steps=[v.step for v in views])
        if not views:
            report.error = f"no usable view of the {self.target} to inspect"
            return report
        images = self.images(views)
        report.n_images = len(images)
        system, user = build_prompt(self.target, list(self.cfg.checklist or []), len(views))
        content = [{"type": "text", "text": user}] + [
            {"type": "image_url",
             "image_url": {"url": f"data:image/jpeg;base64,{_jpeg_b64(im, int(self.cfg.max_image_px))}"}}
            for im in images]
        messages = [{"role": "system", "content": system}, {"role": "user", "content": content}]
        t0 = time.time()
        try:
            text, finish, report.reasoning = self._ask(messages, bool(self.cfg.thinking),
                                                       int(self.cfg.max_tokens))
            if (not text.strip() and finish == "length" and self.cfg.thinking
                    and self.cfg.fallback_without_thinking):
                report.fallback = (f"reasoning used all {int(self.cfg.max_tokens)} tokens without "
                                   "answering; asked again with thinking off")
                text, finish, _ = self._ask(messages, False, 1024)
        except Exception as exc:  # noqa: BLE001 -- reported, never swallowed into a verdict
            report.latency_s = round(time.time() - t0, 2)
            report.error = f"{type(exc).__name__}: {exc}"
            return report
        report.latency_s = round(time.time() - t0, 2)
        if not text.strip():
            report.error = (f"no answer: the model stopped ({finish}) "
                            f"before giving one; raise inspection.max_tokens")
            return report
        try:
            verdict = parse_verdict(text)
        except (ValueError, TypeError) as exc:
            report.error = f"unparseable verdict: {exc}; reply was {text[:200]!r}"
            return report
        report.condition = verdict["condition"]
        report.confidence = verdict["confidence"]
        report.issues = verdict["issues"]
        report.summary = verdict["summary"]
        return report

    def _ask(self, messages, thinking: bool, max_tokens: int):
        """One completion: (answer text, finish reason, reasoning text)."""
        resp = self._client.chat.completions.create(
            model=str(self.cfg.model), messages=messages,
            temperature=float(self.cfg.temperature), max_tokens=int(max_tokens),
            extra_body={"chat_template_kwargs": {"enable_thinking": bool(thinking)}},
        )
        choice = resp.choices[0]
        msg = choice.message
        extra = getattr(msg, "model_extra", None) or {}
        reasoning = str(getattr(msg, "reasoning_content", None) or extra.get("reasoning_content") or "")
        return msg.content or "", choice.finish_reason, reasoning.strip()
