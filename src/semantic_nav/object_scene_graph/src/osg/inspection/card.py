"""The inspection result as one picture: the close-up the model judged, and its verdict.

Saved beside `inspection.json` and published on /osg/inspection, so the
operator watching RViz sees what was decided and from what.
"""
from __future__ import annotations

import textwrap

import numpy as np

from .condition import InspectionReport

COLOURS = {"good": (60, 180, 75), "bad": (40, 40, 220), "unknown": (40, 160, 230)}  # BGR


def render_card(report: InspectionReport, crop_rgb, height: int = 480, width: int = 1000) -> np.ndarray:
    """BGR uint8 (height, width, 3)."""
    import cv2

    card = np.full((height, width, 3), 28, np.uint8)
    x = 16
    if crop_rgb is not None and crop_rgb.size:
        h, w = crop_rgb.shape[:2]
        s = (height - 32) / float(h)
        thumb = cv2.resize(np.ascontiguousarray(crop_rgb[..., ::-1]),
                           (max(1, int(w * s)), height - 32), interpolation=cv2.INTER_AREA)
        thumb = thumb[:, : width // 2]
        card[16:16 + thumb.shape[0], 16:16 + thumb.shape[1]] = thumb
        x = 32 + thumb.shape[1]
    colour = COLOURS.get(report.condition, COLOURS["unknown"])
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(card, f"{report.target.upper()} CONDITION", (x, 44), font, 0.7, (200, 200, 200), 1, cv2.LINE_AA)
    verdict = report.condition.upper()
    if report.condition in ("good", "bad"):
        verdict += f"  {report.confidence:.0%}"
    cv2.putText(card, verdict, (x, 104), font, 1.8, colour, 4, cv2.LINE_AA)
    y = 150
    lines = [f"- {i}" for i in report.issues] or (["- no defects visible"] if report.condition == "good" else [])
    if report.error:
        lines = [f"! {report.error}"]
    for line in lines[:6]:
        for part in textwrap.wrap(line, 44)[:2]:
            cv2.putText(card, part, (x, y), font, 0.6, (235, 235, 235), 1, cv2.LINE_AA)
            y += 28
    for part in textwrap.wrap(report.summary, 52)[:3]:
        y += 6
        cv2.putText(card, part, (x, y), font, 0.5, (170, 170, 170), 1, cv2.LINE_AA)
        y += 20
    cv2.putText(card, f"{report.model}  {report.latency_s:.1f}s", (x, height - 20), font, 0.45,
                (140, 140, 140), 1, cv2.LINE_AA)
    return card
