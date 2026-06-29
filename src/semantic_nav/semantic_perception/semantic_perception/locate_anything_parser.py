"""Parsing helpers for LocateAnything's generated grounding output."""

import re
from typing import Optional


_REF_BLOCK_RE = re.compile(
    r"<ref>\s*([^<]+?)\s*</ref>\s*((?:<box>.*?</box>\s*)+)",
    re.IGNORECASE | re.DOTALL,
)
_BOX_RE = re.compile(
    r"<box>\s*"
    r"<\s*(-?\d+(?:\.\d+)?)\s*>\s*"
    r"<\s*(-?\d+(?:\.\d+)?)\s*>\s*"
    r"<\s*(-?\d+(?:\.\d+)?)\s*>\s*"
    r"<\s*(-?\d+(?:\.\d+)?)\s*>\s*"
    r"</box>",
    re.IGNORECASE,
)


def parse_labeled_boxes(
    answer: str,
    image_width: int,
    image_height: int,
    fallback_label: Optional[str] = None,
) -> list[dict]:
    """Convert LocateAnything output into labeled pixel-coordinate boxes.

    The model emits coordinates normalized to [0, 1000]. Invalid image sizes,
    non-finite coordinates, zero-area boxes, and malformed output are ignored.
    Coordinates outside the nominal range are clamped and reversed corners are
    normalized.
    """
    if image_width <= 0 or image_height <= 0 or not answer:
        return []

    parsed = []
    for match in _REF_BLOCK_RE.finditer(answer):
        label = match.group(1).strip()
        parsed.extend(
            _parse_box_block(
                match.group(2), label, image_width, image_height
            )
        )

    # Phrase-grounding responses normally include <ref>, but accepting an
    # unlabeled box is useful when a single runtime instruction is active.
    if not parsed and fallback_label:
        parsed.extend(
            _parse_box_block(
                answer, fallback_label.strip(), image_width, image_height
            )
        )

    return parsed


def _parse_box_block(
    block: str, label: str, image_width: int, image_height: int
) -> list[dict]:
    boxes = []
    if not label:
        return boxes

    for match in _BOX_RE.finditer(block):
        values = [float(value) for value in match.groups()]
        x1, y1, x2, y2 = [max(0.0, min(1000.0, v)) for v in values]
        x_min, x_max = sorted((x1, x2))
        y_min, y_max = sorted((y1, y2))

        px1 = int(round(x_min * image_width / 1000.0))
        py1 = int(round(y_min * image_height / 1000.0))
        px2 = int(round(x_max * image_width / 1000.0))
        py2 = int(round(y_max * image_height / 1000.0))

        px1 = max(0, min(image_width - 1, px1))
        py1 = max(0, min(image_height - 1, py1))
        px2 = max(0, min(image_width, px2))
        py2 = max(0, min(image_height, py2))
        if px2 <= px1 or py2 <= py1:
            continue

        boxes.append(
            {
                "label": label,
                "x": px1,
                "y": py1,
                "width": px2 - px1,
                "height": py2 - py1,
            }
        )
    return boxes
