"""Interpret simple robot-relative motion instructions.

StreamVLN is a visual navigation policy, not a low-level command parser, and
its learned vocabulary has no BACKWARD token.  This module recognizes only
unambiguous direct reverse commands.  Room-relative language such as "the
back of the room" intentionally falls through to the navigation model.
"""

import math
import re
from typing import List, Optional

from .backends.base import BACKWARD, FORWARD_M


_REVERSE_COMMAND = re.compile(
    r"^\s*(?:please\s+)?(?:"
    r"(?:move|go|drive)\s+(?:straight\s+)?back(?:ward|wards)?"
    r"|back\s+up"
    r"|back(?:ward|wards)"
    r"|reverse"
    r")"
    r"(?:\s+(?:for|by))?"
    r"(?:\s+(?P<distance>\d+(?:\.\d+)?)\s*(?P<unit>"
    r"m|meter|meters|metre|metres|cm|centimeter|centimeters|"
    r"centimetre|centimetres|in|inch|inches|ft|foot|feet"
    r")?)?"
    r"\s*(?:please\s*)?[.!]?\s*$",
    re.IGNORECASE,
)


def _distance_m(value: str, unit: Optional[str]) -> float:
    distance = float(value)
    normalized = (unit or "m").lower()
    if normalized in {"cm", "centimeter", "centimeters",
                      "centimetre", "centimetres"}:
        return distance / 100.0
    if normalized in {"in", "inch", "inches"}:
        return distance * 0.0254
    if normalized in {"ft", "foot", "feet"}:
        return distance * 0.3048
    return distance


def parse_robot_relative_command(instruction: str) -> Optional[List[str]]:
    """Return quantized BACKWARD actions for a direct reverse command.

    A command without a distance means one 0.25 m action.  Explicit distances
    are rounded to the nearest action step, with one step as the minimum.
    Returns None when the instruction is not an unambiguous reverse command.
    """
    match = _REVERSE_COMMAND.fullmatch(instruction)
    if match is None:
        return None

    value = match.group("distance")
    if value is None:
        steps = 1
    else:
        distance_m = _distance_m(value, match.group("unit"))
        if not math.isfinite(distance_m) or distance_m <= 0.0:
            return None
        # Avoid Python's bankers' rounding at exact half steps.
        steps = max(1, int(math.floor(distance_m / FORWARD_M + 0.5)))
    return [BACKWARD] * steps
