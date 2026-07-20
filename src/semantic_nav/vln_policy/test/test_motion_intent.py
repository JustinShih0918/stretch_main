"""Robot-relative natural-language command tests."""

import pytest

from vln_policy.backends.base import BACKWARD
from vln_policy.motion_intent import parse_robot_relative_command


@pytest.mark.parametrize(
    "instruction",
    [
        "backward",
        "Move backward",
        "go backwards please",
        "please back up",
        "reverse!",
        "drive straight back",
    ],
)
def test_reverse_phrases_default_to_one_step(instruction):
    assert parse_robot_relative_command(instruction) == [BACKWARD]


@pytest.mark.parametrize(
    ("instruction", "steps"),
    [
        ("back up 50 cm", 2),
        ("move backward for 1 meter", 4),
        ("reverse by 2 ft", 2),
        ("go back 10 inches", 1),
    ],
)
def test_explicit_distance_is_quantized_to_25_cm(instruction, steps):
    assert parse_robot_relative_command(instruction) == [BACKWARD] * steps


@pytest.mark.parametrize(
    "instruction",
    [
        "go to the back of the room",
        "look behind the robot",
        "move to the rear wall",
        "turn around and move forward",
        "go back to the kitchen",
    ],
)
def test_navigation_language_is_not_misclassified(instruction):
    assert parse_robot_relative_command(instruction) is None
