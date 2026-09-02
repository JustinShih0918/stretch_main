import numpy as np
import pytest

from semantic_perception.image_rotation import (
    inverse_rotation,
    normalize_rgb_rotation,
    rotate_box,
    rotate_rgb_image,
    rotate_size,
)

ROTATIONS = ("none", "clockwise_90", "counterclockwise_90", "180")


def labeled_image():
    # 2 rows x 3 cols, each channel carrying the same 1..6 pattern.
    plane = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.uint8)
    return np.dstack([plane, plane, plane])


def test_rotate_counterclockwise_90_image():
    rotated = rotate_rgb_image(labeled_image(), "counterclockwise_90")
    assert rotated[:, :, 0].tolist() == [[3, 6], [2, 5], [1, 4]]


def test_rotate_size_swaps_only_for_right_angles():
    assert rotate_size(640, 480, "counterclockwise_90") == (480, 640)
    assert rotate_size(640, 480, "clockwise_90") == (480, 640)
    assert rotate_size(640, 480, "180") == (640, 480)
    assert rotate_size(640, 480, "none") == (640, 480)


def test_rotate_box_matches_the_pixels_it_moves():
    """A one-pixel box must land on that same pixel after rotation."""
    image = labeled_image()
    height, width = image.shape[:2]
    for rotation in ROTATIONS:
        rotated = rotate_rgb_image(image, rotation)
        for y in range(height):
            for x in range(width):
                box = {"x": x, "y": y, "width": 1, "height": 1}
                moved = rotate_box(box, rotation, width, height)
                assert (
                    rotated[moved["y"], moved["x"], 0] == image[y, x, 0]
                ), f"{rotation} moved pixel ({x}, {y}) onto the wrong value"


def test_inverse_rotation_round_trips_a_box():
    box = {"label": "curtain", "x": 12, "y": 30, "width": 40, "height": 25}
    width, height = 640, 480
    for rotation in ROTATIONS:
        rot_w, rot_h = rotate_size(width, height, rotation)
        forward = rotate_box(box, rotation, width, height)
        back = rotate_box(forward, inverse_rotation(rotation), rot_w, rot_h)
        assert back == box


def test_rotate_box_keeps_other_keys():
    box = {"label": "door", "x": 1, "y": 1, "width": 2, "height": 3}
    moved = rotate_box(box, "counterclockwise_90", 10, 10)
    assert moved["label"] == "door"


def test_none_is_a_passthrough():
    image = labeled_image()
    assert rotate_rgb_image(image, "none") is image
    box = {"x": 1, "y": 2, "width": 3, "height": 4}
    assert rotate_box(box, "none", 10, 10) is box


def test_invalid_rotation_rejected():
    with pytest.raises(ValueError):
        normalize_rgb_rotation("ccw90")
