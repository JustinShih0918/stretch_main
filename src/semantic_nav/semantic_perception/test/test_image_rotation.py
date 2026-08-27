"""RGB orientation correction + box round-trip tests."""

import numpy as np
import pytest

from semantic_perception.image_rotation import (
    VALID_RGB_ROTATIONS,
    invert_rgb_rotation,
    normalize_rgb_rotation,
    rotate_box,
    rotate_rgb_image,
    rotated_image_size,
    unrotate_box,
)


@pytest.fixture
def labeled_image():
    # Distinct values make rotation direction unambiguous.
    labels = np.array([
        [[1], [2], [3]],
        [[4], [5], [6]],
    ], dtype=np.uint8)
    return np.repeat(labels, 3, axis=2)


def test_clockwise_90(labeled_image):
    rotated = rotate_rgb_image(labeled_image, "clockwise_90")
    assert rotated[:, :, 0].tolist() == [[4, 1], [5, 2], [6, 3]]


def test_counterclockwise_90(labeled_image):
    rotated = rotate_rgb_image(labeled_image, "counterclockwise_90")
    assert rotated[:, :, 0].tolist() == [[3, 6], [2, 5], [1, 4]]


def test_180(labeled_image):
    rotated = rotate_rgb_image(labeled_image, "180")
    assert rotated[:, :, 0].tolist() == [[6, 5, 4], [3, 2, 1]]


def test_none_returns_original_object(labeled_image):
    assert rotate_rgb_image(labeled_image, "none") is labeled_image


def test_invalid_rotation_rejected():
    with pytest.raises(ValueError, match="rgb_rotation"):
        normalize_rgb_rotation("90ish")


@pytest.mark.parametrize("rotation", VALID_RGB_ROTATIONS)
def test_rotated_image_size_matches_cv2(rotation):
    image = np.zeros((7, 11, 3), dtype=np.uint8)  # height 7, width 11
    rotated = rotate_rgb_image(image, rotation)
    assert rotated_image_size(11, 7, rotation) == (
        rotated.shape[1], rotated.shape[0]
    )


@pytest.mark.parametrize("rotation", VALID_RGB_ROTATIONS)
def test_box_tracks_the_pixels_it_encloses(rotation):
    """The rotated box must enclose exactly the rotated pixels."""
    width, height = 11, 7
    image = np.zeros((height, width, 3), dtype=np.uint8)
    box = (2, 1, 4, 3)  # x, y, w, h
    x, y, w, h = box
    image[y:y + h, x:x + w] = 255

    rotated = rotate_rgb_image(image, rotation)
    rx, ry, rw, rh = rotate_box(box, rotation, width, height)

    marked = np.zeros(rotated.shape[:2], dtype=bool)
    marked[ry:ry + rh, rx:rx + rw] = True
    assert np.array_equal(rotated[:, :, 0] == 255, marked)


@pytest.mark.parametrize("rotation", VALID_RGB_ROTATIONS)
def test_unrotate_box_is_the_inverse(rotation):
    """A box detected in the model's upright frame maps back to raw pixels."""
    width, height = 11, 7
    box = (2, 1, 4, 3)
    rotated_width, rotated_height = rotated_image_size(width, height, rotation)

    model_box = rotate_box(box, rotation, width, height)
    assert unrotate_box(
        model_box, rotation, rotated_width, rotated_height
    ) == box


@pytest.mark.parametrize("rotation", VALID_RGB_ROTATIONS)
def test_invert_rgb_rotation_round_trips(rotation):
    assert invert_rgb_rotation(invert_rgb_rotation(rotation)) == rotation
