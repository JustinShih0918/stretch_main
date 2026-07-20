"""RGB orientation correction tests."""

import numpy as np
import pytest

from vln_policy.image_rotation import (
    normalize_rgb_rotation,
    rotate_rgb_image,
)


@pytest.fixture
def labeled_image():
    # Distinct corner values make rotation direction unambiguous.
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
