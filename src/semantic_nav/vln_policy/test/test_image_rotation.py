"""RGB orientation correction tests."""

import numpy as np
import pytest

from vln_policy.image_rotation import (
    normalize_rgb_rotation,
    rotate_camera_intrinsics,
    rotate_depth_image,
    rotate_rgb_image,
)
from vln_policy.backends.base import CameraIntrinsics


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


def test_depth_uses_same_pixel_rotation(labeled_image):
    depth = labeled_image[:, :, 0].astype(np.uint16)
    assert rotate_depth_image(depth, "clockwise_90").tolist() == [
        [4, 1], [5, 2], [6, 3]
    ]


def test_clockwise_rotation_updates_calibration_and_dimensions():
    original = CameraIntrinsics(
        fx=100, fy=110, cx=20, cy=10, width=64, height=48
    )
    rotated = rotate_camera_intrinsics(original, "clockwise_90")
    assert rotated == CameraIntrinsics(
        fx=110, fy=100, cx=37, cy=20, width=48, height=64
    )


def test_counterclockwise_and_180_intrinsics():
    original = CameraIntrinsics(100, 110, 20, 10, 64, 48)
    ccw = rotate_camera_intrinsics(original, "counterclockwise_90")
    assert (ccw.fx, ccw.fy, ccw.cx, ccw.cy, ccw.width, ccw.height) == (
        110, 100, 10, 43, 48, 64
    )
    half = rotate_camera_intrinsics(original, "180")
    assert (half.cx, half.cy, half.width, half.height) == (43, 37, 64, 48)
