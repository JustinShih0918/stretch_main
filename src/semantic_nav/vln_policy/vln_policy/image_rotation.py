"""Shared image and calibration orientation correction."""

from .backends.base import CameraIntrinsics

VALID_RGB_ROTATIONS = (
    "none",
    "clockwise_90",
    "counterclockwise_90",
    "180",
)


def normalize_rgb_rotation(value: str) -> str:
    rotation = str(value).strip().lower()
    if rotation not in VALID_RGB_ROTATIONS:
        raise ValueError(
            f"rgb_rotation must be one of {VALID_RGB_ROTATIONS}, "
            f"got '{value}'"
        )
    return rotation


def rotate_rgb_image(image, rotation: str):
    """Return `image` with the configured lossless right-angle rotation."""
    rotation = normalize_rgb_rotation(rotation)
    if rotation == "none":
        return image

    import cv2
    code = {
        "clockwise_90": cv2.ROTATE_90_CLOCKWISE,
        "counterclockwise_90": cv2.ROTATE_90_COUNTERCLOCKWISE,
        "180": cv2.ROTATE_180,
    }[rotation]
    return cv2.rotate(image, code)


def rotate_depth_image(image, rotation: str):
    """Depth uses the identical lossless pixel transform as RGB."""
    return rotate_rgb_image(image, rotation)


def rotate_camera_intrinsics(
    intrinsics: CameraIntrinsics, rotation: str
) -> CameraIntrinsics:
    """Rotate a pinhole calibration with its image.

    Pixel centers use the usual zero-based coordinates, hence the ``- 1``.
    This supports the right-angle rotations allowed by the launch interface.
    """
    rotation = normalize_rgb_rotation(rotation)
    k = intrinsics
    if rotation == "none":
        return CameraIntrinsics(**k.__dict__)
    if rotation == "clockwise_90":
        return CameraIntrinsics(
            fx=k.fy, fy=k.fx,
            cx=k.height - 1.0 - k.cy, cy=k.cx,
            width=k.height, height=k.width,
        )
    if rotation == "counterclockwise_90":
        return CameraIntrinsics(
            fx=k.fy, fy=k.fx,
            cx=k.cy, cy=k.width - 1.0 - k.cx,
            width=k.height, height=k.width,
        )
    return CameraIntrinsics(
        fx=k.fx, fy=k.fy,
        cx=k.width - 1.0 - k.cx,
        cy=k.height - 1.0 - k.cy,
        width=k.width, height=k.height,
    )
