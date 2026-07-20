"""Shared RGB orientation correction for model input and visualization."""

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
