"""RGB orientation correction for VLM input, with box round-tripping.

The Stretch head camera publishes its RGB frame rolled sideways, so an open-set
VLM sees a 90-degree-rotated scene and grounds poorly. The perception nodes
rotate the frame upright before inference (same correction as
`vln_policy/image_rotation.py`) and then map the detected boxes **back** to the
original, unrotated pixel frame before publishing `SemanticDetection2D`:
`/depth` and `/camera_info`, which `projection_node` pairs the boxes with, are
never rotated, so the published contract must stay in camera pixel coordinates.
"""

VALID_RGB_ROTATIONS = (
    "none",
    "clockwise_90",
    "counterclockwise_90",
    "180",
)

_INVERSE_RGB_ROTATION = {
    "none": "none",
    "clockwise_90": "counterclockwise_90",
    "counterclockwise_90": "clockwise_90",
    "180": "180",
}


def normalize_rgb_rotation(value: str) -> str:
    rotation = str(value).strip().lower()
    if rotation not in VALID_RGB_ROTATIONS:
        raise ValueError(
            f"rgb_rotation must be one of {VALID_RGB_ROTATIONS}, "
            f"got '{value}'"
        )
    return rotation


def invert_rgb_rotation(rotation: str) -> str:
    """Return the rotation that undoes `rotation`."""
    return _INVERSE_RGB_ROTATION[normalize_rgb_rotation(rotation)]


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


def rotated_image_size(width: int, height: int, rotation: str):
    """Size of a `width` x `height` image after `rotation`."""
    rotation = normalize_rgb_rotation(rotation)
    if rotation in ("clockwise_90", "counterclockwise_90"):
        return height, width
    return width, height


def rotate_box(box, rotation: str, width: int, height: int):
    """Map `box` into the same image rotated by `rotation`.

    `box` is `(x, y, w, h)` in pixels of an image sized `width` x `height`;
    the result is `(x, y, w, h)` in the rotated image.
    """
    rotation = normalize_rgb_rotation(rotation)
    x, y, w, h = (int(v) for v in box)
    if rotation == "clockwise_90":
        return (height - y - h, x, h, w)
    if rotation == "counterclockwise_90":
        return (y, width - x - w, h, w)
    if rotation == "180":
        return (width - x - w, height - y - h, w, h)
    return (x, y, w, h)


def unrotate_box(box, rotation: str, rotated_width: int, rotated_height: int):
    """Map a box detected in the rotated image back to the original frame.

    `rotated_width` / `rotated_height` are the dimensions of the **rotated**
    image the box was detected in (i.e. what the model saw).
    """
    return rotate_box(
        box, invert_rgb_rotation(rotation), rotated_width, rotated_height
    )
