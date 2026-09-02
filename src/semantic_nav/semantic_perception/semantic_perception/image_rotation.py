"""Right-angle RGB orientation correction for VLM input and visualization.

The Stretch's head camera is mounted portrait, so the RGB stream reaches the
VLM lying on its side and detection quality suffers. Rotating the *model input*
fixes that, but only if the boxes the model returns are mapped back into the
original image frame: the projection node pairs each detection with the
*unrotated* depth image and its camera_info, so a box in rotated pixel
coordinates would deproject to the wrong place in the world.

Hence the two halves here: `rotate_rgb_image` for the pixels, and `rotate_box`
for the coordinates — call it with `inverse_rotation(...)` to come back.

(`vln_policy/image_rotation.py` is the sibling of this module for the VLN
stack. The two are deliberately separate: ROS 2 packages are independent
units, and semantic_perception must not depend on vln_policy.)
"""

VALID_RGB_ROTATIONS = (
    "none",
    "clockwise_90",
    "counterclockwise_90",
    "180",
)

_INVERSE = {
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


def inverse_rotation(rotation: str) -> str:
    """Return the rotation that undoes `rotation`."""
    return _INVERSE[normalize_rgb_rotation(rotation)]


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


def rotate_size(width: int, height: int, rotation: str) -> tuple[int, int]:
    """Return the (width, height) an image of this size has after rotation."""
    if normalize_rgb_rotation(rotation) in ("clockwise_90",
                                            "counterclockwise_90"):
        return height, width
    return width, height


def rotate_box(box: dict, rotation: str, width: int, height: int) -> dict:
    """Map a box through the same rotation `rotate_rgb_image` applies.

    `box` is the parser/message form — top-left `x`/`y` plus `width`/`height`
    in pixels — and `width`/`height` are the dimensions of the image the box is
    currently expressed in (the *source* of the rotation). Other keys (label,
    score, ...) are carried through untouched.
    """
    rotation = normalize_rgb_rotation(rotation)
    if rotation == "none":
        return box

    x, y = box["x"], box["y"]
    w, h = box["width"], box["height"]
    if rotation == "clockwise_90":
        # (x, y) -> (height - 1 - y, x); the box's top-left corner becomes its
        # bottom-left one, so the new origin comes from y + h.
        moved = {"x": height - (y + h), "y": x, "width": h, "height": w}
    elif rotation == "counterclockwise_90":
        moved = {"x": y, "y": width - (x + w), "width": h, "height": w}
    else:  # 180
        moved = {"x": width - (x + w), "y": height - (y + h),
                 "width": w, "height": h}
    return {**box, **moved}
