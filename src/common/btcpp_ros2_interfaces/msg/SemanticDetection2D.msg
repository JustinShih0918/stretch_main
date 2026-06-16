# A single landmark detection in the camera image plane, with an
# action-aware attribute (paper 2310.08873: traversable / untraversable).
std_msgs/Header header        # stamp + camera optical frame the bbox refers to
string label                  # detected landmark class (e.g. "curtain")
bool traversable              # action-aware attribute: true = robot may pass through
float32 confidence            # detector confidence [0, 1]

# Axis-aligned bounding box in pixels (top-left origin)
int32 x                       # top-left corner x
int32 y                       # top-left corner y
int32 width                   # box width  in pixels
int32 height                  # box height in pixels

# Optional binary/instance mask (same size as the source image, mono8).
# Leave empty (height == 0) to fall back to the bounding box.
sensor_msgs/Image mask
