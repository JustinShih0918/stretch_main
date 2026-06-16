# A semantic region projected onto the ground plane, consumed by the
# SemanticTraversabilityLayer to override the action-aware costmap.
string label                  # source landmark class (e.g. "curtain")
bool traversable              # action-aware attribute: true = may override LETHAL cells
float32 cost                  # cost to write into traversable cells [0..254]
                              # (0 = FREESPACE like the paper, ~100 = risk-aware)
geometry_msgs/PolygonStamped polygon   # ground-plane footprint (XY in polygon.header.frame_id)
