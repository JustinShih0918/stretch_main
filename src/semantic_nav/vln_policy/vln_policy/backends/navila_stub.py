"""NaVILA / NaVid adapter slot.

The swap plan if StreamVLN underperforms: stand up another self-hosted VLN
server implementing the SAME wire contract as docker/vln (normative spec in
vln_policy/DESIGN.md), then run the demo with `backend:=navila
server_url:=http://localhost:18081`. Nothing on the ROS side changes.

What that server must implement:
  * GET  /health  -> {"status": "ok", "backend": "navila", ...}
  * POST /reset   {"instruction"} -> {"session_id"} — start a new episode,
    clear any frame/history state for the model.
  * POST /step    {"session_id", "image_jpeg_b64", "odom"|null}
                  -> {"actions": [...], "done", "latency_ms"}
    `actions` MUST use the discrete vocabulary STOP / FORWARD / TURN_LEFT /
    TURN_RIGHT with the geometry fixed in backends/base.py (0.25 m, 15 deg).
    Models with continuous outputs (NaVILA emits velocity-style commands,
    NaVid variable distances) must quantize server-side — the adapter owns
    that mapping, not the robot client.

There is no NaVILA install script yet; until a server exists this backend
simply fails its health check / reset gracefully (agent goes to ERROR and
stays alive), which is also how the demo shows the swap seam.
"""

from .streamvln_http import StreamVLNHttpBackend


class NaVILAHttpBackend(StreamVLNHttpBackend):
    name = "navila"

    def __init__(self, server_url: str = "http://localhost:18081", **kwargs):
        super().__init__(server_url=server_url, **kwargs)
