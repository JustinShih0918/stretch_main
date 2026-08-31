"""DualVLN protocol-v2 client (RGB + synchronized metric depth)."""

from .http import HttpVLNBackend


class DualVLNHttpBackend(HttpVLNBackend):
    name = "dualvln"
    requires_rgb = True
    requires_depth = True

    def __init__(self, server_url="http://localhost:18082", **kwargs):
        super().__init__(server_url=server_url, **kwargs)
