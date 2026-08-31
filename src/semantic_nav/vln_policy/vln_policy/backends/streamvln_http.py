"""Backward-compatible StreamVLN name over the shared HTTP client."""

from .http import HttpVLNBackend


class StreamVLNHttpBackend(HttpVLNBackend):
    name = "streamvln"
    requires_rgb = True
    requires_depth = False

    def __init__(self, server_url="http://localhost:18080", **kwargs):
        super().__init__(server_url=server_url, **kwargs)
