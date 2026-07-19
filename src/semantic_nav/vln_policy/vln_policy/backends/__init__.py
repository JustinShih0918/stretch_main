"""Backend registry: name -> lazy factory.

Factories import their module only when selected, mirroring the lazy
model-load idiom of semantic_perception (a missing optional dep must not
break the other backends).
"""


def _streamvln(**kwargs):
    from .streamvln_http import StreamVLNHttpBackend
    return StreamVLNHttpBackend(**kwargs)


def _dummy(**kwargs):
    from .dummy import DummyBackend
    return DummyBackend(**kwargs)


def _navila(**kwargs):
    from .navila_stub import NaVILAHttpBackend
    return NaVILAHttpBackend(**kwargs)


BACKENDS = {
    "streamvln": _streamvln,
    "dummy": _dummy,
    "navila": _navila,
}


def make_backend(name: str, **kwargs):
    if name not in BACKENDS:
        raise ValueError(
            f"unknown VLN backend '{name}' (available: {sorted(BACKENDS)})"
        )
    return BACKENDS[name](**kwargs)
