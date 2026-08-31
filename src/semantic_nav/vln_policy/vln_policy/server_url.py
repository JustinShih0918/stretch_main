"""Backend-specific server URL selection shared by launch/tests."""


def resolve_server_url(
    backend: str,
    explicit: str,
    streamvln_url: str,
    dualvln_url: str,
) -> str:
    if str(explicit).strip():
        return str(explicit).strip()
    if backend == "dualvln":
        return str(dualvln_url).strip()
    return str(streamvln_url).strip()
