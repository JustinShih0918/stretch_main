"""A small HTTP + WebSocket server on the standard library alone.

The panel's gateway runs inside the Thor's bridge image (`ros:humble-ros-base`
plus a few ROS packages), which has no aiohttp and no pip-installed web
framework, and that is on purpose: nothing to install, nothing to pin. What the
panel needs from a server is small -- static files, a couple of GET endpoints
and one WebSocket per open page -- and RFC 6455 for text frames is a page of
code.

Handlers are plain callables, so the server knows nothing about ROS:

    server = WebServer(static_dir, routes={"/api/map.png": get_map_png},
                       on_ws_message=..., on_ws_open=..., on_ws_close=...)
    await server.serve("0.0.0.0", 8080)
"""
from __future__ import annotations

import asyncio
import base64
import hashlib
import json
import mimetypes
import os
import struct
from pathlib import Path
from typing import Awaitable, Callable, Dict, Optional, Tuple
from urllib.parse import parse_qs, urlsplit

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
MAX_REQUEST_BYTES = 64 * 1024
MAX_FRAME_BYTES = 1 << 20  # a client sends commands, never anything large

# A GET route returns (status, content_type, body) or None for 404.
Route = Callable[[Dict[str, list]], Optional[Tuple[int, str, bytes]]]


def ws_accept_key(key: str) -> str:
    """Sec-WebSocket-Accept for a client's Sec-WebSocket-Key (RFC 6455 4.2.2)."""
    digest = hashlib.sha1((key.strip() + WS_GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def encode_frame(payload: bytes, opcode: int = 0x1) -> bytes:
    """One unmasked, unfragmented server frame."""
    head = bytes([0x80 | opcode])
    n = len(payload)
    if n < 126:
        head += bytes([n])
    elif n < 1 << 16:
        head += bytes([126]) + struct.pack("!H", n)
    else:
        head += bytes([127]) + struct.pack("!Q", n)
    return head + payload


async def read_frame(reader: asyncio.StreamReader) -> Tuple[int, bytes]:
    """(opcode, payload) of the next client frame, unmasked. Fragmented
    messages are joined; control frames in between are returned as they come."""
    message = b""
    first_opcode = None
    while True:
        b0, b1 = await reader.readexactly(2)
        fin, opcode = b0 & 0x80, b0 & 0x0F
        masked, n = b1 & 0x80, b1 & 0x7F
        if n == 126:
            (n,) = struct.unpack("!H", await reader.readexactly(2))
        elif n == 127:
            (n,) = struct.unpack("!Q", await reader.readexactly(8))
        if n > MAX_FRAME_BYTES:
            raise ConnectionError(f"client frame of {n} bytes")
        mask = await reader.readexactly(4) if masked else b"\0\0\0\0"
        data = bytearray(await reader.readexactly(n))
        for i in range(n):
            data[i] ^= mask[i % 4]
        if opcode >= 0x8:  # control frames are never fragmented
            return opcode, bytes(data)
        if first_opcode is None:
            first_opcode = opcode
        message += bytes(data)
        if fin:
            return first_opcode, message


class WsClient:
    """One open page. `send` is safe to call from the event loop only."""

    def __init__(self, writer: asyncio.StreamWriter, peer: str):
        self.writer = writer
        self.peer = peer
        self.closed = False
        self._lock = asyncio.Lock()

    async def send(self, obj) -> None:
        if self.closed:
            return
        data = obj if isinstance(obj, (bytes, bytearray)) else json.dumps(obj, separators=(",", ":")).encode()
        async with self._lock:
            try:
                self.writer.write(encode_frame(bytes(data)))
                await self.writer.drain()
            except (ConnectionError, RuntimeError):
                self.closed = True


class WebServer:
    def __init__(
        self,
        static_dir: Path,
        routes: Optional[Dict[str, Route]] = None,
        on_ws_open: Optional[Callable[[WsClient], Awaitable[None]]] = None,
        on_ws_message: Optional[Callable[[WsClient, dict], Awaitable[None]]] = None,
        on_ws_close: Optional[Callable[[WsClient], Awaitable[None]]] = None,
    ):
        self.static_dir = Path(static_dir).resolve()
        self.routes = routes or {}
        self.on_ws_open = on_ws_open
        self.on_ws_message = on_ws_message
        self.on_ws_close = on_ws_close
        self.clients: set = set()

    async def serve(self, host: str, port: int) -> asyncio.base_events.Server:
        return await asyncio.start_server(self._handle, host, port)

    async def broadcast(self, obj) -> None:
        data = json.dumps(obj, separators=(",", ":")).encode()
        for client in list(self.clients):
            await client.send(data)

    # ------------------------------------------------------------ HTTP
    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = "%s:%s" % (writer.get_extra_info("peername") or ("?", "?"))[:2]
        try:
            head = await reader.readuntil(b"\r\n\r\n")
        except (asyncio.IncompleteReadError, asyncio.LimitOverrunError, ConnectionError):
            writer.close()
            return
        if len(head) > MAX_REQUEST_BYTES:
            writer.close()
            return
        lines = head.decode("latin-1").split("\r\n")
        try:
            method, target, _ = lines[0].split(" ", 2)
        except ValueError:
            writer.close()
            return
        headers = {}
        for line in lines[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()
        url = urlsplit(target)
        if headers.get("upgrade", "").lower() == "websocket" and url.path == "/ws":
            await self._websocket(reader, writer, headers, peer)
            return
        try:
            if method not in ("GET", "HEAD"):
                await self._respond(writer, 405, "text/plain", b"method not allowed")
            else:
                await self._get(writer, url.path, parse_qs(url.query), method == "HEAD")
        except ConnectionError:
            pass
        finally:
            writer.close()

    async def _respond(self, writer, status: int, ctype: str, body: bytes, head_only=False,
                       cache: str = "no-store") -> None:
        reason = {200: "OK", 404: "Not Found", 405: "Method Not Allowed", 503: "Service Unavailable"}.get(status, "")
        hdr = (f"HTTP/1.1 {status} {reason}\r\nContent-Type: {ctype}\r\n"
               f"Content-Length: {len(body)}\r\nCache-Control: {cache}\r\nConnection: close\r\n\r\n")
        writer.write(hdr.encode("latin-1") + (b"" if head_only else body))
        await writer.drain()

    async def _get(self, writer, path: str, query: Dict[str, list], head_only: bool) -> None:
        route = self.routes.get(path)
        if route is not None:
            result = route(query)
            if result is None:
                await self._respond(writer, 404, "text/plain", b"nothing yet", head_only)
            else:
                await self._respond(writer, *result, head_only=head_only)
            return
        rel = "index.html" if path in ("", "/") else path.lstrip("/")
        file = (self.static_dir / rel).resolve()
        if not str(file).startswith(str(self.static_dir) + os.sep) or not file.is_file():
            await self._respond(writer, 404, "text/plain", b"not found", head_only)
            return
        ctype = mimetypes.guess_type(str(file))[0] or "application/octet-stream"
        if ctype.startswith("text/") or ctype in ("application/javascript",):
            ctype += "; charset=utf-8"
        await self._respond(writer, 200, ctype, file.read_bytes(), head_only)

    # ------------------------------------------------------- WebSocket
    async def _websocket(self, reader, writer, headers: dict, peer: str) -> None:
        key = headers.get("sec-websocket-key")
        if not key:
            await self._respond(writer, 400, "text/plain", b"no key")
            writer.close()
            return
        writer.write((
            "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {ws_accept_key(key)}\r\n\r\n").encode("latin-1"))
        await writer.drain()
        client = WsClient(writer, peer)
        self.clients.add(client)
        try:
            if self.on_ws_open:
                await self.on_ws_open(client)
            while not client.closed:
                opcode, payload = await read_frame(reader)
                if opcode == 0x8:  # close
                    break
                if opcode == 0x9:  # ping -> pong
                    async with client._lock:
                        writer.write(encode_frame(payload, 0xA))
                        await writer.drain()
                    continue
                if opcode != 0x1 or not self.on_ws_message:
                    continue
                try:
                    msg = json.loads(payload.decode("utf-8"))
                except (ValueError, UnicodeDecodeError):
                    continue
                if isinstance(msg, dict):
                    await self.on_ws_message(client, msg)
        except (asyncio.IncompleteReadError, ConnectionError):
            pass
        finally:
            client.closed = True
            self.clients.discard(client)
            if self.on_ws_close:
                await self.on_ws_close(client)
            try:
                writer.write(encode_frame(b"", 0x8))
                writer.close()
            except (ConnectionError, RuntimeError):
                pass
