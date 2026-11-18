"""Minimal RFC 6455 WebSocket server side, standard library only.

The car's daemon must not depend on a third-party package to talk to the
console, so this file implements exactly what a browser client needs:
handshake, masked client frames, text/binary payloads, ping/pong and close.
It is not a general-purpose WebSocket library: fragmentation, extensions and
subprotocol negotiation are rejected rather than half-supported.
"""

from __future__ import annotations

import base64
import hashlib
import json
import os
import secrets
import socket
import struct
from typing import Any
from urllib.parse import parse_qs, urlparse

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
OP_CONT = 0x0
OP_TEXT = 0x1
OP_BINARY = 0x2
OP_CLOSE = 0x8
OP_PING = 0x9
OP_PONG = 0xA
MAX_PAYLOAD = 1 << 20


class WebSocketError(Exception):
    pass


def accept_key(key: str) -> str:
    digest = hashlib.sha1((key + GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def _read_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise WebSocketError("connection_closed")
        chunks.extend(chunk)
    return bytes(chunks)


def read_request(sock: socket.socket) -> tuple[str, dict[str, str]]:
    raw = bytearray()
    while b"\r\n\r\n" not in raw:
        chunk = sock.recv(4096)
        if not chunk:
            raise WebSocketError("connection_closed")
        raw.extend(chunk)
        if len(raw) > 16 * 1024:
            raise WebSocketError("request_too_large")
    head = bytes(raw).split(b"\r\n\r\n", 1)[0].decode("iso-8859-1")
    lines = head.split("\r\n")
    request_line = lines[0]
    headers: dict[str, str] = {}
    for line in lines[1:]:
        if ":" in line:
            name, value = line.split(":", 1)
            headers[name.strip().lower()] = value.strip()
    return request_line, headers


def handshake(sock: socket.socket, *, expected_token: str | None = None) -> str:
    request_line, headers = read_request(sock)
    parts = request_line.split()
    if len(parts) < 3 or parts[0].upper() != "GET":
        raise WebSocketError("not_a_get")
    path = parts[1] if len(parts) > 1 else "/"
    if expected_token:
        supplied = parse_qs(urlparse(path).query).get("token", [""])[0]
        if not secrets.compare_digest(supplied, expected_token):
            body = b'{"ok":false,"error":"token_required"}'
            sock.sendall(
                b"HTTP/1.1 401 Unauthorized\r\n"
                b"content-type: application/json\r\n"
                + f"content-length: {len(body)}\r\n".encode("ascii")
                + b"connection: close\r\n\r\n"
                + body
            )
            raise WebSocketError("token_required")
    if headers.get("upgrade", "").lower() != "websocket":
        raise WebSocketError("missing_upgrade")
    key = headers.get("sec-websocket-key")
    if not key:
        raise WebSocketError("missing_key")
    response = (
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Accept: {accept_key(key)}\r\n"
        "\r\n"
    )
    sock.sendall(response.encode("ascii"))
    return path


def read_frame(sock: socket.socket) -> tuple[int, bytes]:
    header = _read_exact(sock, 2)
    first, second = header[0], header[1]
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", _read_exact(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", _read_exact(sock, 8))[0]
    if length > MAX_PAYLOAD:
        raise WebSocketError("payload_too_large")
    mask = _read_exact(sock, 4) if masked else b""
    payload = _read_exact(sock, length) if length else b""
    if masked:
        payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
    if not (first & 0x80):
        raise WebSocketError("fragmented_frames_unsupported")
    return opcode, payload


def send_frame(sock: socket.socket, opcode: int, payload: bytes = b"", *, mask: bool = False) -> None:
    length = len(payload)
    header = bytearray([0x80 | opcode])
    flag = 0x80 if mask else 0x00
    if length < 126:
        header.append(flag | length)
    elif length < 65536:
        header.append(flag | 126)
        header.extend(struct.pack("!H", length))
    else:
        header.append(flag | 127)
        header.extend(struct.pack("!Q", length))
    if mask:
        key = os.urandom(4)
        header.extend(key)
        payload = bytes(byte ^ key[index % 4] for index, byte in enumerate(payload))
    sock.sendall(bytes(header) + payload)


def send_json(sock: socket.socket, value: Any) -> None:
    send_frame(sock, OP_TEXT, json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8"))


def recv_json(sock: socket.socket) -> dict[str, Any]:
    while True:
        opcode, payload = read_frame(sock)
        if opcode == OP_TEXT:
            value = json.loads(payload.decode("utf-8"))
            if not isinstance(value, dict):
                raise WebSocketError("json_must_be_object")
            return value
        if opcode == OP_BINARY:
            raise WebSocketError("binary_frames_unsupported")
        if opcode == OP_CLOSE:
            raise WebSocketError("client_closed")
        if opcode == OP_PING:
            send_frame(sock, OP_PONG, payload)
        # OP_PONG: ignore


def client_handshake(sock: socket.socket, *, host: str = "127.0.0.1", path: str = "/ws") -> None:
    """Connect side of the handshake, used by tests and the local console."""

    key = base64.b64encode(os.urandom(16)).decode("ascii")
    request = (
        f"GET {path} HTTP/1.1\r\n"
        f"Host: {host}\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "\r\n"
    )
    sock.sendall(request.encode("ascii"))
    status_line, headers = read_request(sock)
    if "101" not in status_line.split():
        raise WebSocketError(f"handshake_failed: {status_line[:80]}")
    expected = accept_key(key)
    if headers.get("sec-websocket-accept") != expected:
        raise WebSocketError("bad_accept_key")


def send_client_json(sock: socket.socket, value: Any) -> None:
    send_frame(sock, OP_TEXT, json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8"), mask=True)
