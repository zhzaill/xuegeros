#!/usr/bin/env python3
"""Human-operated motion lease gateway.

This service is intentionally separate from the read-only body adapter.
It runs in shadow mode by default and has no physical command backend.
"""

from __future__ import annotations

import json
import math
import os
import secrets
import threading
import time
import urllib.request
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlparse


LISTEN_HOST = os.environ.get("XUEGEROS_MOTION_LISTEN_HOST", "127.0.0.1")
LISTEN_PORT = int(os.environ.get("XUEGEROS_MOTION_LISTEN_PORT", "18111"))
ENABLE_VALUE = "I_UNDERSTAND_PHYSICAL_RISK"
MOTION_ENABLED = os.environ.get("XUEGEROS_MOTION_ENABLE") == ENABLE_VALUE
SHADOW_MODE = os.environ.get("XUEGEROS_MOTION_SHADOW", "1") != "0"
OPERATOR_TOKEN = os.environ.get("XUEGEROS_OPERATOR_TOKEN", "")
BACKEND_URL = os.environ.get("XUEGEROS_MOTION_BACKEND_URL", "").strip()
BACKEND_TOKEN = os.environ.get("XUEGEROS_BRIDGE_TOKEN", "")
LEASE_MAX_MS = int(os.environ.get("XUEGEROS_MOTION_LEASE_MAX_MS", "1000"))
WATCHDOG_TIMEOUT_MS = int(os.environ.get("XUEGEROS_WATCHDOG_TIMEOUT_MS", "200"))
MAX_LINEAR = float(os.environ.get("XUEGEROS_MAX_LINEAR", "0.1"))
MAX_ANGULAR = float(os.environ.get("XUEGEROS_MAX_ANGULAR", "0.5"))
MAX_BODY_BYTES = 4096
BACKEND_TIMEOUT_SECONDS = float(os.environ.get("XUEGEROS_BACKEND_TIMEOUT_SECONDS", "1.0"))


def _post_backend(path: str, payload: dict[str, Any]) -> dict[str, Any]:
    """Forward an already-validated command to the physical bridge (ROS side)."""
    if not BACKEND_URL:
        return {"ok": False, "error": "physical_backend_not_configured"}
    headers = {"content-type": "application/json"}
    if BACKEND_TOKEN:
        headers["X-Bridge-Token"] = BACKEND_TOKEN
    request = urllib.request.Request(
        BACKEND_URL.rstrip("/") + path,
        data=json.dumps(payload, separators=(",", ":")).encode("utf-8"),
        headers=headers,
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=BACKEND_TIMEOUT_SECONDS) as response:
            body = response.read().decode("utf-8", "replace")
            value = json.loads(body) if body else {}
            return value if isinstance(value, dict) else {"ok": False, "error": "invalid_backend_response"}
    except Exception as exc:  # network, timeout, invalid JSON
        return {"ok": False, "error": "backend_unreachable", "detail": str(exc)[:200]}


@dataclass
class Lease:
    lease_id: str
    expires_at: float
    last_command_at: float
    reason: str


class MotionState:
    def __init__(
        self,
        *,
        enabled: bool,
        shadow_mode: bool,
        operator_token: str,
        lease_max_ms: int,
        watchdog_timeout_ms: int,
        max_linear: float,
        max_angular: float,
        backend_url: str = "",
    ) -> None:
        self.enabled = enabled
        self.shadow_mode = shadow_mode
        self.operator_token = operator_token
        self.lease_max_ms = lease_max_ms
        self.watchdog_timeout_ms = watchdog_timeout_ms
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.backend_url = backend_url
        self._lock = threading.Lock()
        self._lease: Lease | None = None
        self._safe_stop_events = 0

    def authorized(self, token: str | None) -> bool:
        return bool(
            self.enabled
            and self.operator_token
            and token
            and secrets.compare_digest(token, self.operator_token)
        )

    def status(self, now: float | None = None) -> dict[str, Any]:
        now = time.monotonic() if now is None else now
        with self._lock:
            lease = self._lease
            remaining_ms = (
                max(0, int((lease.expires_at - now) * 1000)) if lease else 0
            )
            return {
                "enabled": self.enabled,
                "shadow_mode": self.shadow_mode,
                "physical_backend_available": bool(self.backend_url),
                "operator_token_configured": bool(self.operator_token),
                "watchdog_timeout_ms": self.watchdog_timeout_ms,
                "lease_max_ms": self.lease_max_ms,
                "max_linear": self.max_linear,
                "max_angular": self.max_angular,
                "active_lease": lease.lease_id if lease else None,
                "lease_remaining_ms": remaining_ms,
                "safe_stop_events": self._safe_stop_events,
            }

    def issue_lease(
        self, duration_ms: int, reason: str, now: float | None = None
    ) -> dict[str, Any]:
        if not self.enabled:
            return {"ok": False, "error": "motion_disabled"}
        if duration_ms < 100 or duration_ms > self.lease_max_ms:
            return {
                "ok": False,
                "error": "invalid_duration",
                "allowed_ms": [100, self.lease_max_ms],
            }
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._lease and self._lease.expires_at > now:
                return {"ok": False, "error": "lease_already_active"}
            lease = Lease(
                lease_id=secrets.token_urlsafe(24),
                expires_at=now + duration_ms / 1000.0,
                last_command_at=now,
                reason=reason[:200],
            )
            self._lease = lease
            return {
                "ok": True,
                "lease_id": lease.lease_id,
                "expires_in_ms": duration_ms,
                "shadow_mode": self.shadow_mode,
            }

    def command(
        self,
        lease_id: str,
        linear: float,
        angular: float,
        duration_ms: int = 300,
        now: float | None = None,
    ) -> dict[str, Any]:
        if not self.enabled:
            return {"ok": False, "error": "motion_disabled"}
        if not math.isfinite(linear) or not math.isfinite(angular):
            return {"ok": False, "error": "velocity_not_finite"}
        if abs(linear) > self.max_linear or abs(angular) > self.max_angular:
            return {
                "ok": False,
                "error": "velocity_limit",
                "max_linear": self.max_linear,
                "max_angular": self.max_angular,
            }
        if duration_ms < 100 or duration_ms > self.lease_max_ms:
            return {"ok": False, "error": "invalid_duration", "allowed_ms": [100, self.lease_max_ms]}

        now = time.monotonic() if now is None else now
        with self._lock:
            lease = self._lease
            if not lease or not secrets.compare_digest(lease.lease_id, lease_id):
                return {"ok": False, "error": "lease_not_found"}
            if lease.expires_at <= now:
                self._lease = None
                return {"ok": False, "error": "lease_expired"}
            lease.last_command_at = now
            remaining_ms = max(0, int((lease.expires_at - now) * 1000))

        if self.shadow_mode:
            return {
                "ok": True,
                "accepted": True,
                "published": False,
                "shadow_mode": True,
                "lease_remaining_ms": remaining_ms,
            }

        backend = _post_backend(
            "/command",
            {
                "linear": linear,
                "angular": angular,
                "duration_ms": duration_ms,
                "lease_id": lease_id,
            },
        )
        accepted = bool(backend.get("ok"))
        return {
            "ok": accepted,
            "accepted": accepted,
            "published": bool(backend.get("published")),
            "shadow_mode": False,
            "lease_remaining_ms": remaining_ms,
            **({} if accepted else {"error": backend.get("error", "backend_rejected"), "detail": backend}),
        }

    def stop(self, reason: str) -> dict[str, Any]:
        if not self.enabled:
            return {"ok": False, "error": "motion_disabled"}
        with self._lock:
            had_lease = self._lease is not None
            self._lease = None
        result: dict[str, Any] = {
            "ok": True,
            "published": False,
            "shadow_mode": self.shadow_mode,
            "reason": reason[:200],
            "lease_cleared": had_lease,
        }
        if not self.shadow_mode:
            backend = _post_backend("/stop", {"reason": reason[:200]})
            result["ok"] = bool(backend.get("ok"))
            result["published"] = bool(backend.get("published"))
            result["physical_backend_available"] = bool(self.backend_url)
            if not backend.get("ok"):
                result["error"] = backend.get("error", "backend_rejected")
                result["detail"] = backend
        return result

    def tick(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        with self._lock:
            lease = self._lease
            if not lease:
                return False
            lease_expired = lease.expires_at <= now
            command_stale = (
                now - lease.last_command_at
                >= self.watchdog_timeout_ms / 1000.0
            )
            if lease_expired or command_stale:
                self._lease = None
                self._safe_stop_events += 1
                return True
        return False


STATE = MotionState(
    enabled=MOTION_ENABLED,
    shadow_mode=SHADOW_MODE,
    operator_token=OPERATOR_TOKEN,
    lease_max_ms=LEASE_MAX_MS,
    watchdog_timeout_ms=WATCHDOG_TIMEOUT_MS,
    max_linear=MAX_LINEAR,
    max_angular=MAX_ANGULAR,
    backend_url=BACKEND_URL,
)


def _watchdog_loop() -> None:
    while True:
        if STATE.tick():
            print(
                json.dumps(
                    {
                        "event": "safe_stop_shadow",
                        "at": time.time(),
                        "timeout_ms": WATCHDOG_TIMEOUT_MS,
                    },
                    separators=(",", ":"),
                ),
                flush=True,
            )
        time.sleep(min(0.02, WATCHDOG_TIMEOUT_MS / 4000.0))


class Handler(BaseHTTPRequestHandler):
    server_version = "XuegerosMotionGateway/1.0"

    def _json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > MAX_BODY_BYTES:
            raise ValueError("invalid body size")
        payload = json.loads(self.rfile.read(length))
        if not isinstance(payload, dict):
            raise ValueError("JSON body must be an object")
        return payload

    def _authorized(self) -> bool:
        return STATE.authorized(self.headers.get("X-Operator-Token"))

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode(
            "utf-8"
        )
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        if urlparse(self.path).path == "/v1/status":
            self._send_json(200, STATE.status())
        else:
            self._send_json(404, {"error": "not_found"})

    def do_POST(self) -> None:  # noqa: N802
        if not self._authorized():
            self._send_json(403, {"error": "operator_authorization_required"})
            return
        path = urlparse(self.path).path
        try:
            body = self._json_body()
        except (ValueError, json.JSONDecodeError) as exc:
            self._send_json(400, {"error": "invalid_json", "detail": str(exc)})
            return

        if path == "/v1/leases":
            result = STATE.issue_lease(
                duration_ms=int(body.get("duration_ms", 0)),
                reason=str(body.get("reason", "")),
            )
            self._send_json(200 if result.get("ok") else 409, result)
            return
        if path == "/v1/commands":
            result = STATE.command(
                lease_id=str(body.get("lease_id", "")),
                linear=float(body.get("linear", 0.0)),
                angular=float(body.get("angular", 0.0)),
                duration_ms=int(body.get("duration_ms", 300)),
            )
            self._send_json(200 if result.get("ok") else 409, result)
            return
        if path == "/v1/stop":
            result = STATE.stop(str(body.get("reason", "operator_stop")))
            self._send_json(200, result)
            return
        self._send_json(404, {"error": "not_found"})

    def log_message(self, fmt: str, *args) -> None:
        print(f"{self.address_string()} - {fmt % args}", flush=True)


def main() -> None:
    threading.Thread(target=_watchdog_loop, daemon=True).start()
    server = ThreadingHTTPServer((LISTEN_HOST, LISTEN_PORT), Handler)
    print(
        json.dumps(
            {
                "listening": f"http://{LISTEN_HOST}:{LISTEN_PORT}",
                "enabled": MOTION_ENABLED,
                "shadow_mode": SHADOW_MODE,
                "physical_backend_available": bool(BACKEND_URL),
            },
            separators=(",", ":"),
        ),
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
