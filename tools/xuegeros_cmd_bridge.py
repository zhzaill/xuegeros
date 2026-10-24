#!/usr/bin/env python3
"""XuegerOS physical command bridge (guest side).

The host-side motion gateway validates operator tokens, leases, limits and
durations, then forwards already-validated short pulses here. This bridge is the
only process that publishes motion topics, and it refuses anything outside its
own hard limits.

Endpoints (127.0.0.1 by default; the host gateway reaches it through an SSH
tunnel):

    GET  /health
    POST /command  {linear, angular, duration_ms}
    POST /stop     {reason}

Every command publishes a fresh /motion/lease (expires_at in wall-clock time)
before the Twist stream, because xuegeros_cmd_vel_watchdog requires a valid
lease. The stream runs at 20 Hz so the mux's 200 ms input timeout never fires
during a pulse, and zero velocity is published exactly once at the end.
"""

from __future__ import annotations

import json
import math
import os
import secrets
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


LISTEN_HOST = os.environ.get("XUEGEROS_BRIDGE_LISTEN_HOST", "127.0.0.1")
LISTEN_PORT = int(os.environ.get("XUEGEROS_BRIDGE_LISTEN_PORT", "18200"))
TOKEN = os.environ.get("XUEGEROS_BRIDGE_TOKEN", "")
MAX_LINEAR = float(os.environ.get("XUEGEROS_BRIDGE_MAX_LINEAR", "0.1"))
MAX_ANGULAR = float(os.environ.get("XUEGEROS_BRIDGE_MAX_ANGULAR", "0.5"))
MIN_DURATION_MS = 100
MAX_DURATION_MS = int(os.environ.get("XUEGEROS_BRIDGE_MAX_DURATION_MS", "1000"))
RATE_HZ = 20.0
MAX_BODY_BYTES = 4096


class BridgeNode(Node):
    """Owns the publishers; the HTTP thread only calls into this object."""

    def __init__(self) -> None:
        super().__init__("xuegeros_cmd_bridge")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_autonomy", 10)
        self.lease_pub = self.create_publisher(String, "/motion/lease", 10)
        self._lock = threading.Lock()
        self.last_command_at = 0.0
        self.last_error: str | None = None
        self.get_logger().info(
            f"bridge ready: {MIN_DURATION_MS}-{MAX_DURATION_MS} ms, "
            f"|linear|<={MAX_LINEAR}, |angular|<={MAX_ANGULAR}"
        )

    def publish_lease(self, duration_ms: int) -> str:
        lease_id = f"bridge-{int(time.time() * 1000)}-{secrets.token_hex(3)}"
        payload = {
            "lease_id": lease_id,
            "expires_at": time.time() + max(2.0, duration_ms / 1000.0 + 1.0),
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.lease_pub.publish(message)
        return lease_id

    def publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())

    def execute(self, linear: float, angular: float, duration_ms: int) -> dict[str, Any]:
        if not self._lock.acquire(blocking=False):
            return {"ok": False, "error": "bridge_busy", "published": False}
        try:
            self.publish_lease(duration_ms)
            deadline = time.monotonic() + duration_ms / 1000.0
            period = 1.0 / RATE_HZ
            published = 0
            while time.monotonic() < deadline:
                message = Twist()
                message.linear.x = linear
                message.angular.z = angular
                self.cmd_pub.publish(message)
                published += 1
                time.sleep(period)
            self.publish_zero()
            self.last_command_at = time.time()
            self.last_error = None
            return {
                "ok": True,
                "accepted": True,
                "published": True,
                "duration_ms": duration_ms,
                "published_messages": published,
                "zero_published": True,
            }
        except Exception as exc:  # defensive: never leave a nonzero stream behind
            self.last_error = str(exc)[:200]
            try:
                self.publish_zero()
            except Exception:
                pass
            return {"ok": False, "error": "publish_failed", "detail": self.last_error}
        finally:
            self._lock.release()

    def stop(self, reason: str) -> dict[str, Any]:
        self.publish_zero()
        self.last_command_at = time.time()
        return {"ok": True, "published": True, "zero_published": True, "reason": reason[:200]}


NODE: BridgeNode | None = None


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


class Handler(BaseHTTPRequestHandler):
    server_version = "XuegerosCmdBridge/1.0"

    def _authorized(self) -> bool:
        if not TOKEN:
            return False
        provided = self.headers.get("X-Bridge-Token", "")
        return secrets.compare_digest(provided, TOKEN)

    def _send(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("content-type", "application/json; charset=utf-8")
        self.send_header("cache-control", "no-store")
        self.send_header("content-length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > MAX_BODY_BYTES:
            raise ValueError("invalid body size")
        value = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("body must be a JSON object")
        return value

    def do_GET(self) -> None:  # noqa: N802
        if self.path.split("?", 1)[0] != "/health":
            self._send(404, {"ok": False, "error": "not_found"})
            return
        self._send(200, {
            "ok": True,
            "node": "xuegeros_cmd_bridge",
            "token_configured": bool(TOKEN),
            "limits": {
                "max_linear": MAX_LINEAR,
                "max_angular": MAX_ANGULAR,
                "min_duration_ms": MIN_DURATION_MS,
                "max_duration_ms": MAX_DURATION_MS,
            },
            "last_command_at": NODE.last_command_at if NODE else None,
        })

    def do_POST(self) -> None:  # noqa: N802
        if not self._authorized():
            self._send(403, {"ok": False, "error": "bridge_authorization_required"})
            return
        if NODE is None:
            self._send(503, {"ok": False, "error": "bridge_not_ready"})
            return
        path = self.path.split("?", 1)[0]
        try:
            body = self._body()
        except (ValueError, json.JSONDecodeError) as exc:
            self._send(400, {"ok": False, "error": "invalid_json", "detail": str(exc)[:160]})
            return

        if path == "/stop":
            self._send(200, NODE.stop(str(body.get("reason", "bridge_stop"))))
            return
        if path != "/command":
            self._send(404, {"ok": False, "error": "not_found"})
            return

        linear = body.get("linear", 0.0)
        angular = body.get("angular", 0.0)
        duration_ms = body.get("duration_ms", 0)
        if not _finite(linear) or not _finite(angular):
            self._send(400, {"ok": False, "error": "velocity_not_finite"})
            return
        if abs(float(linear)) > MAX_LINEAR or abs(float(angular)) > MAX_ANGULAR:
            self._send(409, {"ok": False, "error": "velocity_limit", "max_linear": MAX_LINEAR, "max_angular": MAX_ANGULAR})
            return
        if not isinstance(duration_ms, (int, float)) or int(duration_ms) != duration_ms:
            self._send(400, {"ok": False, "error": "invalid_duration"})
            return
        duration = int(duration_ms)
        if duration < MIN_DURATION_MS or duration > MAX_DURATION_MS:
            self._send(409, {"ok": False, "error": "duration_out_of_range", "allowed_ms": [MIN_DURATION_MS, MAX_DURATION_MS]})
            return
        self._send(200, NODE.execute(float(linear), float(angular), duration))

    def log_message(self, fmt: str, *args) -> None:
        print(f"{self.address_string()} - {fmt % args}", flush=True)


def main() -> int:
    global NODE
    rclpy.init()
    NODE = BridgeNode()
    server = ThreadingHTTPServer((LISTEN_HOST, LISTEN_PORT), Handler)
    server.daemon_threads = True
    print(f"xuegeros_cmd_bridge listening on {LISTEN_HOST}:{LISTEN_PORT}", flush=True)

    def _shutdown(_signum, _frame):
        try:
            NODE.stop("signal")
        except Exception:
            pass
        server.shutdown()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _shutdown)
        except (ValueError, OSError):
            pass

    thread = threading.Thread(target=server.serve_forever, name="bridge-http", daemon=True)
    thread.start()
    try:
        rclpy.spin(NODE)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            NODE.publish_zero()
        except Exception:
            pass
        server.shutdown()
        NODE.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
