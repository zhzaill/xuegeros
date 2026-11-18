"""Local AI-session plugin boundary for XuegerOS.

The plugin reads the Body Adapter world and, only after policy validation,
forwards a short pulse to the existing Motion Gateway. It never shells into
the guest and never accepts arbitrary ROS topic names or commands.
"""

from __future__ import annotations

import json
import os
import secrets
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlparse

from .events import EventLogger
from .policy import SafetyPolicy
from .protocol import ActionProposal, WorldSnapshot


LISTEN_HOST = os.environ.get("XUEGEROS_AGENT_PLUGIN_HOST", "127.0.0.1")
LISTEN_PORT = int(os.environ.get("XUEGEROS_AGENT_PLUGIN_PORT", "18112"))
WORLD_URL = os.environ.get("XUEGEROS_AGENT_WORLD_URL", "http://127.0.0.1:18110/v1/world")
GATEWAY_URL = os.environ.get("XUEGEROS_AGENT_GATEWAY_URL", "http://127.0.0.1:18111")
OPERATOR_TOKEN = os.environ.get("XUEGEROS_OPERATOR_TOKEN", "")
PLUGIN_TOKEN = os.environ.get("XUEGEROS_AGENT_PLUGIN_TOKEN", "")
PHYSICAL_ENABLED = (
    os.environ.get("XUEGEROS_AGENT_PHYSICAL_ENABLE") == "1"
    and os.environ.get("XUEGEROS_AGENT_CONFIRM_PHYSICAL") == "I_UNDERSTAND_PHYSICAL_RISK"
    and bool(OPERATOR_TOKEN)
    and bool(PLUGIN_TOKEN)
    and bool(GATEWAY_URL)
)
EVENTS = EventLogger()


def _json_request(url: str, *, method: str = "GET", payload: dict[str, Any] | None = None, headers: dict[str, str] | None = None) -> dict[str, Any]:
    body = None if payload is None else json.dumps(payload, separators=(",", ":")).encode("utf-8")
    request_headers = {"accept": "application/json", **(headers or {})}
    if body is not None:
        request_headers["content-type"] = "application/json"
    request = urllib.request.Request(url, data=body, headers=request_headers, method=method)
    try:
        with urllib.request.urlopen(request, timeout=2.0) as response:
            value = json.loads(response.read().decode("utf-8", "replace"))
            return value if isinstance(value, dict) else {"ok": False, "error": "invalid_json_response"}
    except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
        return {"ok": False, "error": "upstream_unreachable", "detail": str(exc)[:200]}


def get_world() -> dict[str, Any]:
    return _json_request(WORLD_URL)


def gateway_stop(reason: str) -> dict[str, Any]:
    headers = {"X-Operator-Token": OPERATOR_TOKEN} if OPERATOR_TOKEN else {}
    return _json_request(GATEWAY_URL.rstrip("/") + "/v1/stop", method="POST", payload={"reason": reason}, headers=headers)


def gateway_pulse(proposal: ActionProposal) -> dict[str, Any]:
    status = _json_request(GATEWAY_URL.rstrip("/") + "/v1/status")
    if not status.get("enabled"):
        return {"ok": False, "error": "motion_gateway_disabled"}
    if status.get("shadow_mode", True):
        return {"ok": False, "error": "motion_gateway_shadow_mode"}
    if not status.get("operator_token_configured"):
        return {"ok": False, "error": "motion_gateway_operator_token_missing"}
    if not status.get("physical_backend_available"):
        return {"ok": False, "error": "motion_gateway_backend_unavailable"}
    headers = {"X-Operator-Token": OPERATOR_TOKEN} if OPERATOR_TOKEN else {}
    lease = _json_request(
        GATEWAY_URL.rstrip("/") + "/v1/leases",
        method="POST",
        payload={"duration_ms": min(800, proposal.duration_ms), "reason": proposal.rationale or proposal.source},
        headers=headers,
    )
    if not lease.get("ok"):
        return lease
    result = _json_request(
        GATEWAY_URL.rstrip("/") + "/v1/commands",
        method="POST",
        payload={
            "lease_id": lease.get("lease_id"),
            "linear": proposal.linear_mps,
            "angular": proposal.angular_rps,
            "duration_ms": proposal.duration_ms,
        },
        headers=headers,
    )
    if result.get("ok") and (result.get("shadow_mode") or not result.get("published")):
        return {"ok": False, "error": "motion_gateway_did_not_publish", "gateway": result}
    return result


class PluginHandler(BaseHTTPRequestHandler):
    server_version = "XuegerosAgentPlugin/1.0"

    def _authorized(self) -> bool:
        return bool(PLUGIN_TOKEN) and secrets.compare_digest(self.headers.get("X-Agent-Token", ""), PLUGIN_TOKEN)

    def _body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > 8192:
            raise ValueError("invalid body size")
        value = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("body must be an object")
        return value

    def _send(self, status: int, value: dict[str, Any]) -> None:
        body = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("content-type", "application/json; charset=utf-8")
        self.send_header("cache-control", "no-store")
        self.send_header("content-length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        if not self._authorized():
            self._send(403, {"ok": False, "error": "plugin_authorization_required"})
            return
        path = urlparse(self.path).path
        if path == "/health":
            self._send(200, {"ok": True, "physical_enabled": PHYSICAL_ENABLED, "world_url": WORLD_URL})
            return
        if path == "/v1/world":
            world = get_world()
            self._send(200 if world.get("status_available", world.get("ok", True)) else 503, world)
            return
        self._send(404, {"ok": False, "error": "not_found"})

    def do_POST(self) -> None:  # noqa: N802
        if not self._authorized():
            self._send(403, {"ok": False, "error": "plugin_authorization_required"})
            return
        path = urlparse(self.path).path
        try:
            body = self._body()
        except (ValueError, json.JSONDecodeError) as exc:
            self._send(400, {"ok": False, "error": "invalid_json", "detail": str(exc)[:200]})
            return
        if path == "/v1/stop":
            session_id = str(body.get("session_id", ""))
            turn_id = str(body.get("turn_id", ""))
            reason = str(body.get("reason", "agent_stop"))
            result = gateway_stop(reason) if PHYSICAL_ENABLED else {"ok": True, "shadow": True, "published": False}
            event_id = EVENTS.append("stop", {"reason": reason[:200], "result": result, "physical": PHYSICAL_ENABLED}, session_id=session_id, turn_id=turn_id)
            result = {**result, "event_id": event_id}
            self._send(200 if result.get("ok") else 502, result)
            return
        if path != "/v1/propose":
            self._send(404, {"ok": False, "error": "not_found"})
            return
        try:
            proposal = ActionProposal.from_mapping(body.get("proposal", body))
        except (TypeError, ValueError) as exc:
            self._send(400, {"ok": False, "error": "invalid_proposal", "detail": str(exc)[:200]})
            return
        session_id = str(body.get("session_id", ""))
        turn_id = str(body.get("turn_id", ""))
        world_payload = get_world()
        if world_payload.get("error") == "upstream_unreachable":
            self._send(503, world_payload)
            return
        world = WorldSnapshot.from_payload(world_payload)
        policy = SafetyPolicy(
            physical_enabled=True,
            require_battery=PHYSICAL_ENABLED,
            require_map_freshness=True,
            require_mapping_healthy=True,
            max_scan_age_s=2.5,
            max_odom_age_s=1.0,
            max_tf_age_s=1.0,
            max_map_age_s=2.0,
        )
        decision = policy.evaluate(proposal, world)
        if not decision.allowed:
            event_id = EVENTS.append(
                "proposal_blocked",
                {"proposal": proposal.as_dict(), "decision": decision.as_dict()},
                session_id=session_id,
                turn_id=turn_id,
            )
            self._send(409, {"ok": False, "error": "policy_blocked", "decision": decision.as_dict(), "event_id": event_id})
            return
        action = decision.clamped or proposal
        if not PHYSICAL_ENABLED:
            event_id = EVENTS.append(
                "proposal_shadow",
                {"proposal": proposal.as_dict(), "decision": decision.as_dict(), "would_execute": action.as_dict()},
                session_id=session_id,
                turn_id=turn_id,
            )
            self._send(200, {"ok": True, "shadow": True, "would_execute": action.as_dict(), "decision": decision.as_dict(), "event_id": event_id})
            return
        result = gateway_pulse(action)
        event_id = EVENTS.append(
            "proposal_executed" if result.get("ok") else "proposal_failed",
            {"proposal": proposal.as_dict(), "decision": decision.as_dict(), "action": action.as_dict(), "gateway": result},
            session_id=session_id,
            turn_id=turn_id,
        )
        self._send(200 if result.get("ok") else 502, {"ok": bool(result.get("ok")), "shadow": False, "action": action.as_dict(), "gateway": result, "event_id": event_id})

    def log_message(self, fmt: str, *args: object) -> None:
        print(f"{self.address_string()} - {fmt % args}", flush=True)


def main() -> None:
    if not PLUGIN_TOKEN:
        raise SystemExit("XUEGEROS_AGENT_PLUGIN_TOKEN must be set before starting the AI plugin")
    server = ThreadingHTTPServer((LISTEN_HOST, LISTEN_PORT), PluginHandler)
    print(json.dumps({"listening": f"http://{LISTEN_HOST}:{LISTEN_PORT}", "physical_enabled": PHYSICAL_ENABLED}, separators=(",", ":")), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
