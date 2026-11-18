"""Read-only World/Gateway readiness probe used by the flow runner."""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any

from .policy import SafetyPolicy
from .protocol import ActionProposal, WorldSnapshot


def get_json(url: str) -> dict[str, Any]:
    request = urllib.request.Request(url, headers={"accept": "application/json"})
    with urllib.request.urlopen(request, timeout=3.0) as response:
        value = json.loads(response.read().decode("utf-8", "replace"))
    if not isinstance(value, dict):
        raise ValueError("upstream response is not an object")
    return value


def preflight(world_url: str | None, gateway_url: str | None) -> dict[str, Any]:
    result: dict[str, Any] = {"ok": True, "blockers": []}
    if not world_url:
        result["blockers"].append("world_url_missing")
    else:
        try:
            world_payload = get_json(world_url)
            world = WorldSnapshot.from_payload(world_payload)
            policy = SafetyPolicy(require_map=False, physical_enabled=True)
            probe = policy.evaluate(ActionProposal(0.0, 0.0, 100), world)
            result["world"] = {
                "scan_age_s": world.scan_age_s,
                "odom_age_s": world.odom_age_s,
                "tf_age_s": world.tf_age_s,
                "map_present": world.map_present,
                "micro_ros_session": world.micro_ros_session,
                "battery_voltage": world.battery_voltage,
            }
            result["blockers"].extend(probe.reasons)
        except (OSError, urllib.error.URLError, ValueError, json.JSONDecodeError) as exc:
            result["ok"] = False
            result["blockers"].append(f"world_unavailable:{str(exc)[:120]}")
    if not gateway_url:
        result["blockers"].append("gateway_url_missing")
    else:
        try:
            gateway = get_json(gateway_url.rstrip("/") + "/v1/status")
            result["gateway"] = gateway
            if not gateway.get("enabled", False):
                result["blockers"].append("motion_gateway_disabled")
        except (OSError, urllib.error.URLError, ValueError, json.JSONDecodeError) as exc:
            result["blockers"].append(f"gateway_unavailable:{str(exc)[:120]}")
    result["ok"] = not result["blockers"]
    return result
