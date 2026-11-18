"""Wire-level objects shared by the AI plugin and the ROS controller."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Mapping


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _number(value: Any, default: float | None = None) -> float | None:
    return float(value) if _finite(value) else default


@dataclass(frozen=True)
class ActionProposal:
    """A short-horizon differential-drive intent.

    The proposal is not a motor command. It is validated again by the policy
    and, when enabled, converted into a short gateway lease pulse.
    """

    linear_mps: float
    angular_rps: float
    duration_ms: int = 250
    source: str = "agent"
    rationale: str = ""
    confidence: float | None = None

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "ActionProposal":
        linear = value.get("linear_mps", value.get("linear", 0.0))
        angular = value.get("angular_rps", value.get("angular", 0.0))
        duration = value.get("duration_ms", 250)
        if not _finite(linear) or not _finite(angular):
            raise ValueError("linear and angular must be finite numbers")
        if not isinstance(duration, (int, float)) or int(duration) != duration:
            raise ValueError("duration_ms must be an integer")
        duration_i = int(duration)
        if duration_i < 100 or duration_i > 1000:
            raise ValueError("duration_ms must be between 100 and 1000")
        confidence = value.get("confidence")
        if confidence is not None and not _finite(confidence):
            raise ValueError("confidence must be finite")
        return cls(
            linear_mps=float(linear),
            angular_rps=float(angular),
            duration_ms=duration_i,
            source=str(value.get("source", "agent"))[:80],
            rationale=str(value.get("rationale", ""))[:500],
            confidence=None if confidence is None else float(confidence),
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "linear_mps": self.linear_mps,
            "angular_rps": self.angular_rps,
            "duration_ms": self.duration_ms,
            "source": self.source,
            "rationale": self.rationale,
            "confidence": self.confidence,
        }


@dataclass(frozen=True)
class WorldSnapshot:
    """Minimal world contract required before an agent may act."""

    observed_at: float = field(default_factory=time.time)
    scan_age_s: float | None = None
    odom_age_s: float | None = None
    tf_age_s: float | None = None
    map_age_s: float | None = None
    map_present: bool = False
    mapping_healthy: bool = False
    battery_voltage: float | None = None
    battery_age_s: float | None = None
    battery_percentage: float | None = None
    micro_ros_session: bool = False
    camera_state: str | None = None
    raw: Mapping[str, Any] = field(default_factory=dict, repr=False)

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "WorldSnapshot":
        mapping = payload.get("mapping") if isinstance(payload.get("mapping"), Mapping) else {}
        status = payload.get("status") if isinstance(payload.get("status"), Mapping) else {}
        navigation = payload.get("navigation") if isinstance(payload.get("navigation"), Mapping) else {}
        battery = status.get("battery") if isinstance(status.get("battery"), Mapping) else {}
        scan = mapping.get("scan") if isinstance(mapping.get("scan"), Mapping) else {}
        odom = mapping.get("odom") if isinstance(mapping.get("odom"), Mapping) else {}
        tf = mapping.get("tf") if isinstance(mapping.get("tf"), Mapping) else {}
        map_meta = mapping.get("map") if isinstance(mapping.get("map"), Mapping) else {}
        rates = mapping.get("rates") if isinstance(mapping.get("rates"), Mapping) else {}
        observed = _number(payload.get("observed_at"), time.time()) or time.time()

        def age(item: Mapping[str, Any]) -> float | None:
            value = _number(item.get("age_s"), None)
            if value is not None:
                return value
            stamp = _number(item.get("observed_at"), None)
            return None if stamp is None else time.time() - stamp

        scan_evidence = bool(
            mapping.get("scan_age_s") is not None
            or any(key in scan for key in ("age_s", "observed_at"))
        )
        odom_evidence = (
            mapping.get("odom_age_s") is not None
            or any(key in odom for key in ("age_s", "observed_at"))
        )
        tf_evidence = bool(
            mapping.get("tf_age_s") is not None
            or any(key in tf for key in ("age_s", "observed_at"))
        )
        map_evidence = bool(
            mapping.get("map_age_s") is not None
            or any(key in map_meta for key in ("age_s", "stamp", "observed_at"))
            or mapping.get("map_updated")
        )
        return cls(
            observed_at=observed,
            scan_age_s=(
                _number(mapping.get("scan_age_s"), None)
                if mapping.get("scan_age_s") is not None
                else age(scan) if scan_evidence else None
            ),
            odom_age_s=(
                _number(mapping.get("odom_age_s"), None)
                if mapping.get("odom_age_s") is not None
                else age(odom) if odom_evidence else None
            ),
            tf_age_s=(
                _number(mapping.get("tf_age_s"), None)
                if mapping.get("tf_age_s") is not None
                else age(tf) if tf_evidence else None
            ),
            map_age_s=(
                _number(mapping.get("map_age_s"), None)
                if mapping.get("map_age_s") is not None
                else age(map_meta) if map_evidence else None
            ),
            map_present=bool(mapping.get("map_present", mapping.get("map", {}).get("present", False)))
            if isinstance(mapping.get("map", {}), Mapping)
            else bool(mapping.get("map_present", False)),
            mapping_healthy=bool(mapping.get("mapping_healthy", navigation.get("mapping_healthy", False))),
            battery_voltage=_number(battery.get("voltage"), _number(status.get("battery_voltage"), None)),
            battery_age_s=age(battery) if any(key in battery for key in ("age_s", "observed_at")) else None,
            battery_percentage=_number(battery.get("percentage"), None),
            micro_ros_session=bool(
                status.get(
                    "micro_ros_session",
                    status.get(
                        "agent_session",
                        status.get("micro_ros", {}).get("session_observed", navigation.get("micro_ros_session", False))
                        if isinstance(status.get("micro_ros", {}), Mapping)
                        else navigation.get("micro_ros_session", False),
                    ),
                )
            ),
            camera_state=str(payload.get("vision", {}).get("camera_state"))
            if isinstance(payload.get("vision"), Mapping) and payload.get("vision", {}).get("camera_state") is not None
            else None,
            raw=payload,
        )


def normalize_proposal(value: Mapping[str, Any]) -> dict[str, Any]:
    """Return a stable JSON record for logs and plugin responses."""

    proposal = ActionProposal.from_mapping(value)
    return proposal.as_dict()
