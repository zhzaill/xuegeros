"""Fail-closed policy shared by exploration and AI conversation control."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from .protocol import ActionProposal, WorldSnapshot


def physical_output_is_enabled(physical_enable: bool, confirm_physical: bool) -> bool:
    return bool(physical_enable and confirm_physical)


@dataclass(frozen=True)
class SafetyDecision:
    allowed: bool
    reasons: tuple[str, ...] = ()
    clamped: ActionProposal | None = None

    def as_dict(self) -> dict[str, object]:
        return {
            "allowed": self.allowed,
            "reasons": list(self.reasons),
            "clamped": self.clamped.as_dict() if self.clamped else None,
        }


@dataclass(frozen=True)
class SafetyPolicy:
    max_linear_mps: float = 0.05
    max_angular_rps: float = 0.35
    min_duration_ms: int = 100
    max_duration_ms: int = 800
    max_sensor_age_s: float = 0.6
    max_scan_age_s: float | None = None
    max_odom_age_s: float | None = None
    max_tf_age_s: float | None = None
    max_map_age_s: float = 2.0
    max_battery_age_s: float = 2.5
    min_battery_voltage: float = 7.0
    require_battery: bool = False
    require_map_freshness: bool = False
    require_map: bool = True
    require_mapping_healthy: bool = True
    require_micro_ros: bool = True
    physical_enabled: bool = False

    def _health_reasons(self, world: WorldSnapshot) -> list[str]:
        reasons: list[str] = []
        if self.require_map and not world.map_present:
            reasons.append("map_missing")
        if self.require_mapping_healthy and not world.mapping_healthy:
            reasons.append("mapping_unhealthy")
        if self.require_micro_ros and not world.micro_ros_session:
            reasons.append("micro_ros_session_missing")
        if self.require_map_freshness:
            if world.map_age_s is None:
                reasons.append("map_freshness_unknown")
            elif not math.isfinite(world.map_age_s) or world.map_age_s < 0:
                reasons.append("map_freshness_invalid")
            elif world.map_age_s > self.max_map_age_s:
                reasons.append("map_stale")
        for name, age, limit in (
            (
                "scan",
                world.scan_age_s,
                self.max_sensor_age_s if self.max_scan_age_s is None else self.max_scan_age_s,
            ),
            (
                "odom",
                world.odom_age_s,
                self.max_sensor_age_s if self.max_odom_age_s is None else self.max_odom_age_s,
            ),
            (
                "tf",
                world.tf_age_s,
                self.max_sensor_age_s if self.max_tf_age_s is None else self.max_tf_age_s,
            ),
        ):
            if age is None:
                reasons.append(f"{name}_freshness_unknown")
            elif not math.isfinite(age) or age < 0:
                reasons.append(f"{name}_freshness_invalid")
            elif age > limit:
                reasons.append(f"{name}_stale")
        if self.require_battery and world.battery_voltage is None:
            reasons.append("battery_unknown")
        if self.require_battery:
            if world.battery_age_s is None:
                reasons.append("battery_freshness_unknown")
            elif not math.isfinite(world.battery_age_s) or world.battery_age_s < 0:
                reasons.append("battery_freshness_invalid")
            elif world.battery_age_s > self.max_battery_age_s:
                reasons.append("battery_stale")
        if world.battery_voltage is not None:
            if not math.isfinite(world.battery_voltage):
                reasons.append("battery_invalid")
            elif world.battery_voltage < self.min_battery_voltage:
                reasons.append("battery_low")
        return reasons

    def evaluate(self, proposal: ActionProposal, world: WorldSnapshot) -> SafetyDecision:
        reasons = self._health_reasons(world)
        values = (proposal.linear_mps, proposal.angular_rps)
        if not all(math.isfinite(value) for value in values):
            reasons.append("velocity_not_finite")
        if proposal.duration_ms < self.min_duration_ms or proposal.duration_ms > self.max_duration_ms:
            reasons.append("duration_out_of_range")
        if not self.physical_enabled and any(abs(value) > 1e-9 for value in values):
            reasons.append("physical_motion_disabled")
        if reasons:
            return SafetyDecision(False, tuple(dict.fromkeys(reasons)))

        clamped = ActionProposal(
            linear_mps=max(-self.max_linear_mps, min(self.max_linear_mps, proposal.linear_mps)),
            angular_rps=max(-self.max_angular_rps, min(self.max_angular_rps, proposal.angular_rps)),
            duration_ms=proposal.duration_ms,
            source=proposal.source,
            rationale=proposal.rationale,
            confidence=proposal.confidence,
        )
        return SafetyDecision(True, (), clamped)

    def stop(self, source: str = "safety") -> ActionProposal:
        return ActionProposal(0.0, 0.0, self.min_duration_ms, source, "fail-safe stop")


def combine_reasons(*groups: Iterable[str]) -> tuple[str, ...]:
    return tuple(dict.fromkeys(reason for group in groups for reason in group))
