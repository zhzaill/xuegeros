"""Read-only live ROS checks required before an automated mapping run."""

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Any


def normalize_tf_age(age_s: float) -> float:
    """GMapping deliberately stamps map->odom one transform period ahead."""
    return 0.0 if -0.1 <= age_s < 0.0 else age_s


def evaluate_evidence(evidence: dict[str, Any], *, require_battery: bool = False) -> dict[str, Any]:
    limits = {"scan": 2.5, "odom": 1.0, "tf": 1.0, "map": 2.0}
    blockers: list[str] = []
    for topic in ("scan", "odom", "map"):
        count = evidence.get(f"{topic}_publishers")
        if count != 1:
            blockers.append(f"{topic}_publisher_count_{count if count is not None else 'unknown'}")
    if evidence.get("scan_valid_ranges", 0) < 12:
        blockers.append("scan_invalid_or_missing")
    for name, limit in limits.items():
        age = evidence.get(f"{name}_age_s")
        if age is None:
            blockers.append(f"{name}_freshness_unknown")
        elif not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0:
            blockers.append(f"{name}_freshness_invalid")
        elif age > limit:
            blockers.append(f"{name}_stale")
    if not evidence.get("map_to_base_tf", False):
        blockers.append("map_to_base_tf_missing")
    if require_battery:
        voltage = evidence.get("battery_voltage")
        if voltage is None:
            blockers.append("battery_unknown")
        elif not isinstance(voltage, (int, float)) or not math.isfinite(voltage):
            blockers.append("battery_invalid")
        elif voltage < 7.0:
            blockers.append("battery_low")
        battery_age = evidence.get("battery_age_s")
        if battery_age is None:
            blockers.append("battery_freshness_unknown")
        elif not isinstance(battery_age, (int, float)) or not math.isfinite(battery_age) or battery_age < 0:
            blockers.append("battery_freshness_invalid")
        elif battery_age > 2.5:
            blockers.append("battery_stale")
    return {"ok": not blockers, "blockers": blockers, "evidence": evidence}


def collect_evidence(timeout_s: float) -> dict[str, Any]:
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import BatteryState, LaserScan
    from tf2_ros import Buffer, TransformListener

    class Probe(Node):
        def __init__(self) -> None:
            super().__init__("xuegeros_mapping_preflight")
            transient_map_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.received: dict[str, float | None] = {"scan": None, "odom": None, "map": None, "battery": None}
            self.stamp_ns: dict[str, int | None] = {"scan": None, "odom": None, "map": None, "battery": None}
            self.valid_ranges = 0
            self.battery_voltage: float | None = None
            self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid, "/map", self.on_map, transient_map_qos)
            self.create_subscription(BatteryState, "/battery_state", self.on_battery, qos_profile_sensor_data)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        def hit(self, name: str, stamp) -> None:
            self.received[name] = time.monotonic()
            stamp_time = Time.from_msg(stamp)
            self.stamp_ns[name] = stamp_time.nanoseconds if stamp_time.nanoseconds > 0 else None

        def on_scan(self, message: LaserScan) -> None:
            self.valid_ranges = sum(1 for value in message.ranges if math.isfinite(value) and value > 0)
            self.hit("scan", message.header.stamp)

        def on_odom(self, message: Odometry) -> None:
            self.hit("odom", message.header.stamp)

        def on_map(self, message: OccupancyGrid) -> None:
            self.hit("map", message.header.stamp)

        def on_battery(self, message: BatteryState) -> None:
            if math.isfinite(message.voltage) and message.voltage > 0:
                self.battery_voltage = float(message.voltage)
                self.hit("battery", message.header.stamp)

        def evidence(self) -> dict[str, Any]:
            now_mono = time.monotonic()
            now_ros_ns = self.get_clock().now().nanoseconds

            def age(name: str) -> float | None:
                received = self.received[name]
                source_stamp = self.stamp_ns[name]
                if received is None or source_stamp is None:
                    return None
                source_age = (now_ros_ns - source_stamp) / 1_000_000_000.0
                if source_age < 0:
                    return source_age
                return round(max(now_mono - received, source_age), 3)

            ages = {f"{name}_age_s": age(name) for name in ("scan", "odom", "map", "battery")}
            try:
                publishers = {
                    name: len(self.get_publishers_info_by_topic(f"/{name}"))
                    for name in ("scan", "odom", "map")
                }
            except Exception:
                publishers = {name: None for name in ("scan", "odom", "map")}
            try:
                stamped = self.tf_buffer.lookup_transform(
                    "map", "base_link", Time(), timeout=Duration(seconds=0.05)
                )
                stamp = Time.from_msg(stamped.header.stamp)
                delta = (self.get_clock().now() - stamp).nanoseconds / 1_000_000_000.0
                tf_age = normalize_tf_age(delta) if stamp.nanoseconds > 0 else None
                tf_present = stamp.nanoseconds > 0
            except Exception:
                tf_age = None
                tf_present = False
            return {
                **{f"{name}_publishers": count for name, count in publishers.items()},
                **ages,
                "tf_age_s": None if tf_age is None else round(tf_age, 3),
                "map_to_base_tf": tf_present,
                "scan_valid_ranges": self.valid_ranges,
                "battery_voltage": self.battery_voltage,
            }

    rclpy.init()
    node = Probe()
    try:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.evidence()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--require-battery", action="store_true")
    args = parser.parse_args()
    if args.timeout <= 0 or args.timeout > 60:
        parser.error("--timeout must be between 0 and 60 seconds")
    try:
        evidence = collect_evidence(args.timeout)
        result = evaluate_evidence(evidence, require_battery=args.require_battery)
    except Exception as exc:
        result = {"ok": False, "blockers": ["ros_probe_failed"], "detail": str(exc)[:200]}
    print(json.dumps(result, ensure_ascii=False, separators=(",", ":")))
    return 0 if result["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
