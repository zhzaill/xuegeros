#!/usr/bin/env python3
"""Short physical motion pulse routed through xuegeros_safety."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger


MAX_LINEAR = 0.1
MAX_ANGULAR = 0.5
MAX_DURATION_SEC = 1.0


class MotionBackend(Node):
    def __init__(self) -> None:
        super().__init__("xuegeros_motion_backend")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_autonomy", 10)
        self.lease_pub = self.create_publisher(String, "/motion/lease", 10)
        self.path_counts = {
            "/cmd_vel_autonomy": 0,
            "/cmd_vel_safe_raw": 0,
            "/cmd_vel": 0,
        }
        self.path_nonzero = {
            "/cmd_vel_autonomy": 0,
            "/cmd_vel_safe_raw": 0,
            "/cmd_vel": 0,
        }
        self.path_max_linear = {
            "/cmd_vel_autonomy": 0.0,
            "/cmd_vel_safe_raw": 0.0,
            "/cmd_vel": 0.0,
        }
        self.create_subscription(
            Twist,
            "/cmd_vel_autonomy",
            lambda msg: self._on_path("/cmd_vel_autonomy", msg),
            10,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel_safe_raw",
            lambda msg: self._on_path("/cmd_vel_safe_raw", msg),
            10,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            lambda msg: self._on_path("/cmd_vel", msg),
            10,
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.disarm = self.create_client(
            Trigger, "/xuegeros_cmd_vel_watchdog/disarm"
        )
        self.last_odom: Odometry | None = None
        self.last_odom_at = 0.0
        self.last_scan_at = 0.0

    def _on_odom(self, msg: Odometry) -> None:
        self.last_odom = msg
        self.last_odom_at = time.monotonic()

    def _on_scan(self, _msg: LaserScan) -> None:
        self.last_scan_at = time.monotonic()

    def _on_path(self, topic: str, msg: Twist) -> None:
        self.path_counts[topic] += 1
        if abs(msg.linear.x) > 0.0001 or abs(msg.angular.z) > 0.0001:
            self.path_nonzero[topic] += 1
        self.path_max_linear[topic] = max(
            self.path_max_linear[topic], abs(msg.linear.x)
        )

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, timeout_sec: float = 8.0) -> dict[str, bool]:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            ready = {
                "odom_fresh": now - self.last_odom_at <= 1.0,
                "scan_fresh": now - self.last_scan_at <= 1.0,
                "mux_subscribed": self.cmd_pub.get_subscription_count() >= 1,
                "lease_subscribed": self.lease_pub.get_subscription_count() >= 1,
            }
            if all(ready.values()):
                return ready
        raise RuntimeError(f"motion safety path not ready: {ready}")

    def publish_lease(self, duration_sec: float) -> None:
        lease = {
            "lease_id": f"move-test-{int(time.time() * 1000)}",
            "expires_at": time.time() + max(2.0, duration_sec + 1.0),
        }
        msg = String()
        msg.data = json.dumps(lease, separators=(",", ":"))
        self.lease_pub.publish(msg)
        self.spin_for(0.2)

    def publish_command(self, linear: float, angular: float, duration_sec: float) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        rate_period = 0.05
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            self.cmd_pub.publish(msg)
            self.spin_for(rate_period)

    def publish_zero(self, hold_sec: float = 0.6) -> None:
        deadline = time.monotonic() + hold_sec
        while time.monotonic() < deadline:
            self.cmd_pub.publish(Twist())
            self.spin_for(0.05)

    def call_disarm(self) -> bool:
        if not self.disarm.wait_for_service(timeout_sec=2.0):
            return False
        future = self.disarm.call_async(Trigger.Request())
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                try:
                    return bool(future.result().success)
                except Exception:
                    return False
        return False

    def odom_snapshot(self) -> dict[str, float] | None:
        if self.last_odom is None:
            return None
        pose = self.last_odom.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        twist = self.last_odom.twist.twist
        return {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "yaw": yaw,
            "linear_x": twist.linear.x,
            "angular_z": twist.angular.z,
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--linear", type=float, default=0.03)
    parser.add_argument("--angular", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--confirm-physical", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.confirm_physical:
        raise SystemExit("physical confirmation is required")
    if abs(args.linear) > MAX_LINEAR or abs(args.angular) > MAX_ANGULAR:
        raise SystemExit("velocity exceeds safety limits")
    if not (0.1 <= args.duration <= MAX_DURATION_SEC):
        raise SystemExit("duration must be between 0.1 and 1.0 seconds")

    rclpy.init()
    node = MotionBackend()
    result: dict[str, object] = {
        "requested": {
            "linear": args.linear,
            "angular": args.angular,
            "duration_sec": args.duration,
        }
    }
    try:
        result["ready"] = node.wait_ready()
        result["before"] = node.odom_snapshot()
        node.publish_lease(args.duration)
        node.publish_command(args.linear, args.angular, args.duration)
        node.publish_zero()
        node.spin_for(0.5)
        result["after"] = node.odom_snapshot()
        result["disarmed"] = node.call_disarm()
        result["path_counts"] = node.path_counts
        result["path_nonzero"] = node.path_nonzero
        result["path_max_linear"] = node.path_max_linear
        result["ok"] = True
    except Exception as exc:
        node.publish_zero()
        result["ok"] = False
        result["error"] = str(exc)
        result["after_error"] = node.odom_snapshot()
    finally:
        print(json.dumps(result, ensure_ascii=False, separators=(",", ":")))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
