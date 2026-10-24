from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class CmdVelWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("xuegeros_cmd_vel_watchdog")
        self.declare_parameter("input_topic", "/cmd_vel_safe_raw")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("lease_topic", "/motion/lease")
        self.declare_parameter("watchdog_timeout_sec", 0.2)
        self.declare_parameter("output_rate_hz", 20.0)
        self.declare_parameter("max_linear", 0.1)
        self.declare_parameter("max_angular", 0.5)
        self.declare_parameter("require_lease", True)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.lease_topic = str(self.get_parameter("lease_topic").value)
        self.watchdog_timeout_sec = float(
            self.get_parameter("watchdog_timeout_sec").value
        )
        self.output_rate_hz = float(self.get_parameter("output_rate_hz").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.require_lease = bool(self.get_parameter("require_lease").value)

        self._last_command_at = 0.0
        self._last_command = Twist()
        self._lease_id: str | None = None
        self._lease_expires_at = 0.0
        self._last_veto_log_at = 0.0

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self._on_command, 10)
        self.create_subscription(String, self.lease_topic, self._on_lease, 10)
        self.create_service(Trigger, "~/disarm", self._on_disarm)

        period = 1.0 / max(1.0, self.output_rate_hz)
        self.create_timer(period, self._publish)
        self.get_logger().info(
            f"watchdog input={self.input_topic} output={self.output_topic} "
            f"timeout={self.watchdog_timeout_sec:.3f}s "
            f"limits=({self.max_linear:.3f},{self.max_angular:.3f})"
        )

    def _on_command(self, msg: Twist) -> None:
        self._last_command_at = time.monotonic()
        self._last_command = msg

    def _on_lease(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            lease_id = str(payload["lease_id"])
            expires_at = float(payload["expires_at"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"invalid motion lease: {exc}")
            return
        if expires_at <= time.time():
            self.get_logger().warning("rejected expired motion lease")
            return
        self._lease_id = lease_id
        self._lease_expires_at = expires_at
        self.get_logger().info(f"motion lease accepted id={lease_id}")

    def _on_disarm(self, _request, response):
        self._lease_id = None
        self._lease_expires_at = 0.0
        self._last_command_at = 0.0
        self._publish_zero()
        response.success = True
        response.message = "disarmed; zero velocity published"
        return response

    def _lease_valid(self, now_wall: float) -> bool:
        return bool(
            not self.require_lease
            or (self._lease_id and now_wall < self._lease_expires_at)
        )

    def _publish(self) -> None:
        now_mono = time.monotonic()
        now_wall = time.time()
        command_fresh = (
            now_mono - self._last_command_at <= self.watchdog_timeout_sec
        )
        if command_fresh and self._lease_valid(now_wall):
            self.publisher.publish(self._clamped(self._last_command))
            return

        self._publish_zero()
        if now_mono - self._last_veto_log_at >= 2.0:
            self._last_veto_log_at = now_mono
            reason = "lease" if not self._lease_valid(now_wall) else "watchdog"
            self.get_logger().warning(f"zero velocity active: {reason}")

    def _clamped(self, command: Twist) -> Twist:
        output = Twist()
        output.linear.x = self._clamp(command.linear.x, self.max_linear)
        output.linear.y = self._clamp(command.linear.y, self.max_linear)
        output.linear.z = self._clamp(command.linear.z, self.max_linear)
        output.angular.x = self._clamp(command.angular.x, self.max_angular)
        output.angular.y = self._clamp(command.angular.y, self.max_angular)
        output.angular.z = self._clamp(command.angular.z, self.max_angular)
        return output

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-abs(limit), min(abs(limit), float(value)))

    def _publish_zero(self) -> None:
        self.publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_zero()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
