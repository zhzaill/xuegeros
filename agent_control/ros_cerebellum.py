#!/usr/bin/env python3
"""ROS 2 adapter for the cerebellum.

Subscribes to a *semantic* goal (``/xuegeros/goal``), turns it into a bounded
motion with scan geometry and odometry, and publishes it through the existing
safety chain. The cortex never writes wheel speeds.

Topic choice: ``/cmd_vel_nav`` (twist_mux priority 50) rather than
``/cmd_vel_autonomy`` (priority 20). The mapping controller and the command
bridge both own the autonomy slot; putting the cerebellum above it means a
stale autonomy pulse can never fight a live goal, and human teleop (priority
100) still outranks everything. No twist_mux configuration changes.

Shadow mode is the default: unless both ``physical_enable`` and
``confirm_physical`` are true, this node publishes *nothing* on the command
topic (not even zeros) and reports ``would_execute`` in its status. The chassis
previously paid a measured price for inbound Twist traffic, so silence is the
correct shadow behaviour.
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .cerebellum import Cerebellum, profile_scan
from .policy import physical_output_is_enabled
from .robotd import DaemonServer, RobotDaemon
from .skills import SkillTableError, load_table


class RosTourLink:
    """Thin pipe between robotd and the mapping controller's raw facts.

    robotd never grows a ROS dependency: it asks this object, and this object
    only publishes a command and caches the last facts message.
    """

    def __init__(self, node: Node) -> None:
        self.node = node
        self._facts: dict | None = None
        self._facts_at = 0.0
        self._status: dict | None = None
        self._status_at = 0.0
        self.publisher = node.create_publisher(String, "/xuegeros/tour", 10)
        node.create_subscription(String, "/agent/tour_facts", self._on_facts, 10)
        node.create_subscription(String, "/agent/mapping_status", self._on_status, 10)

    def _on_facts(self, message: String) -> None:
        try:
            self._facts = json.loads(message.data)
        except ValueError:
            return
        self._facts_at = time.monotonic()

    def _on_status(self, message: String) -> None:
        try:
            self._status = json.loads(message.data)
        except ValueError:
            return
        self._status_at = time.monotonic()

    def controller_age_s(self) -> float | None:
        return None if not self._facts_at else round(time.monotonic() - self._facts_at, 2)

    def status(self) -> dict | None:
        return self._facts

    def start(self, *, request_id: str, source: str) -> dict:
        # Build the preflight from the facts that actually arrived, then drop
        # them so robotd cannot mistake the previous DONE for this run.
        facts = self._facts or {}
        preflight = {
            "ages": facts.get("ages"),
            "publishers": facts.get("publishers"),
            "battery_v": (facts.get("envelope") or {}).get("battery_v"),
            "battery_min_v": (facts.get("envelope") or {}).get("battery_min_v"),
            "imu": facts.get("imu"),
        }
        self._facts = None
        self._facts_at = 0.0
        self.publisher.publish(
            String(data=json.dumps({"action": "start", "request_id": request_id, "source": source}))
        )
        return {
            "accepted": True,
            "request_id": request_id,
            "source": source,
            "physical": None if self._status is None else self._status.get("physical"),
            "controller_age_s": self.controller_age_s(),
            "preflight": preflight,
        }

    def stop(self, *, reason: str = "requested") -> None:
        self.publisher.publish(String(data=json.dumps({"action": "stop", "reason": reason})))


class CerebellumNode(Node):
    def __init__(self) -> None:
        super().__init__("xuegeros_cerebellum")
        self.declare_parameter("physical_enable", False)
        self.declare_parameter("confirm_physical", False)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("status_hz", 5.0)
        self.declare_parameter("scan_timeout_s", 1.5)
        self.declare_parameter("odom_timeout_s", 1.0)
        self.declare_parameter("max_linear_mps", 0.05)
        self.declare_parameter("max_angular_rps", 0.35)
        self.declare_parameter("min_clearance_m", 0.25)
        self.declare_parameter("min_clearance_turn_m", 0.12)
        self.declare_parameter("sectors", 12)
        self.declare_parameter("forward_distance_m", 0.12)
        self.declare_parameter("turn_deg", 30.0)
        self.declare_parameter("open_space_distance_m", 0.20)
        self.declare_parameter("goal_topic", "/xuegeros/goal")
        self.declare_parameter("priority_topic", "/xuegeros/priority")
        self.declare_parameter("cmd_topic", "/cmd_vel_nav")
        self.declare_parameter("lease_topic", "/motion/lease")
        self.declare_parameter("status_topic", "/xuegeros/cerebellum_status")
        # Empty = next to this package (skills.json); a malformed table refuses
        # to start the node instead of running with a silently different vocabulary.
        self.declare_parameter("skills_file", "")
        # robotd is this process: the control loop and the console socket share
        # one owner. The socket is only reachable through ssh -L in deployment.
        self.declare_parameter("console_enable", True)
        self.declare_parameter("console_socket", "/run/user/1000/xuegeros-robotd.sock")
        self.declare_parameter("console_token", "")
        self.declare_parameter("stream_hz", 50.0)

        self.physical_enable = bool(self.get_parameter("physical_enable").value)
        self.confirm_physical = bool(self.get_parameter("confirm_physical").value)
        self.physical = physical_output_is_enabled(self.physical_enable, self.confirm_physical)
        self.scan_timeout = float(self.get_parameter("scan_timeout_s").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout_s").value)
        self.status_hz = max(0.5, float(self.get_parameter("status_hz").value))

        skills_path = str(self.get_parameter("skills_file").value) or None
        try:
            self.skill_table = load_table(skills_path)
        except SkillTableError as exc:
            self.get_logger().error(f"skill table rejected: {exc}")
            raise
        self.cerebellum = Cerebellum(
            max_linear_mps=float(self.get_parameter("max_linear_mps").value),
            max_angular_rps=float(self.get_parameter("max_angular_rps").value),
            min_clearance_m=float(self.get_parameter("min_clearance_m").value),
            min_clearance_turn_m=float(self.get_parameter("min_clearance_turn_m").value),
            sectors=int(self.get_parameter("sectors").value),
            forward_distance_m=float(self.get_parameter("forward_distance_m").value),
            turn_rad=math.radians(float(self.get_parameter("turn_deg").value)),
            open_space_distance_m=float(self.get_parameter("open_space_distance_m").value),
            scan_timeout_s=self.scan_timeout,
            odom_timeout_s=self.odom_timeout,
            skill_table=self.skill_table,
        )

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_topic").value), 10)
        self.lease_pub = self.create_publisher(String, str(self.get_parameter("lease_topic").value), 10)
        self.status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self.create_subscription(
            String, str(self.get_parameter("goal_topic").value), self._on_goal, 10
        )
        self.create_subscription(
            String, str(self.get_parameter("priority_topic").value), self._on_priority, 10
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)

        # Console and control loop are the same object graph; the page can only
        # reach it through the Unix socket.
        self.tour_link = RosTourLink(self)
        self.daemon = RobotDaemon(self.cerebellum, physical=self.physical, tour=self.tour_link)
        self.console: DaemonServer | None = None
        if bool(self.get_parameter("console_enable").value):
            self.console = DaemonServer(
                self.daemon,
                socket_path=str(self.get_parameter("console_socket").value),
                state_hz=float(self.get_parameter("stream_hz").value),
                token=str(self.get_parameter("console_token").value),
            )
            try:
                self.console.start()
            except OSError as exc:
                self.get_logger().error(f"console socket unavailable: {exc}")
                self.console = None

        self.last_scan: LaserScan | None = None
        self.last_scan_at = 0.0
        self.last_odom: Odometry | None = None
        self.last_odom_at = 0.0
        self.last_lease_at = 0.0
        self.last_status_at = 0.0
        self.last_state = ""
        self.set_result: dict[str, object] | None = None
        self.priority_result: dict[str, object] | None = None
        self._last_decision: dict[str, object] | None = None

        control_period = 1.0 / max(5.0, float(self.get_parameter("control_hz").value))
        self.create_timer(control_period, self._tick)
        self.get_logger().info(
            f"cerebellum ready physical={self.physical} "
            f"cmd={self.get_parameter('cmd_topic').value} "
            f"goal={self.get_parameter('goal_topic').value} "
            f"skills={','.join(self.skill_table.names())} "
            f"skills_version={self.skill_table.version} source={self.skill_table.source}"
        )

    # --- inputs -------------------------------------------------------------
    def _on_scan(self, message: LaserScan) -> None:
        self.last_scan = message
        self.last_scan_at = time.monotonic()
        self.daemon.set_scan(
            message.angle_min,
            message.angle_increment,
            message.ranges,
            age_s=0.0,
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        self.daemon.set_map(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin=(float(message.info.origin.position.x), float(message.info.origin.position.y)),
            data=message.data,
        )

    def _on_odom(self, message: Odometry) -> None:
        self.last_odom = message
        self.last_odom_at = time.monotonic()

    def _on_goal(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("goal body must be a JSON object")
        except (ValueError, json.JSONDecodeError) as exc:
            self.set_result = {"ok": False, "accepted": False, "error": "invalid_json", "detail": str(exc)[:160]}
            self._publish_status(force=True)
            return
        result = self.cerebellum.set_goal(payload)
        result.setdefault("goal_id", payload.get("goal_id"))
        self.set_result = result
        self._publish_status(force=True)

    def _on_priority(self, message: String) -> None:
        """An L2 opinion: veto + per-sector preference, valid for its TTL."""

        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("priority body must be a JSON object")
        except (ValueError, json.JSONDecodeError) as exc:
            self.priority_result = {
                "ok": False,
                "accepted": False,
                "error": "invalid_json",
                "detail": str(exc)[:160],
            }
            self._publish_status(force=True)
            return
        result = self.cerebellum.set_priority(payload)
        if payload.get("request_id") is not None:
            result["request_id"] = str(payload.get("request_id"))[:64]
        self.priority_result = result
        self._publish_status(force=True)

    # --- ages / pose --------------------------------------------------------
    def _message_age(self, received_at: float, stamp_message) -> float | None:
        if not received_at:
            return None
        source_stamp = Time.from_msg(stamp_message)
        if source_stamp.nanoseconds <= 0:
            return None
        source_age = (self.get_clock().now() - source_stamp).nanoseconds / 1_000_000_000.0
        if source_age < 0:
            return source_age
        return max(time.monotonic() - received_at, source_age)

    def _pose(self) -> tuple[float, float, float] | None:
        odom = self.last_odom
        if odom is None:
            return None
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        return float(position.x), float(position.y), yaw

    # --- output -------------------------------------------------------------
    def _publish_lease(self) -> None:
        now = time.time()
        if now - self.last_lease_at < 1.0:
            return
        self.last_lease_at = now
        message = String()
        message.data = json.dumps(
            {
                "lease_id": f"cerebellum-{int(now * 1000)}",
                "expires_at": now + 2.0,
            },
            separators=(",", ":"),
        )
        self.lease_pub.publish(message)

    def _publish_command(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        # /motion/lease is published on the same topic the watchdog reads.
        self._publish_lease()
        self.cmd_pub.publish(message)

    def _publish_status(self, *, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_status_at < 1.0 / self.status_hz:
            return
        self.last_status_at = now
        payload = {
            "node": "xuegeros_cerebellum",
            "physical": self.physical,
            "physical_enable": self.physical_enable,
            "confirm_physical": self.confirm_physical,
            "goal_topic": str(self.get_parameter("goal_topic").value),
            "cmd_topic": str(self.get_parameter("cmd_topic").value),
            "skills_version": self.skill_table.version,
            "skills_source": self.skill_table.source,
            "skills": self.skill_table.names(),
            "updated_at": time.time(),
            **(self._last_decision or {}),
        }
        if self.set_result is not None:
            payload["last_goal_request"] = self.set_result
            # Echo the goal_id at the top level so the command bridge can ack a
            # submission without parsing two different shapes.
            if self.set_result.get("goal_id"):
                payload["goal_id"] = self.set_result["goal_id"]
                if self.set_result.get("state"):
                    payload["state"] = self.set_result["state"]
        if self.priority_result is not None:
            payload["last_priority_request"] = self.priority_result
            if self.priority_result.get("ok"):
                payload["priority_accepted"] = True
            elif self.priority_result.get("error"):
                payload["priority_error"] = self.priority_result["error"]
        guide = self.cerebellum.guide
        payload["priority_fresh"] = guide is not None and guide.expires_at > time.monotonic()
        if guide is not None:
            payload["priority_mode"] = guide.mode
            payload["priority_age_s"] = round(max(0.0, time.monotonic() - guide.set_at), 3)
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.status_pub.publish(message)

    # --- loop ---------------------------------------------------------------
    def _tick(self) -> None:
        scan = self.last_scan
        if scan is None:
            # No scan at all is the same contract as a stale one: the cerebellum
            # cancels and emits its zero burst, so a latched nonzero command is
            # never left behind.
            profile = profile_scan([], 0.0, 1.0, sectors=self.cerebellum.sectors)
            scan_age = None
            odom_age = (
                self._message_age(self.last_odom_at, self.last_odom.header.stamp)
                if self.last_odom is not None
                else None
            )
        else:
            scan_age = self._message_age(self.last_scan_at, scan.header.stamp)
            odom_age = (
                self._message_age(self.last_odom_at, self.last_odom.header.stamp)
                if self.last_odom is not None
                else None
            )
            profile = profile_scan(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                sectors=self.cerebellum.sectors,
                front_half_angle_rad=self.cerebellum.front_half_angle,
            )
        decision = self.daemon.tick(
            now=time.monotonic(),
            profile=profile,
            pose=self._pose(),
            scan_age_s=scan_age,
            odom_age_s=odom_age,
        )
        if self.physical and decision.publish:
            self._publish_command(decision.linear_mps, decision.angular_rps)
        self._last_decision = decision.as_status()
        self._publish_status()

    def destroy_node(self) -> bool:
        try:
            if self.console is not None:
                self.console.stop()
        except Exception:
            pass
        try:
            if self.physical:
                self._publish_command(0.0, 0.0)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CerebellumNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
