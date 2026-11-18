#!/usr/bin/env python3
"""ROS 2 adapter for the standardized frontier-mapping control flow.

The node only owns ``/cmd_vel_autonomy`` and ``/motion/lease``. It never writes
``/cmd_vel`` directly. Physical output is disabled unless both ROS parameters
``physical_enable`` and ``confirm_physical`` are true, and every non-zero
command is bounded by a short lease and the local watchdog.
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .explorer import FrontierExplorer, GridMap
from .coverage import CoverageController, VariantError, load_variant
from .policy import SafetyPolicy, physical_output_is_enabled
from .protocol import ActionProposal, WorldSnapshot
from .ros_preflight import normalize_tf_age
from .tour import TourController


class MappingController(Node):
    def __init__(self) -> None:
        super().__init__("xuegeros_agent_mapping_controller")
        self.declare_parameter("physical_enable", False)
        self.declare_parameter("confirm_physical", False)
        self.declare_parameter("control_hz", 10.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        # The firmware nominally emits scan at 10 Hz, but field logs measured
        # roughly 0.7-1.1 Hz. Keep a bounded window without treating a missing
        # publisher as healthy.
        self.declare_parameter("scan_timeout_s", 2.5)
        self.declare_parameter("odom_timeout_s", 1.0)
        self.declare_parameter("tf_timeout_s", 1.0)
        self.declare_parameter("map_timeout_s", 2.0)
        self.declare_parameter("min_battery_voltage", 7.0)
        self.declare_parameter("max_linear_mps", 0.04)
        self.declare_parameter("max_angular_rps", 0.3)
        self.declare_parameter("max_runtime_s", 300.0)
        self.declare_parameter("max_travel_m", 8.0)
        self.declare_parameter("saturation_window_s", 20.0)
        self.declare_parameter("growth_eps_cells", 10)
        self.declare_parameter("no_frontier_samples", 3)
        self.declare_parameter("return_timeout_s", 90.0)
        self.declare_parameter("target_reached_m", 0.22)
        self.declare_parameter("no_progress_timeout_s", 30.0)
        self.declare_parameter("obstacle_stop_m", 0.32)
        self.declare_parameter("replan_period_s", 3.0)
        self.declare_parameter("strategy", "tour")
        # Coverage lane geometry is a *variant* (docs/SIM-CONTRACT.md §2/§8.3):
        # a run is named by variant_id, never by loose lane numbers. The legacy
        # lane parameters stay declared so an older params file does not abort
        # the node -- their values are ignored, with one warning saying so.
        self.declare_parameter("variant_id", "room/legacy-guest/01")
        self.declare_parameter("variant_root", "")
        self.declare_parameter("lane_length_m", 1.0)
        self.declare_parameter("lane_width_m", 0.60)
        self.declare_parameter("max_lanes", 4)

        self.physical_enable = bool(self.get_parameter("physical_enable").value)
        self.confirm_physical = bool(self.get_parameter("confirm_physical").value)
        self.physical_output_enabled = physical_output_is_enabled(
            self.physical_enable, self.confirm_physical
        )
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout_s").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout_s").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_s").value)
        self.map_timeout = float(self.get_parameter("map_timeout_s").value)
        self.min_battery = float(self.get_parameter("min_battery_voltage").value)
        max_linear = float(self.get_parameter("max_linear_mps").value)
        max_angular = float(self.get_parameter("max_angular_rps").value)
        self.variant_id, self.variant_root = self._resolve_coverage_variant()

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_autonomy", 10)
        self.lease_pub = self.create_publisher(String, "/motion/lease", 10)
        self.status_pub = self.create_publisher(String, "/agent/mapping_status", 10)
        self.facts_pub = self.create_publisher(String, "/agent/tour_facts", 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
        # The firmware publishes battery_state with BEST_EFFORT QoS.
        self.create_subscription(BatteryState, "/battery_state", self._on_battery, qos_profile_sensor_data)
        self.create_subscription(String, "/xuegeros/tour", self._on_tour_command, 10)
        self.create_subscription(Imu, "/imu", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_scan: LaserScan | None = None
        self.last_scan_at = 0.0
        self.last_odom: Odometry | None = None
        self.last_odom_at = 0.0
        self.map_msg: OccupancyGrid | None = None
        self.map_at = 0.0
        self.tf_age_s: float | None = None
        self.battery_voltage: float | None = None
        self.battery_at = 0.0
        self.battery_age_s: float | None = None
        self.last_battery_stamp = None
        self.explorer = FrontierExplorer(
            max_linear_mps=max_linear,
            max_angular_rps=max_angular,
            obstacle_stop_m=float(self.get_parameter("obstacle_stop_m").value),
            replan_period_s=float(self.get_parameter("replan_period_s").value),
            max_runtime_s=float(self.get_parameter("max_runtime_s").value),
            max_travel_m=float(self.get_parameter("max_travel_m").value),
        )
        self.tour = TourController(
            max_linear_mps=max_linear,
            max_angular_rps=max_angular,
            obstacle_stop_m=float(self.get_parameter("obstacle_stop_m").value),
            replan_period_s=float(self.get_parameter("replan_period_s").value),
            max_runtime_s=float(self.get_parameter("max_runtime_s").value),
            max_travel_m=float(self.get_parameter("max_travel_m").value),
            saturation_window_s=float(self.get_parameter("saturation_window_s").value),
            growth_eps_cells=int(self.get_parameter("growth_eps_cells").value),
            no_frontier_samples=int(self.get_parameter("no_frontier_samples").value),
            return_timeout_s=float(self.get_parameter("return_timeout_s").value),
            target_reached_m=float(self.get_parameter("target_reached_m").value),
            min_battery_v=self.min_battery,
            no_progress_timeout_s=float(self.get_parameter("no_progress_timeout_s").value),
        )
        self.coverage = CoverageController(
            variant_id=self.variant_id,
            variant_root=self.variant_root,
            max_linear_mps=max_linear,
            max_angular_rps=max_angular,
            obstacle_stop_m=float(self.get_parameter("obstacle_stop_m").value),
            max_runtime_s=float(self.get_parameter("max_runtime_s").value),
            max_travel_m=float(self.get_parameter("max_travel_m").value),
            no_progress_timeout_s=float(self.get_parameter("no_progress_timeout_s").value),
        )
        coverage_variant = self.coverage.variant
        self.get_logger().info(
            "coverage variant=%s lane=%.2fx%.2f lanes=%d world_sha256=%s acceptance=%s",
            coverage_variant.variant_id,
            coverage_variant.lane_length_m,
            coverage_variant.lane_width_m,
            coverage_variant.max_lanes,
            coverage_variant.world_sha256,
            coverage_variant.acceptance,
        )
        self.strategy_name = str(self.get_parameter("strategy").value).lower()
        self.active = self.coverage if self.strategy_name == "coverage" else self.tour
        self.pending_start: dict[str, object] | None = None
        self.last_facts_at = 0.0
        self.last_imu: Imu | None = None
        self.last_imu_at = 0.0
        self.last_imu_yaw: float | None = None
        self.yaw_offset: float | None = None
        self.last_cmd: Twist | None = None
        self.last_cmd_at = 0.0
        # The policy validates the requested action even in shadow mode. The
        # separate emit flag below decides whether a non-zero pulse is sent.
        self.policy = SafetyPolicy(
            max_linear_mps=max_linear,
            max_angular_rps=max_angular,
            max_sensor_age_s=max(self.scan_timeout, self.odom_timeout, self.tf_timeout),
            max_scan_age_s=self.scan_timeout,
            max_odom_age_s=self.odom_timeout,
            max_tf_age_s=self.tf_timeout,
            max_map_age_s=self.map_timeout,
            min_battery_voltage=self.min_battery,
            require_battery=self.physical_enable and self.confirm_physical,
            require_map_freshness=True,
            require_mapping_healthy=True,
            physical_enabled=True,
        )
        self.state = "WAIT_READY"
        self.last_nonzero_at = 0.0
        self.timer = self.create_timer(1.0 / max(1.0, float(self.get_parameter("control_hz").value)), self._tick)
        self.get_logger().info(
            f"agent mapping controller ready physical={self.physical_enable} "
            f"confirmed={self.confirm_physical}; output=/cmd_vel_autonomy"
        )

    def _resolve_variant_root(self) -> str | None:
        """Where ``sim/contracts/variants`` lives on this deployment.

        The package may be installed by ``colcon`` far away from the repository,
        so the variant directory is looked up instead of assumed: explicit ROS
        parameter, then ``XUEGEROS_VARIANTS``, then the repository layout.
        """

        candidates: list[str] = []
        configured = str(self.get_parameter("variant_root").value or "").strip()
        if configured:
            candidates.append(configured)
        env = os.environ.get("XUEGEROS_VARIANTS", "").strip()
        if env:
            candidates.append(env)
        package = Path(__file__).resolve().parent
        candidates.append(str(package.parent / "sim" / "contracts" / "variants"))
        candidates.append(str(package / "variants"))
        for candidate in candidates:
            if Path(candidate).is_dir():
                return candidate
        return None

    def _resolve_coverage_variant(self) -> tuple[str, str | None]:
        variant_id = str(self.get_parameter("variant_id").value or "").strip()
        if not variant_id:
            raise RuntimeError("variant_id 不能为空：coverage 的车道几何只能来自变体文件")
        declared = {"lane_length_m": 1.0, "lane_width_m": 0.60, "max_lanes": 4}
        legacy = {name: self.get_parameter(name).value for name in declared}
        overridden = {name: value for name, value in legacy.items() if value != declared[name]}
        if overridden:
            self.get_logger().warning(
                "DEPRECATED 忽略 %s：coverage 车道几何由 variant_id=%s 从 sim/contracts/variants/ 提供；"
                "params 文件里改用 variant_id 即可消除这条警告",
                overridden,
                variant_id,
            )
        root = self._resolve_variant_root()
        try:
            load_variant(variant_id, root=root)
        except VariantError as exc:
            raise RuntimeError(
                f"coverage 变体不可用：{exc}。已尝试 variant_root={root!r}、XUEGEROS_VARIANTS、"
                "仓库布局 <package>/../../sim/contracts/variants"
            ) from exc
        return variant_id, root

    def _on_scan(self, message: LaserScan) -> None:
        self.last_scan = message
        self.last_scan_at = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        self.last_odom = message
        self.last_odom_at = time.monotonic()

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map_msg = message
        self.map_at = time.monotonic()

    def _on_battery(self, message: BatteryState) -> None:
        if math.isfinite(message.voltage) and message.voltage > 0:
            self.battery_voltage = float(message.voltage)
            self.battery_at = time.monotonic()
            self.last_battery_stamp = message.header.stamp
            self.battery_age_s = self._message_age(self.battery_at, message.header.stamp)

    def _on_imu(self, message: Imu) -> None:
        self.last_imu = message
        self.last_imu_at = time.monotonic()
        q = message.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if math.isfinite(norm) and 0.5 <= norm <= 1.5:
            self.last_imu_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        else:
            self.last_imu_yaw = None

    def _control_pose(self, pose: tuple[float, float, float]) -> tuple[float, float, float]:
        """Use IMU yaw for heading; the wheel odometry yaw under-reports badly."""

        if self.last_imu_yaw is None:
            return pose
        if self.yaw_offset is None:
            self.yaw_offset = pose[2] - self.last_imu_yaw
        return (pose[0], pose[1], self.last_imu_yaw + self.yaw_offset)

    def _on_cmd_vel(self, message: Twist) -> None:
        self.last_cmd = message
        self.last_cmd_at = time.monotonic()

    def _on_tour_command(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except ValueError:
            self.get_logger().warn("tour command is not JSON")
            return
        action = str(payload.get("action") or "").lower()
        if action == "start":
            self.pending_start = {
                "request_id": str(payload.get("request_id") or f"tour-{int(time.time())}")[:64],
                "source": str(payload.get("source") or "ws")[:32],
            }
        elif action in ("stop", "halt"):
            self.pending_start = None
            self.active.stop(reason=str(payload.get("reason") or "requested")[:120])
        else:
            self.get_logger().warn(f"unknown tour action {action!r}")

    def _control_odom_pose(self) -> tuple[float, float, float] | None:
        pose = self._odom_pose()
        if pose is None or self.last_imu_yaw is None:
            return pose
        return (pose[0], pose[1], self.last_imu_yaw)

    def _odom_pose(self) -> tuple[float, float, float] | None:
        if self.last_odom is None:
            return None
        pose = self.last_odom.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return pose.position.x, pose.position.y, yaw

    def _imu_facts(self) -> dict[str, object]:
        if self.last_imu is None:
            return {"orientation_valid": False}
        q = self.last_imu.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        valid = bool(math.isfinite(norm) and 0.5 <= norm <= 1.5)
        roll = pitch = None
        if valid:
            roll = math.degrees(
                math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
            )
            pitch = math.degrees(
                math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
            )
        gyro = self.last_imu.angular_velocity
        return {
            "orientation_valid": valid,
            "yaw_deg": None if self.last_imu_yaw is None else round(math.degrees(self.last_imu_yaw), 1),
            "roll_deg": None if roll is None else round(roll, 1),
            "pitch_deg": None if pitch is None else round(pitch, 1),
            "gyro_mag_rps": round(math.sqrt(gyro.x ** 2 + gyro.y ** 2 + gyro.z ** 2), 3),
            "age_s": None
            if not self.last_imu_at
            else round(self._message_age(self.last_imu_at, self.last_imu.header.stamp) or 0.0, 3),
        }

    def _publisher_counts(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for topic in ("/scan", "/odom", "/map", "/imu", "/cmd_vel"):
            try:
                counts[topic.strip("/")] = len(self.get_publishers_info_by_topic(topic))
            except Exception:
                counts[topic.strip("/")] = -1
        return counts

    def _publish_facts(self, facts: dict[str, object]) -> None:
        now = time.monotonic()
        payload = dict(facts)
        payload["ages"] = {
            "scan_s": None if not self.last_scan_at else round(now - self.last_scan_at, 3),
            "odom_s": None if not self.last_odom_at else round(now - self.last_odom_at, 3),
            "map_s": None if not self.map_at else round(now - self.map_at, 3),
            "tf_s": self.tf_age_s,
        }
        message = String()
        message.data = json.dumps({"t": "tour_facts", "ts": time.time(), **payload}, separators=(",", ":"))
        self.facts_pub.publish(message)

    def _tour_summary(self) -> dict[str, object]:
        # This summary is emitted from the status topic for BOTH strategies.
        # tour.py reports `return.dist_to_start_m`, coverage.py reports
        # `return.dist_to_home_m`; indexing either one directly crashed the node
        # with KeyError as soon as strategy:=coverage was selected (2026-11-15).
        snapshot = self.active.snapshot()
        return_block = snapshot.get("return") or {}
        distance = return_block.get("dist_to_start_m")
        if distance is None:
            distance = return_block.get("dist_to_home_m")
        return {
            "request_id": snapshot.get("request_id"),
            "phase": snapshot.get("phase"),
            "active": snapshot.get("active"),
            "physical": self.physical_output_enabled,
            "elapsed_s": snapshot.get("elapsed_s"),
            "dist_to_start_m": distance,
        }

    def _tour_step_inputs(self):
        scan = self.last_scan
        return (scan.ranges, scan.angle_min, scan.angle_increment) if scan is not None else ([], 0.0, 0.0)

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
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            self.tf_age_s = None
            return None
        stamp = Time.from_msg(transform_stamped.header.stamp)
        if stamp.nanoseconds <= 0:
            self.tf_age_s = None
            return None
        delta = (self.get_clock().now() - stamp).nanoseconds / 1_000_000_000.0
        self.tf_age_s = normalize_tf_age(delta)
        transform = transform_stamped.transform
        q = transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return transform.translation.x, transform.translation.y, yaw

    def _world(self, pose: tuple[float, float, float] | None) -> WorldSnapshot:
        now = time.monotonic()
        map_age = None if not self.map_at else now - self.map_at
        scan_age = None if not self.last_scan_at else now - self.last_scan_at
        odom_age = None if not self.last_odom_at else now - self.last_odom_at
        if self.last_scan is not None:
            scan_age = self._message_age(self.last_scan_at, self.last_scan.header.stamp)
        if self.last_odom is not None:
            odom_age = self._message_age(self.last_odom_at, self.last_odom.header.stamp)
        if self.map_msg is not None:
            map_age = self._message_age(self.map_at, self.map_msg.header.stamp)
        if self.battery_at:
            self.battery_age_s = self._message_age(self.battery_at, self.last_battery_stamp)
        mapping_healthy = bool(
            self.map_msg is not None
            and self.map_msg.data
            and map_age is not None
            and map_age <= self.map_timeout
            and scan_age is not None
            and scan_age <= self.scan_timeout
            and odom_age is not None
            and odom_age <= self.odom_timeout
            and self.tf_age_s is not None
            and 0 <= self.tf_age_s <= self.tf_timeout
        )
        return WorldSnapshot(
            scan_age_s=scan_age,
            odom_age_s=odom_age,
            tf_age_s=self.tf_age_s,
            map_age_s=map_age,
            map_present=self.map_msg is not None and bool(self.map_msg.data),
            mapping_healthy=mapping_healthy,
            battery_voltage=self.battery_voltage,
            battery_age_s=self.battery_age_s,
            micro_ros_session=self.last_odom_at > 0,
        )

    def _publish_lease(self, duration_ms: int) -> None:
        message = String()
        message.data = json.dumps(
            {
                "lease_id": f"agent-mapping-{int(time.time() * 1000)}",
                "expires_at": time.time() + max(2.0, duration_ms / 1000.0 + 1.0),
            },
            separators=(",", ":"),
        )
        self.lease_pub.publish(message)

    def _zero(self) -> None:
        self.cmd_pub.publish(Twist())

    def _emit_status(self, decision: dict[str, object]) -> None:
        message = String()
        message.data = json.dumps(
            {
                "state": self.state,
                "physical": self.physical_output_enabled,
                "physical_enable": self.physical_enable,
                "confirm_physical": self.confirm_physical,
                "tour": self._tour_summary(),
                **decision,
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(message)

    def _tick(self) -> None:
        pose = self._pose()
        world = self._world(pose)
        if self.map_msg is None or self.last_scan is None or pose is None:
            self.state = "WAIT_READY"
            self._zero()
            self._emit_status({"reason": "map_scan_or_tf_missing"})
            return
        grid = GridMap(
            self.map_msg.info.width,
            self.map_msg.info.height,
            self.map_msg.info.resolution,
            self.map_msg.info.origin.position.x,
            self.map_msg.info.origin.position.y,
            self.map_msg.data,
        )
        if self.pending_start is not None:
            self.yaw_offset = None
        map_pose = self._control_pose(pose)
        odom_pose = self._control_odom_pose()
        # Coverage is fixed-geometry sweeping and does not need the map frame;
        # gmapping's map->base pose lags or freezes at low scan rates, which made
        # lane progress and the trip home stall. Tour stays on map because
        # frontiers are map cells.
        strategy_pose = (
            odom_pose
            if self.strategy_name == "coverage" and odom_pose is not None
            else map_pose
        )
        self.active.observe(
            pose_odom=self._control_odom_pose(),
            battery_v=self.battery_voltage,
            publishers=self._publisher_counts(),
            imu=self._imu_facts(),
            actuator=(0.0, 0.0)
            if self.last_cmd is None
            else (self.last_cmd.linear.x, self.last_cmd.angular.z),
            actuator_age_s=None if not self.last_cmd_at else time.monotonic() - self.last_cmd_at,
            encoder_twist_mps=None if self.last_odom is None else self.last_odom.twist.twist.linear.x,
        )
        ranges, angle_min, angle_increment = self._tour_step_inputs()
        if self.pending_start is not None:
            self.active.start(
                now=time.monotonic(),
                request_id=str(self.pending_start["request_id"]),
                source=str(self.pending_start["source"]),
                grid=grid,
                pose_map=strategy_pose,
                pose_odom=self._control_odom_pose(),
                ranges=ranges,
                angle_min=angle_min,
                angle_increment=angle_increment,
                battery_v=self.battery_voltage,
                publishers=self._publisher_counts(),
                imu=self._imu_facts(),
            )
            self.pending_start = None
        if self.active.request_id is not None:
            now = time.monotonic()
            facts_due = now - self.last_facts_at >= 1.0
            action, facts = self.active.step(
                now=now,
                grid=grid,
                pose_map=strategy_pose,
                ranges=ranges,
                angle_min=angle_min,
                angle_increment=angle_increment,
                facts_due=facts_due,
            )
            if facts_due:
                self._publish_facts(facts)
                self.last_facts_at = now
            safety = self.policy.evaluate(action, world)
            if not safety.allowed:
                self.state = "BLOCKED"
                self._zero()
                self._emit_status({"reason": list(safety.reasons), "phase": facts["phase"]})
                return
            output = safety.clamped or ActionProposal(0.0, 0.0)
            self.state = str(facts["phase"])
            if self.physical_enable and self.confirm_physical and (
                abs(output.linear_mps) > 1e-9 or abs(output.angular_rps) > 1e-9
            ):
                self._publish_lease(output.duration_ms)
                command = Twist()
                command.linear.x = output.linear_mps
                command.angular.z = output.angular_rps
                self.cmd_pub.publish(command)
                self.last_nonzero_at = time.monotonic()
            else:
                self._zero()
            self._emit_status({"reason": f"tour:{facts['phase']}", "action": output.as_dict()})
            return
        decision = self.explorer.decide(
            grid,
            map_pose,
            self.last_scan.ranges,
            self.last_scan.angle_min,
            self.last_scan.angle_increment,
        )
        action = decision.proposal
        safety = self.policy.evaluate(action, world)
        if decision.state == "COMPLETE" or decision.state == "LIMIT":
            self.state = decision.state
            self._zero()
            self._emit_status({"reason": decision.reason, "target": decision.target})
            return
        if not safety.allowed:
            self.state = "BLOCKED"
            self._zero()
            self._emit_status({"reason": list(safety.reasons), "target": decision.target})
            return
        self.state = decision.state
        output = safety.clamped or ActionProposal(0.0, 0.0)
        if not (self.physical_enable and self.confirm_physical):
            self._zero()
            self._emit_status(
                {
                    "reason": "shadow_mode",
                    "planner_reason": decision.reason,
                    "target": decision.target,
                    "would_execute": output.as_dict(),
                }
            )
            return
        if abs(output.linear_mps) > 1e-9 or abs(output.angular_rps) > 1e-9:
            self._publish_lease(output.duration_ms)
            command = Twist()
            command.linear.x = output.linear_mps
            command.angular.z = output.angular_rps
            self.cmd_pub.publish(command)
            self.last_nonzero_at = time.monotonic()
        else:
            self._zero()
        self._emit_status({"reason": decision.reason, "target": decision.target, "action": output.as_dict()})

    def destroy_node(self) -> bool:
        try:
            self._zero()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingController()
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
