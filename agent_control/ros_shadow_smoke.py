"""Isolated ROS integration smoke test; all controller outputs must stay zero."""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time


def main() -> int:
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        print("refusing to run unless ROS_DOMAIN_ID=42", file=sys.stderr)
        return 2

    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState, LaserScan
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster

    class SyntheticWorld(Node):
        def __init__(self) -> None:
            super().__init__("agent_control_shadow_smoke_world")
            map_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
            self.odom_pub = self.create_publisher(Odometry, "/odom", qos_profile_sensor_data)
            self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
            self.battery_pub = self.create_publisher(BatteryState, "/battery_state", qos_profile_sensor_data)
            self.tf_pub = TransformBroadcaster(self)
            self.status: list[dict] = []
            self.commands: list[tuple[float, float]] = []
            self.create_subscription(String, "/agent/mapping_status", self.on_status, 10)
            self.create_subscription(Twist, "/cmd_vel_autonomy", self.on_command, 10)
            self.map_msg = self.make_map()
            self.create_timer(0.1, self.publish_world)

        def make_map(self) -> OccupancyGrid:
            message = OccupancyGrid()
            message.header.frame_id = "map"
            message.info.resolution = 0.1
            message.info.width = 40
            message.info.height = 40
            message.info.origin.position.x = -2.0
            message.info.origin.position.y = -2.0
            message.info.origin.orientation.w = 1.0
            cells = [-1] * (40 * 40)
            for y in range(10, 31):
                for x in range(10, 31):
                    cells[y * 40 + x] = 0
            message.data = cells
            return message

        def on_status(self, message: String) -> None:
            try:
                self.status.append(json.loads(message.data))
            except json.JSONDecodeError:
                pass

        def on_command(self, message: Twist) -> None:
            self.commands.append((float(message.linear.x), float(message.angular.z)))

        def publish_world(self) -> None:
            now = self.get_clock().now().to_msg()
            scan = LaserScan()
            scan.header.stamp = now
            scan.header.frame_id = "laser_frame"
            scan.angle_min = -math.pi
            scan.angle_increment = 2.0 * math.pi / 360
            scan.angle_max = scan.angle_min + 359 * scan.angle_increment
            scan.range_min = 0.02
            scan.range_max = 8.0
            scan.ranges = [3.0] * 360
            self.scan_pub.publish(scan)

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odom)

            self.map_msg.header.stamp = now
            self.map_pub.publish(self.map_msg)

            battery = BatteryState()
            battery.header.stamp = now
            battery.voltage = 8.1
            battery.percentage = 0.8
            self.battery_pub.publish(battery)

            for parent, child in (("map", "odom"), ("odom", "base_link")):
                transform = TransformStamped()
                transform.header.stamp = now
                transform.header.frame_id = parent
                transform.child_frame_id = child
                transform.transform.rotation.w = 1.0
                self.tf_pub.sendTransform(transform)

    rclpy.init()
    world = SyntheticWorld()
    executor_thread = threading.Thread(target=rclpy.spin, args=(world,), daemon=True)
    executor_thread.start()
    controller: subprocess.Popen | None = None
    try:
        time.sleep(1.0)
        preflight = subprocess.run(
            [sys.executable, "-m", "agent_control.ros_preflight", "--timeout", "3", "--require-battery"],
            capture_output=True,
            text=True,
            timeout=12,
            env=os.environ.copy(),
            check=False,
        )
        try:
            readiness = json.loads(preflight.stdout.splitlines()[-1])
        except (IndexError, json.JSONDecodeError):
            readiness = {"ok": False, "detail": preflight.stdout[-500:]}
        if preflight.returncode != 0 or not readiness.get("ok"):
            print(json.dumps({"ok": False, "stage": "preflight", "result": readiness}, ensure_ascii=False))
            return 1

        controller = subprocess.Popen(
            ["ros2", "run", "agent_control", "mapping_controller"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=os.environ.copy(),
        )
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and controller.poll() is None:
            if any(item.get("state") in {"MOVE", "AVOID", "REPLAN"} for item in world.status):
                break
            time.sleep(0.1)
        time.sleep(1.0)
        states = [item.get("state") for item in world.status]
        active = any(state in {"MOVE", "AVOID", "REPLAN"} for state in states)
        nonzero = [command for command in world.commands if abs(command[0]) > 1e-9 or abs(command[1]) > 1e-9]
        result = {
            "ok": active and not nonzero and controller.poll() is None,
            "preflight": readiness,
            "controller_states": states[-10:],
            "autonomy_command_count": len(world.commands),
            "nonzero_command_count": len(nonzero),
            "ros_domain_id": os.environ["ROS_DOMAIN_ID"],
        }
        print(json.dumps(result, ensure_ascii=False))
        return 0 if result["ok"] else 1
    except (OSError, subprocess.SubprocessError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)[:200]}, ensure_ascii=False))
        return 1
    finally:
        if controller is not None and controller.poll() is None:
            controller.send_signal(signal.SIGINT)
            try:
                controller.wait(timeout=4)
            except subprocess.TimeoutExpired:
                controller.terminate()
                try:
                    controller.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    controller.kill()
                    controller.wait(timeout=2)
        world.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        executor_thread.join(timeout=2)


if __name__ == "__main__":
    raise SystemExit(main())
