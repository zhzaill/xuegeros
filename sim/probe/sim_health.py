#!/usr/bin/env python3
"""One command health check for the xuegeros shadow (ROS_DOMAIN_ID=42).

Prints a single JSON line: JSON_HEALTH={...}

Checks, in this order (each one is a question the manual used to answer in prose):
  rates      /scan /odom /imu measured from source stamps
  publishers how many publishers each topic has right now
  tf         odom->base_footprint and map->base_link, looked up with tf2
  map        size, resolution, known/occupied cells, update rate
  robotd     WebSocket frame rates over the shadow Unix socket (token read from
             the params file and never printed)

The tf lookup uses rclpy.duration.Duration; passing a float there raises
"Can't compare duration with object of type float" (the bug that made the
previous probe useless).
"""

import base64
import json
import os
import socket
import struct
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String  # noqa: F401  (kept for parity with probes)
from tf2_ros import Buffer, TransformListener

ROBOTD_SOCK = os.environ.get("ROBOTD_SOCK", "/run/user/1000/xuegeros-robotd-shadow.sock")
ROBOTD_PARAMS = "/home/chuyun/robotd-shadow-params.yaml"


def robotd_token() -> str:
    token = os.environ.get("ROBOTD_TOKEN")
    if token:
        return token
    try:
        with open(ROBOTD_PARAMS, "r", errors="replace") as handle:
            for line in handle:
                if "console_token" in line:
                    return line.split(":", 1)[1].strip().strip("'\"")
    except OSError:
        pass
    return ""


def recv_exact(sock, size):
    buf = bytearray()
    while len(buf) < size:
        chunk = sock.recv(size - len(buf))
        if not chunk:
            raise RuntimeError("closed")
        buf.extend(chunk)
    return bytes(buf)


def read_frame(sock):
    first, second = recv_exact(sock, 2)
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", recv_exact(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", recv_exact(sock, 8))[0]
    mask = recv_exact(sock, 4) if masked else b""
    payload = recv_exact(sock, length) if length else b""
    if masked:
        payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    return opcode, payload


def probe_robotd(window=3.0):
    if not os.path.exists(ROBOTD_SOCK):
        return {"ok": False, "error": "socket_missing"}
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect(ROBOTD_SOCK)
        token = robotd_token()
        key = base64.b64encode(os.urandom(16)).decode()
        path = f"/ws?token={token}" if token else "/ws"
        sock.sendall((
            f"GET {path} HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n"
            f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        ).encode())
        head = bytearray()
        while b"\r\n\r\n" not in head:
            chunk = sock.recv(4096)
            if not chunk:
                return {"ok": False, "error": "handshake_closed"}
            head.extend(chunk)
        status = bytes(head).split(b"\r\n", 1)[0].decode("latin-1")
        if "101" not in status:
            return {"ok": False, "error": status[:80]}
        counts, first, last = {}, {}, {}
        deadline = time.monotonic() + window
        sock.settimeout(0.5)
        while time.monotonic() < deadline:
            try:
                opcode, payload = read_frame(sock)
            except socket.timeout:
                continue
            except RuntimeError:
                break
            if opcode == 0x8:
                break
            if opcode not in (0x1, 0x2):
                continue
            now = time.monotonic()
            try:
                kind = json.loads(payload.decode("utf-8")).get("t", "?")
            except ValueError:
                continue
            counts[kind] = counts.get(kind, 0) + 1
            first.setdefault(kind, now)
            last[kind] = now
        sock.close()
        rates = {}
        for kind, count in counts.items():
            span = last[kind] - first[kind]
            rates[kind] = round((count - 1) / span, 2) if count > 1 and span > 0 else None
        return {"ok": counts.get("state", 0) > 0, "counts": counts, "rates_hz": rates}
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": f"{type(exc).__name__}: {str(exc)[:80]}"}


class Health(Node):
    def __init__(self):
        super().__init__("xuegeros_sim_health")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.stats = {}
        self.map_info = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(LaserScan, "/scan", lambda m: self.hit("scan", m.header.stamp), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", lambda m: self.hit("odom", m.header.stamp), qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu", lambda m: self.hit("imu", m.header.stamp), qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)

    def hit(self, name, stamp):
        now = time.monotonic()
        ns = Time.from_msg(stamp).nanoseconds
        entry = self.stats.get(name)
        if entry is None:
            self.stats[name] = {"count": 1, "first": ns, "last": ns}
        else:
            entry["count"] += 1
            entry["last"] = ns
        entry = self.stats[name]
        entry["arrival_last"] = now

    def on_map(self, msg):
        self.map_info = {
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution": round(float(msg.info.resolution), 3),
            "known_cells": sum(1 for c in msg.data if c >= 0),
            "occupied_cells": sum(1 for c in msg.data if c > 50),
        }
        self.hit("map", msg.header.stamp)

    def rate(self, name):
        entry = self.stats.get(name)
        if not entry or entry["count"] < 2:
            return None
        span = (entry["last"] - entry["first"]) / 1e9
        return round((entry["count"] - 1) / span, 2) if span > 0 else None

    def publishers(self, topic):
        try:
            return len(self.get_publishers_info_by_topic(topic))
        except Exception:  # noqa: BLE001
            return None

    def lookup(self, target, source):
        try:
            ts = self.tf_buffer.lookup_transform(
                target, source, Time(), timeout=Duration(seconds=0.5))
            return {
                "ok": True,
                "x": round(ts.transform.translation.x, 3),
                "y": round(ts.transform.translation.y, 3),
                "yaw_deg": round(
                    2.0 * __import__("math").degrees(ts.transform.rotation.z), 2),
            }
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": f"{type(exc).__name__}: {str(exc)[:90]}"}


def main():
    window = float(os.environ.get("HEALTH_WINDOW", "8"))
    rclpy.init()
    node = Health()
    end = time.monotonic() + window
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    result = {
        "window_s": window,
        "domain_id": int(os.environ.get("ROS_DOMAIN_ID", "0")),
        "rates_hz": {name: node.rate(name) for name in ("scan", "odom", "imu", "map")},
        "publishers": {topic: node.publishers(topic) for topic in
                       ("/scan", "/odom", "/imu", "/map", "/cmd_vel", "/cmd_vel_safe_raw", "/cmd_vel_autonomy")},
        "tf": {
            "odom_to_base_footprint": node.lookup("odom", "base_footprint"),
            "map_to_base_link": node.lookup("map", "base_link"),
        },
        "map": node.map_info,
        "robotd": probe_robotd(),
    }
    checks = {
        "scan_rate_ok": (result["rates_hz"]["scan"] or 0) >= 2.0,
        "odom_rate_ok": (result["rates_hz"]["odom"] or 0) >= 10.0,
        "imu_rate_ok": (result["rates_hz"]["imu"] or 0) >= 3.0,
        "map_single_publisher": result["publishers"]["/map"] == 1,
        "tf_chain_ok": bool(result["tf"]["odom_to_base_footprint"]["ok"]
                            and result["tf"]["map_to_base_link"]["ok"]),
        "robotd_state_50hz": (result["robotd"].get("rates_hz", {}).get("state") or 0) >= 45.0,
    }
    result["checks"] = checks
    result["verdict"] = "PASS" if all(checks.values()) else "FAIL"
    print("JSON_HEALTH=" + json.dumps(result, separators=(",", ":")))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
