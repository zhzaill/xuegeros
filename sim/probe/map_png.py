#!/usr/bin/env python3
"""Render the live shadow /map (plus scan and robot pose) to a PNG.

The VM has no display (no Xvfb, gzclient cannot run), so "seeing" the
simulation means drawing what the robot actually senses: the occupancy grid
gmapping built from /scan, the scan itself, and the pose from odom->map.

    python3 map_png.py --out /home/chuyun/xuegeros_ws/log/sim/map_shadow.png
"""

import argparse
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from PIL import Image, ImageDraw
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class Snapshot(Node):
    def __init__(self):
        super().__init__("xuegeros_map_png")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.map_msg = None
        self.scan = None
        self.pose = None
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)

    def on_map(self, msg):
        self.map_msg = msg

    def on_scan(self, msg):
        self.scan = msg

    def on_odom(self, msg):
        self.pose = msg.pose.pose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--wait", type=float, default=15.0)
    parser.add_argument("--scale", type=int, default=2)
    # The scan overlay is only trustworthy when the map->base TF lookup
    # succeeds; mixing odom position with map yaw painted the beams outside the
    # room (measured 2026-11-15), so it is opt-in.
    parser.add_argument("--scan", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = Snapshot()
    deadline = time.monotonic() + args.wait
    while time.monotonic() < deadline and (node.map_msg is None or node.scan is None):
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.map_msg is None:
        print("MAP_PNG error=no_map")
        return
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)

    msg = node.map_msg
    width, height = int(msg.info.width), int(msg.info.height)
    resolution = float(msg.info.resolution)
    origin = msg.info.origin.position
    data = np.array(msg.data, dtype=np.int16).reshape((height, width))

    # Grid -> image: unknown mid grey, free light, occupied dark.
    image = np.full((height, width, 3), 128, dtype=np.uint8)
    image[data == 0] = (245, 245, 245)
    image[data > 50] = (30, 30, 30)
    image[(data > 0) & (data <= 50)] = (150, 150, 150)

    # gmapping's grid has its origin at the bottom-left in map coordinates; the
    # image is stored top-down, hence the flip.
    image = np.flipud(image)

    # Crop to the explored area so the room fills the frame instead of sitting
    # as a small island inside 19 m of unknown cells.
    known_rows, known_cols = np.where(data >= 0)
    pad = 12
    if known_rows.size and known_cols.size:
        r0 = max(0, int(known_rows.min()) - pad)
        r1 = min(height - 1, int(known_rows.max()) + pad)
        c0 = max(0, int(known_cols.min()) - pad)
        c1 = min(width - 1, int(known_cols.max()) + pad)
    else:
        r0, r1, c0, c1 = 0, height - 1, 0, width - 1
    # image rows are flipped: image row = height-1-row
    img_r0 = height - 1 - r1
    img_r1 = height - 1 - r0
    image = image[img_r0:img_r1 + 1, c0:c1 + 1]

    def to_pixel(x, y):
        col = (x - origin.x) / resolution - c0
        row = (height - 1 - (y - origin.y) / resolution) - img_r0
        return int(round(col)), int(round(row))

    # Robot pose must come from the SAME frame as the grid (map). Odom-only
    # fallbacks land outside the room because map and odom differ by the
    # map->odom correction, so a failed lookup simply draws no robot.
    tf_ok = False
    px = py = yaw = None
    if node.pose is not None:
        try:
            ts = node.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            px = ts.transform.translation.x
            py = ts.transform.translation.y
            tq = ts.transform.rotation
            yaw = math.atan2(2.0 * (tq.w * tq.z + tq.x * tq.y),
                             1.0 - 2.0 * (tq.y * tq.y + tq.z * tq.z))
            tf_ok = True
        except Exception:  # noqa: BLE001
            px = py = yaw = None

    img = Image.fromarray(image, mode="RGB")
    if args.scale != 1:
        img = img.resize((img.width * args.scale, img.height * args.scale), Image.NEAREST)
    draw = ImageDraw.Draw(img)

    scan_points = 0
    if args.scan and node.scan is not None and px is not None and tf_ok:
        angle = node.scan.angle_min
        if px is not None:
            for r in node.scan.ranges:
                if math.isfinite(r) and node.scan.range_min < r < node.scan.range_max:
                    wx = px + r * math.cos(yaw + angle)
                    wy = py + r * math.sin(yaw + angle)
                    col, row = to_pixel(wx, wy)
                    col *= args.scale
                    row *= args.scale
                    if 0 <= col < img.width and 0 <= row < img.height:
                        draw.rectangle([col - 1, row - 1, col + 1, row + 1],
                                       fill=(220, 40, 40))
                        scan_points += 1
                angle += node.scan.angle_increment

    if px is not None:
        col, row = to_pixel(px, py)
        col *= args.scale
        row *= args.scale
        r = max(4, 3 * args.scale)
        draw.ellipse([col - r, row - r, col + r, row + r], fill=(30, 90, 230))
        tip = (col + int(2.2 * r * math.cos(-yaw)),
               row + int(2.2 * r * math.sin(-yaw)))
        draw.line([col, row, tip[0], tip[1]], fill=(30, 90, 230), width=2)

    draw.text((6, 6), "xuegeros shadow  domain 42  %dx%d @ %.2fm  known=%d  occupied=%d" %
              (width, height, resolution, int((data >= 0).sum()), int((data > 50).sum())),
              fill=(20, 20, 20))
    img.save(args.out)

    known = int((data >= 0).sum())
    occupied = int((data > 50).sum())
    print("MAP_PNG=" + json.dumps({
        "path": args.out,
        "width": width, "height": height, "scale": args.scale,
        "resolution": resolution,
        "known_cells": known, "occupied_cells": occupied,
        "scan_points_drawn": scan_points,
        "robot_xy": None if px is None else [round(px, 3), round(py, 3)],
        "robot_yaw_deg": None if yaw is None else round(math.degrees(yaw), 1),
    }, separators=(",", ":")))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
