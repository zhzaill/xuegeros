#!/usr/bin/env python3
"""Watch a coverage run to completion and print one COVERAGE_JSON line.

Run it BEFORE sending {"action":"start"} to /xuegeros/tour, otherwise the first
phases are missed. It stops on DONE/LIMIT/STOPPED and reports exactly the
quantities T6 asks for:

  phase chain (LANE -> TURN -> SHIFT -> ... -> RETURN -> DONE)
  encoder_path_m first/last        (must grow)
  known_cells first/last           (must grow)
  dist_to_home_m last              (must be <= 0.2)
  whether "no_progress" ever appeared
"""

import argparse
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

TERMINAL = {"DONE", "LIMIT", "STOPPED"}


class Watch(Node):
    def __init__(self):
        super().__init__("xuegeros_coverage_watch")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.phases = []
        self.facts = {}
        self.enc_first = None
        self.enc_last = None
        self.dist_last = None
        self.reason = None
        self.no_progress = False
        self.known_first = None
        self.known_last = None
        self.known_max = None
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "/agent/tour_facts", self.on_facts, 10)
        self.create_subscription(String, "/agent/mapping_status", self.on_status, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.cmd_count = 0
        self.cmd_max_linear = 0.0
        self.cmd_nonzero = 0
        self.cmd_last = None
        self.status_reasons = {}
        self.blocked_ticks = 0

    def on_cmd(self, msg):
        self.cmd_count += 1
        lin, ang = abs(msg.linear.x), abs(msg.angular.z)
        self.cmd_max_linear = max(self.cmd_max_linear, lin, ang)
        if lin > 1e-6 or ang > 1e-6:
            self.cmd_nonzero += 1
        self.cmd_last = [round(msg.linear.x, 4), round(msg.angular.z, 4)]

    def on_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        reason = payload.get("reason")
        if isinstance(reason, list):
            reason = ",".join(str(r) for r in reason)
        if reason:
            self.status_reasons[reason] = self.status_reasons.get(reason, 0) + 1
        if payload.get("state") == "BLOCKED":
            self.blocked_ticks += 1

    def on_map(self, msg):
        known = sum(1 for c in msg.data if c >= 0)
        if self.known_first is None:
            self.known_first = known
        self.known_last = known
        if self.known_max is None or known > self.known_max:
            self.known_max = known

    def on_facts(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        facts = payload.get("facts") if isinstance(payload.get("facts"), dict) else payload
        phase = facts.get("phase")
        if not phase:
            return
        if not self.phases or self.phases[-1] != phase:
            self.phases.append(phase)
        self.facts = facts
        enc = facts.get("encoder_path_m")
        if enc is not None:
            if self.enc_first is None:
                self.enc_first = enc
            self.enc_last = enc
        if facts.get("dist_to_home_m") is not None:
            self.dist_last = facts.get("dist_to_home_m")
        if facts.get("reason"):
            self.reason = facts.get("reason")
        blob = json.dumps(facts).lower()
        if "no_progress" in blob:
            self.no_progress = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-window", type=float, default=420.0)
    parser.add_argument("--settle", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = Watch()
    start = time.monotonic()
    deadline = start + args.max_window
    terminal_since = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.phases and node.phases[-1] in TERMINAL:
            terminal_since = terminal_since or time.monotonic()
            if time.monotonic() - terminal_since >= args.settle:
                break

    result = {
        "phases": node.phases,
        "phase_count": len(node.phases),
        "final_phase": node.phases[-1] if node.phases else None,
        "finish_reason": node.reason,
        "saw_lane_turn_shift": node.phases[:3] == ["LANE", "TURN", "SHIFT"],
        "saw_return": "RETURN" in node.phases,
        "reached_done": bool(node.phases) and node.phases[-1] == "DONE",
        "no_progress_seen": node.no_progress,
        "encoder_path_first": node.enc_first,
        "encoder_path_last": node.enc_last,
        "encoder_path_grew": (node.enc_first is not None and node.enc_last is not None
                              and node.enc_last > node.enc_first),
        "dist_to_home_m": node.dist_last,
        "dist_to_home_ok": (node.dist_last is not None and node.dist_last <= 0.2),
        "known_cells_first": node.known_first,
        "known_cells_last": node.known_last,
        "known_cells_max": node.known_max,
        "known_cells_grew": (node.known_first is not None and node.known_max is not None
                             and node.known_max > node.known_first),
        "elapsed_s": round(time.monotonic() - start, 1),
        "last_facts_phase": node.facts.get("phase"),
        "last_facts_pose": node.facts.get("pose"),
        "cmd_vel": {
            "messages": node.cmd_count,
            "nonzero": node.cmd_nonzero,
            "max_magnitude": round(node.cmd_max_linear, 4),
            "last": node.cmd_last,
        },
        "status": {
            "blocked_ticks": node.blocked_ticks,
            "top_reasons": sorted(node.status_reasons.items(), key=lambda kv: -kv[1])[:5],
        },
    }
    result["t6_pass"] = bool(
        result["reached_done"] and result["saw_lane_turn_shift"] and result["saw_return"]
        and result["encoder_path_grew"] and result["known_cells_grew"]
        and result["dist_to_home_ok"] and not result["no_progress_seen"])
    print("COVERAGE_JSON=" + json.dumps(result, separators=(",", ":")))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
