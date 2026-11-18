#!/usr/bin/env python3
"""Send one command to /xuegeros/tour (start/stop) for the shadow controller.

Publishes repeatedly for a few seconds because a single publish can land before
the controller has discovered the publisher.

    python3 tour_cmd.py start [source]
    python3 tour_cmd.py stop
"""

import json
import sys
import time

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String


def main():
    action = sys.argv[1] if len(sys.argv) > 1 else "start"
    source = sys.argv[2] if len(sys.argv) > 2 else "ops"
    rclpy.init()
    node = rclpy.create_node("xuegeros_tour_cmd")
    node.set_parameters([Parameter("use_sim_time", value=True)])
    pub = node.create_publisher(String, "/xuegeros/tour", 10)
    payload = json.dumps({"action": action, "source": source,
                          "request_id": f"{action}-{int(time.time())}"})
    deadline = time.time() + 5.0
    sent = 0
    while time.time() < deadline:
        pub.publish(String(data=payload))
        rclpy.spin_once(node, timeout_sec=0.2)
        sent += 1
        time.sleep(0.2)
    print(f"TOUR_CMD sent={sent} payload={payload}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
