import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("xuegeros_safety"),
        "config",
        "twist_mux.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="twist_mux",
                executable="twist_mux",
                name="twist_mux",
                output="screen",
                parameters=[config],
                remappings=[("/cmd_vel_out", "/cmd_vel_safe_raw")],
            ),
            Node(
                package="xuegeros_safety",
                executable="cmd_vel_watchdog",
                name="xuegeros_cmd_vel_watchdog",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/cmd_vel_safe_raw",
                        "output_topic": "/cmd_vel",
                        "watchdog_timeout_sec": 0.2,
                        "output_rate_hz": 20.0,
                        "max_linear": 0.1,
                        "max_angular": 0.5,
                        "require_lease": True,
                    }
                ],
            ),
        ]
    )
