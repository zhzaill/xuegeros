from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("physical_enable", default_value="false"),
            DeclareLaunchArgument("confirm_physical", default_value="false"),
            Node(
                package="agent_control",
                executable="mapping_controller",
                name="xuegeros_agent_mapping_controller",
                output="screen",
                parameters=[
                    {
                        "physical_enable": LaunchConfiguration("physical_enable"),
                        "confirm_physical": LaunchConfiguration("confirm_physical"),
                    }
                ],
            ),
        ]
    )
