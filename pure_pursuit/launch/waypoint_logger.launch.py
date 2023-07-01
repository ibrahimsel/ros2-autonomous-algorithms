import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("pure_pursuit"), "config", "waypoint_logger.yaml"
    )

    node_waypoint_logger = Node(
        package="pure_pursuit",
        executable="waypoint_logger",
        name="waypoint_logger",
        output="screen",
        parameters=[config_file],
    )

    # Create LaunchDescription object and add nodes to launch
    ld = LaunchDescription()
    ld.add_action(node_waypoint_logger)
    return ld
