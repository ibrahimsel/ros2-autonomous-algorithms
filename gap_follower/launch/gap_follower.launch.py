import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    node_gap_follower = Node(
        package="gap_follower",
        executable="gap_follower_node",
        name="gap_follower",
        output="screen",
    )
    node_safety = Node(
        package="autonomous_emergency_break",
        executable="safety_node",
        name="safety_node",
        output="screen",
    )

    # Create LaunchDescription object and add nodes to launch
    ld = LaunchDescription()
    ld.add_action(node_gap_follower)
    # ld.add_action(node_safety)
    return ld
