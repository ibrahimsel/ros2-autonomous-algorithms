import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("pure_pursuit"), "config", "pure_pursuit.yaml"
    )

    node_pure_pursuit = Node(
        package="pure_pursuit",
        executable="pure_pursuit",
        name="pure_pursuit",
        output="screen",
        # parameters=[config_file]
    )

    marker_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("pure_pursuit"),
                    "launch",
                    "marker_publisher.launch.py",
                )
            ]
        )
    )

    node_safety = Node(
        package="autonomous_emergency_break",
        executable="safety_node",
        name="safety_node",
        output="screen",
    )

    # Create LaunchDescription object and add nodes to launch
    ld = LaunchDescription()
    ld.add_action(node_pure_pursuit)
    ld.add_action(marker_publisher)
    # ld.add_action(node_safety)
    return ld
