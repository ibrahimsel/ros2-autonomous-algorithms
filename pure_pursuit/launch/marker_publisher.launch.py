from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node_marker_publisher = Node(
        package="pure_pursuit",
        executable="marker_publisher",
        name="marker_publisher",
        output="screen",
        parameters=[
            # {"wp_file_path": ""},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(node_marker_publisher)
    return ld
