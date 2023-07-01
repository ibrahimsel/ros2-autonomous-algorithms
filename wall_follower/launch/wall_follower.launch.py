import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(get_package_share_directory(
        'wall_follower'), 'config', 'wall_follower.yaml')
    node_wall_follower = Node(
        package="wall_follower",
        executable="wall_follower_node",
        name="wall_follower_node",
        output="screen",
        parameters=[config_file]
    )

    # Create LaunchDescription object and add nodes to launch
    ld = LaunchDescription()
    ld.add_action(node_wall_follower)
    return ld
