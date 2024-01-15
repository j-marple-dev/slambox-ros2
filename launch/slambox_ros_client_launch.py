"""SLAMBOX ros2 launch file

-Author: Haneol Kim
-Contact: hekim@jmarple.ai
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('slambox_ros2'),
        'config',
        'client.yaml'
    )
    return LaunchDescription([
        Node(
            package="slambox_ros2",
            executable="slambox_ros2-node_client",
            name="slambox_ros2_client_node",
            parameters=[config]
        )
    ])
