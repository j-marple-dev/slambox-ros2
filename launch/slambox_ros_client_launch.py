"""SLAMBOX ros2 launch file

-Author: Haneol Kim
-Contact: hekim@jmarple.ai
"""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("slambox_ros2"), "config", "client.yaml"
    )

    node_list = []

    slambox_ros2_node = Node(
        package="slambox_ros2",
        executable="slambox_ros2-node_client",
        name="slambox_ros2_client_node",
        parameters=[config],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            [
                os.path.join(
                    get_package_share_directory("slambox_ros2"),
                    "config",
                    "slambox_rviz2.rviz",
                )
            ],
        ],
    )

    node_list.append(slambox_ros2_node)

    node_list.append(rviz2_node)

    return LaunchDescription(node_list)
