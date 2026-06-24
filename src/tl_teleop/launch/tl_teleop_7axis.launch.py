import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("tl_teleop")
    config_path = os.path.join(pkg_share, "config", "tl_teleop_7axis_config.yaml")

    return LaunchDescription(
        [
            Node(
                package="tl_teleop",
                executable="tl_teleop",
                name="tl_teleop_node",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
