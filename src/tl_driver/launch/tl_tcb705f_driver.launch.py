import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("tl_driver")
    config_path = os.path.join(pkg_share, "config", "tl_tcb705f_config.yaml")

    return LaunchDescription(
        [
            Node(
                package="tl_driver",
                executable="tl_driver",
                name="tl_driver",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
