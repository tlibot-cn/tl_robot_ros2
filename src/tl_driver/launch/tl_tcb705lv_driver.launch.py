import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("tl_driver")
    config_path = os.path.join(pkg_share, "config", "tl_tcb705lv_config.yaml")
    arm_ip = LaunchConfiguration("arm_ip")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_ip",
                default_value="192.168.1.13",
                description="机械臂控制器IP，可覆盖config中的默认值",
            ),
            Node(
                package="tl_driver",
                executable="tl_driver",
                name="tl_driver",
                output="screen",
                parameters=[config_path, {"arm_ip": arm_ip}],
            ),
        ]
    )
