import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("tl_description")

    urdf_file = os.path.join(pkg_share, "urdf", "tcb610v.urdf")
    rviz_file = os.path.join(pkg_share, "rviz", "tcb610v.rviz")

    with open(urdf_file, "r") as f:
        robot_description_content = f.read()

    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Use joint_state_publisher_gui if true",
        choices=["true", "false"],
    )

    return LaunchDescription(
        [
            declare_use_sim,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description_content}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_sim")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_file, "--ros-args", "--log-level", "error"],
            ),
        ]
    )
