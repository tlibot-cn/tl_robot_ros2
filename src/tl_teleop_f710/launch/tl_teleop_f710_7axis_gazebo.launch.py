"""天链机械臂 7 轴 + F710 手柄遥操作 — 通用 Gazebo 仿真启动文件 (C++)。

IK 由仿真桥接节点内部使用 KDL 本地求解，无需 MoveIt2。

用法：
  ros2 launch tl_teleop_f710 tl_teleop_f710_7axis_gazebo.launch.py arm_type:=tcb710
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    teleop_share = get_package_share_directory("tl_teleop_f710")
    gazebo_share = get_package_share_directory("tl_gazebo")
    arm_type = LaunchConfiguration("arm_type")
    joy_dev = LaunchConfiguration("joy_dev")
    config_path = os.path.join(teleop_share, "config", "tl_teleop_f710_7axis_sim.yaml")
    gazebo_launch = PathJoinSubstitution(
        [gazebo_share, "launch", "gazebo_7axis_f710_sim.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_type", default_value="tcb710"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch)),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="joy",
                        executable="joy_node",
                        name="joy_node",
                        parameters=[{"dev": joy_dev, "deadzone": 0.1, "autorepeat_rate": 30.0}],
                    )
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="tl_teleop_f710",
                        executable="tl_teleop_f710_node",
                        name="tl_teleop_f710_node",
                        output="screen",
                        parameters=[config_path, {"arm_type": arm_type}],
                    )
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="tl_teleop_f710",
                        executable="tl_teleop_f710_sim_bridge",
                        name="tl_teleop_f710_sim_bridge",
                        output="screen",
                        parameters=[{"arm_type": arm_type}],
                    )
                ],
            ),
        ]
    )
