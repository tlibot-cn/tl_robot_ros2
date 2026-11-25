#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_vision = FindPackageShare('tl_vision')

    # 配置文件路径
    control_config = PathJoinSubstitution([pkg_vision, 'config', 'control_node.yaml'])
    gripper_config = PathJoinSubstitution([pkg_vision, 'config', 'gripper.yaml'])

    control_node = Node(
        package='tl_vision',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[control_config, gripper_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_type',
            default_value='tcb710',
            description='机械臂型号，例如 tcb605、tcb710'
        ),
        control_node,
    ])
