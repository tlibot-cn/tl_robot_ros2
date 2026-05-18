#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution, PathJoinSubstitution

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('tl_vision')

    # 配置文件路径
    control_config = PathJoinSubstitution([pkg_share, 'config', 'control_node.yaml'])

    control_node = Node(
        package='tl_vision',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[control_config]
    )

    return LaunchDescription([
        control_node,
    ])