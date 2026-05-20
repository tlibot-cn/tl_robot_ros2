#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_vision = FindPackageShare('tl_vision')
    pkg_driver = FindPackageShare('tl_driver')

    arm_type = LaunchConfiguration('arm_type')

    # 配置文件路径
    control_config = PathJoinSubstitution([pkg_vision, 'config', 'control_node.yaml'])

    # tl_driver 节点（control_node 依赖其服务与话题）
    tl_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_driver, 'launch', 'tl_driver.launch.py'])
        ),
        launch_arguments={
            'arm_type': arm_type,
        }.items()
    )

    control_node = Node(
        package='tl_vision',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[control_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_type',
            default_value='tcb710',
            description='机械臂型号，例如 tcb605、tcb710'
        ),
        tl_driver_launch,
        control_node,
    ])
