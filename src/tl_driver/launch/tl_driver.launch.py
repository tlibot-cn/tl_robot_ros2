import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    arm_type = LaunchConfiguration('arm_type').perform(context)

    pkg_share = get_package_share_directory('tl_driver')

    configs = {
        'tcb605': {
            'config': os.path.join(pkg_share, 'config', 'tl_tcb605_config.yaml')
        },
        'tcb605v': {
            'config': os.path.join(pkg_share, 'config', 'tl_tcb605v_config.yaml')
        },
        'tcb605f': {
            'config': os.path.join(pkg_share, 'config', 'tl_tcb605f_config.yaml')
        },
        'tcb605l': {
            'config': os.path.join(pkg_share, 'config', 'tl_tcb605l_config.yaml')
        },
        'tcb605lv': {
            'config': os.path.join(pkg_share, 'config', 'tl_tcb605lv_config.yaml')
        },
        'tcb610': {
            'config': os.path.join(pkg_share, 'config', 'tl_tcb610_config.yaml')
        },
        'tcb610v': {
            'config': os.path.join(pkg_share, 'config', 'tl_tcb610v_config.yaml')
        },
        'tcb705': {
            'config': os.path.join(pkg_share, 'config', 'tl_tcb705_config.yaml')
        },
        'tcb705v': {
            'config': os.path.join(pkg_share, 'config', 'tl_tcb705v_config.yaml')
        },
        'tcb705f': {
            'config': os.path.join(pkg_share, 'config', 'tl_tcb705f_config.yaml')
        },
        'tcb705l': {
            'config': os.path.join(pkg_share, 'config', 'tl_tcb705l_config.yaml')
        },
        'tcb705lv': {
            'config': os.path.join(pkg_share, 'config', 'tl_tcb705lv_config.yaml')
        },
        'tcb710': {
            'config': os.path.join(pkg_share, 'config', 'tl_tcb710_config.yaml')
        },
        'tcb710v': {
            'config': os.path.join(pkg_share, 'config', 'tl_tcb710v_config.yaml')
        },
    }

    if arm_type not in configs:
        raise RuntimeError(
            f'Unsupported arm_type: {arm_type}. '
            f'Supported arm_type: {list(configs.keys())}'
        )

    config_path = configs[arm_type]['config']

    if not os.path.exists(config_path):
        raise FileNotFoundError(f'Config file not found: {config_path}')

    tl_driver_node = Node(
        package='tl_driver',
        executable='tl_driver',
        name='tl_driver',
        output='screen',
        parameters=[
            config_path
        ]
    )

    return [
        tl_driver_node
    ]

def generate_launch_description():
    declare_arm_type = DeclareLaunchArgument(
        'arm_type',
        default_value='tcb605',
        description='Arm type',
        choices=[
            'tcb605', 'tcb605v', 'tcb605f', 'tcb605l', 'tcb605lv', 'tcb610', 'tcb610v',
            'tcb705', 'tcb705f', 'tcb705v', 'tcb705l', 'tcb705lv', 'tcb710', 'tcb710v'
        ]
    )

    return LaunchDescription([
        declare_arm_type,
        OpaqueFunction(function=launch_setup)
    ])