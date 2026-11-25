from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_vision = FindPackageShare('tl_vision')
    pkg_driver = FindPackageShare('tl_driver')

    arm_type = LaunchConfiguration('arm_type')

    # 配置文件路径
    calib_config = PathJoinSubstitution([pkg_vision, 'config', 'calib_node.yaml'])

    # tl_driver 节点（calib_node online 模式依赖其服务与 /tcp_pose 话题）
    tl_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_driver, 'launch', 'tl_driver.launch.py'])
        ),
        launch_arguments={
            'arm_type': arm_type,
        }.items()
    )

    # 标定节点
    calib_node = Node(
        package='tl_vision',
        executable='calib_node',
        name='calib_node',
        output='screen',
        parameters=[calib_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_type',
            default_value='tcb710',
            description='机械臂型号，例如 tcb605、tcb710'
        ),
        tl_driver_launch,
        calib_node,
    ])
