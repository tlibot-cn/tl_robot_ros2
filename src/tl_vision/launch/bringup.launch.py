from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包路径
    pkg_vision = FindPackageShare('tl_vision')
    pkg_driver = FindPackageShare('tl_driver')

    arm_type = LaunchConfiguration('arm_type')

    # 配置文件路径
    camera_config = PathJoinSubstitution([pkg_vision, 'config', 'camera_params.yaml'])
    yolo_config = PathJoinSubstitution([pkg_vision, 'config', 'yolo_node.yaml'])
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

    # RealSense 相机节点
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        output='screen',
        parameters=[camera_config]
    )

    # YOLO 检测节点
    yolo_node = Node(
        package='tl_vision',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[yolo_config]
    )

    # 机械臂控制节点
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
        realsense_node,
        yolo_node,
        control_node,
    ])
