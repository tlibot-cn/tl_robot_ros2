from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import LogInfo
from launch.substitutions import TextSubstitution, PathJoinSubstitution


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('tl_vision')
    
    # 配置文件路径
    camera_config = PathJoinSubstitution([pkg_share, 'config', 'camera_params.yaml'])
    yolo_config = PathJoinSubstitution([pkg_share, 'config', 'yolo_node.yaml'])
    control_config = PathJoinSubstitution([pkg_share, 'config', 'control_node.yaml'])
    
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

        # YOLO 检测节点
    control_node = Node(
        package='tl_vision',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[control_config]
    )
    
    return LaunchDescription([
        realsense_node,
        yolo_node,
        control_node,
    ])
