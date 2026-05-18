from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import LogInfo
from launch.substitutions import TextSubstitution, PathJoinSubstitution


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('tl_vision')
    
    # 配置文件路径
    calib_config = PathJoinSubstitution([pkg_share, 'config', 'calib_node.yaml'])
    
    # 标定节点
    calib_node = Node(
        package='tl_vision',
        executable='calib_node',
        name='calib_node',
        output='screen',
        parameters=[calib_config]
    )
    
    return LaunchDescription([
	    calib_node,
    ])
