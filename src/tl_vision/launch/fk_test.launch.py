from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    arm_type = LaunchConfiguration('arm_type')
    use_sim = LaunchConfiguration('use_sim')

    base_link = LaunchConfiguration('base_link')
    tip_link = LaunchConfiguration('tip_link')
    joint_states_topic = LaunchConfiguration('joint_states_topic')
    publish_rate = LaunchConfiguration('publish_rate')

    tl_description_launch = PathJoinSubstitution([
        FindPackageShare('tl_description'),
        'launch',
        'tl_description.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_type',
            default_value='tcb710',
            description='机械臂型号，例如 tcb605'
        ),

        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description=(
                '是否启动 tl_description 中的 joint_state_publisher_gui。'
                '和 fk_test_node 一起使用时建议 false，避免 /joint_states 冲突。'
            )
        ),

        DeclareLaunchArgument(
            'base_link',
            default_value='link0',
            description='FK base link'
        ),

        DeclareLaunchArgument(
            'tip_link',
            default_value='link7',
            description='FK tip link'
        ),

        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/joint_states',
            description='fk_test_node 发布 JointState 的话题'
        ),

        DeclareLaunchArgument(
            'publish_rate',
            default_value='30.0',
            description='fk_test_node 发布 /joint_states 的频率'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tl_description_launch),
            launch_arguments={
                'arm_type': arm_type,
                'use_sim': use_sim,
            }.items()
        ),

        Node(
            package='tl_vision',
            executable='fk_test_node',
            name='fk_test_node',
            output='screen',
            parameters=[
                {
                    'base_link': base_link,
                    'tip_link': tip_link,
                    'joint_states_topic': joint_states_topic,
                    'publish_rate': publish_rate,
                }
            ]
        ),
    ])