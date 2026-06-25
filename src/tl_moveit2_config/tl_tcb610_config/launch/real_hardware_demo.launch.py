import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "tl_tcb610", package_name="tl_tcb610_config"
    ).to_moveit_configs()

    pkg_share = get_package_share_directory("tl_tcb610_config")
    xacro_path = os.path.join(pkg_share, "config", "tl_tcb610.urdf.xacro")
    initial_positions_path = os.path.join(pkg_share, "config", "initial_positions.yaml")

    # Robot description with real hardware ros2_control plugin
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    xacro_path,
                    " use_real_hardware:=true",
                    " initial_positions_file:=",
                    initial_positions_path,
                ]
            ),
            value_type=str,
        )
    }

    use_sim_time_param = {"use_sim_time": False}

    # Controller manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(pkg_share, "config", "ros2_controllers.yaml"),
        ],
        output="screen",
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Trajectory controller spawner
    tcb_group_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "tcb_group_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # MoveGroup
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            use_sim_time_param,
        ],
    )

    # RViz
    rviz_config = os.path.join(pkg_share, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            use_sim_time_param,
        ],
    )

    return LaunchDescription(
        [
            controller_manager_node,
            joint_state_broadcaster_spawner,
            tcb_group_controller_spawner,
            robot_state_publisher_node,
            move_group_node,
            rviz_node,
        ]
    )
