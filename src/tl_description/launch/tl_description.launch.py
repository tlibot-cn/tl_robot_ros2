import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    arm_type = LaunchConfiguration("arm_type").perform(context)
    use_sim = LaunchConfiguration("use_sim").perform(context)

    pkg_share = get_package_share_directory("tl_description")

    configs = {
        "tcb605": {
            "urdf": os.path.join(pkg_share, "urdf", "tcb605.urdf"),
            "rviz": os.path.join(pkg_share, "rviz", "tcb605.rviz"),
        },
        "tcb605v": {
            "urdf": os.path.join(pkg_share, "urdf", "tcb605v.urdf"),
            "rviz": os.path.join(pkg_share, "rviz", "tcb605v.rviz"),
        },
        "tcb605f": {
            "urdf": os.path.join(pkg_share, "urdf", "tcb605f.urdf"),
            "rviz": os.path.join(pkg_share, "rviz", "tcb605f.rviz"),
        },
        "tcb605l": {
            "urdf": os.path.join(pkg_share, "urdf", "tcb605l.urdf"),
            "rviz": os.path.join(pkg_share, "rviz", "tcb605l.rviz"),
        },
        "tcb605lv": {
            "urdf": os.path.join(pkg_share, "urdf", "tcb605lv.urdf"),
            "rviz": os.path.join(pkg_share, "rviz", "tcb605lv.rviz"),
        },
        "tcb610": {
            "urdf": os.path.join(pkg_share, "urdf", "tcb610.urdf"),
            "rviz": os.path.join(pkg_share, "rviz", "tcb610.rviz"),
        },
        "tcb610v": {
            "urdf": os.path.join(pkg_share, "urdf", "tcb610v.urdf"),
            "rviz": os.path.join(pkg_share, "rviz", "tcb610v.rviz"),
        },
        "tcb705": {
            "urdf": os.path.join(pkg_share, "urdf", "tcb705.urdf"),
            "rviz": os.path.join(pkg_share, "rviz", "tcb705.rviz"),
        },
        "tcb705v": {
            "urdf": os.path.join(pkg_share, "urdf", "tcb705v.urdf"),
            "rviz": os.path.join(pkg_share, "rviz", "tcb705v.rviz"),
        },
        "tcb705f": {
            "urdf": os.path.join(pkg_share, "urdf", "tcb705f.urdf"),
            "rviz": os.path.join(pkg_share, "rviz", "tcb705f.rviz"),
        },
        "tcb705l": {
            "urdf": os.path.join(pkg_share, "urdf", "tcb705l.urdf"),
            "rviz": os.path.join(pkg_share, "rviz", "tcb705l.rviz"),
        },
        "tcb705lv": {
            "urdf": os.path.join(pkg_share, "urdf", "tcb705lv.urdf"),
            "rviz": os.path.join(pkg_share, "rviz", "tcb705lv.rviz"),
        },
        "tcb710": {
            "urdf": os.path.join(pkg_share, "urdf", "tcb710.urdf"),
            "rviz": os.path.join(pkg_share, "rviz", "tcb710.rviz"),
        },
        "tcb710v": {
            "urdf": os.path.join(pkg_share, "urdf", "tcb710v.urdf"),
            "rviz": os.path.join(pkg_share, "rviz", "tcb710v.rviz"),
        },
    }

    if arm_type not in configs:
        raise RuntimeError(
            f"Unsupported arm_type: {arm_type}. " f"Supported arm_type: {list(configs.keys())}"
        )

    urdf_file = configs[arm_type]["urdf"]
    rviz_file = configs[arm_type]["rviz"]

    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    if not os.path.exists(rviz_file):
        raise FileNotFoundError(f"RViz file not found: {rviz_file}")

    with open(urdf_file, "r") as f:
        robot_description_content = f.read()

    nodes = []

    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description_content}],
            output="screen",
        )
    )

    nodes.append(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(use_sim),  # ✅ 当 use_sim 为 'true' 时启动
        )
    )

    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_file, "--ros-args", "--log-level", "error"],
        )
    )

    return nodes


def generate_launch_description():
    declare_arm_type = DeclareLaunchArgument(
        "arm_type",
        default_value="tcb605",
        description="Arm type",
        choices=[
            "tcb605",
            "tcb605v",
            "tcb605f",
            "tcb605l",
            "tcb605lv",
            "tcb610",
            "tcb610v",
            "tcb705",
            "tcb705f",
            "tcb705v",
            "tcb705l",
            "tcb705lv",
            "tcb710",
            "tcb710v",
        ],
    )

    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Use joint_state_publisher_gui if true",
        choices=["true", "false"],
    )

    return LaunchDescription(
        [declare_arm_type, declare_use_sim, OpaqueFunction(function=launch_setup)]
    )
