import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def supported_arm_types(pkg_share):
    """从 config 目录推导支持的臂型，避免与配置文件清单脱节。"""
    pattern = os.path.join(pkg_share, "config", "tl_*_config.yaml")
    prefix, suffix = "tl_", "_config.yaml"
    return [os.path.basename(p)[len(prefix) : -len(suffix)] for p in sorted(glob.glob(pattern))]


def launch_setup(context, *args, **kwargs):
    arm_type = LaunchConfiguration("arm_type").perform(context)

    pkg_share = get_package_share_directory("tl_driver")

    arm_types = supported_arm_types(pkg_share)
    if arm_type not in arm_types:
        raise RuntimeError(f"Unsupported arm_type: {arm_type}. " f"Supported arm_type: {arm_types}")

    config_path = os.path.join(pkg_share, "config", f"tl_{arm_type}_config.yaml")
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Config file not found: {config_path}")

    # 必须用多线程容器：monitor 组的电流/温度/电压查询单次可达数百 ms，
    # 单线程容器会让它阻塞运动服务与 ServoJ 订阅。
    container = ComposableNodeContainer(
        name="tl_driver_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="tl_driver",
                plugin="tl_driver::DriverNode",
                name="tl_driver",
                parameters=[config_path, {"arm_ip": LaunchConfiguration("arm_ip")}],
            )
        ],
        output="screen",
    )

    return [container]


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

    declare_arm_ip = DeclareLaunchArgument(
        "arm_ip",
        default_value="192.168.1.13",
        description="机械臂控制器IP，可覆盖config中的默认值",
    )

    return LaunchDescription(
        [declare_arm_type, declare_arm_ip, OpaqueFunction(function=launch_setup)]
    )
