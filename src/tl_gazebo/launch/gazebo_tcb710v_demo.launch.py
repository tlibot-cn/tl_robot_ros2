import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():
    package_name = "tl_gazebo"
    robot_name_in_model = "tl_tcb710v"

    # ---------------- Gazebo resource path setup ----------------
    tl_description_share = get_package_share_directory("tl_description")

    gazebo_model_root = os.path.expanduser("~/.gazebo/tl_robot_models")
    gazebo_tl_description_model = os.path.join(gazebo_model_root, "tl_description")
    gazebo_tl_description_meshes = os.path.join(gazebo_tl_description_model, "meshes")

    os.makedirs(gazebo_tl_description_model, exist_ok=True)

    real_meshes_dir = os.path.join(tl_description_share, "meshes")

    if os.path.lexists(gazebo_tl_description_meshes):
        if os.path.islink(gazebo_tl_description_meshes):
            if os.path.realpath(gazebo_tl_description_meshes) != os.path.realpath(real_meshes_dir):
                os.unlink(gazebo_tl_description_meshes)
        else:
            raise RuntimeError(
                f"{gazebo_tl_description_meshes} already exists and is not a symlink. "
                f"Please remove it manually."
            )

    if not os.path.lexists(gazebo_tl_description_meshes):
        os.symlink(real_meshes_dir, gazebo_tl_description_meshes)

    model_config_path = os.path.join(gazebo_tl_description_model, "model.config")
    dummy_sdf_path = os.path.join(gazebo_tl_description_model, "dummy.sdf")

    if not os.path.exists(model_config_path):
        with open(model_config_path, "w") as f:
            f.write("""<?xml version="1.0"?>
                        <model>
                        <name>tl_description</name>
                        <version>1.0</version>
                        <sdf version="1.6">dummy.sdf</sdf>
                        <author>
                            <name>tl</name>
                            <email>none@example.com</email>
                        </author>
                        <description>
                            Mesh resource package for TL robot.
                        </description>
                        </model>
                        """)

    if not os.path.exists(dummy_sdf_path):
        with open(dummy_sdf_path, "w") as f:
            f.write("""<?xml version="1.0"?>
                        <sdf version="1.6">
                        <model name="tl_description">
                            <static>true</static>
                            <link name="dummy_link"/>
                        </model>
                        </sdf>
                        """)

    bad_model_path = os.path.dirname(tl_description_share)
    existing_gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")

    # Gazebo Classic 系统模型路径，用于 ground_plane 和 sun
    system_gazebo_models = "/usr/share/gazebo-11/models"

    model_paths = [gazebo_model_root]

    if os.path.exists(system_gazebo_models):
        model_paths.append(system_gazebo_models)

    for p in existing_gazebo_model_path.split(":"):
        if not p:
            continue

        # 过滤可能导致 package://tl_description 解析错误的路径
        if os.path.realpath(p) == os.path.realpath(bad_model_path):
            continue

        if p not in model_paths:
            model_paths.append(p)

    gazebo_model_path_value = ":".join(model_paths)

    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH", value=gazebo_model_path_value
    )

    # 禁止 Gazebo 去线上下载模型，避免卡住
    set_gazebo_model_database_uri = SetEnvironmentVariable(
        name="GAZEBO_MODEL_DATABASE_URI", value=""
    )
    # ------------------------------------------------------------

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, "config/gazebo_tcb710v_description.urdf.xacro")

    # 直接加载 Gazebo 自带 empty.world
    gazebo_default_world = "/usr/share/gazebo-11/worlds/empty.world"

    if not os.path.exists(gazebo_default_world):
        raise RuntimeError(
            f"Gazebo default world not found: {gazebo_default_world}\n"
            f"Please check your Gazebo version with:\n"
            f"  ls /usr/share/gazebo-*/worlds/default.world"
        )

    print("--- urdf_model_path:", urdf_model_path)
    print("--- gazebo_default_world:", gazebo_default_world)
    print("--- GAZEBO_MODEL_PATH:", gazebo_model_path_value)

    with open(urdf_model_path, "r") as urdf_file:
        doc = xacro.parse(urdf_file)

    xacro.process_doc(doc)

    params = {"robot_description": doc.toxml()}

    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            gazebo_default_world,
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True}, params, {"publish_frequency": 15.0}],
        output="screen",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", robot_name_in_model],
        output="screen",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "tcb_group_controller"],
        output="screen",
    )

    close_evt1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )

    close_evt2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_joint_trajectory_controller],
        )
    )

    ld = LaunchDescription(
        [
            set_gazebo_model_database_uri,
            set_gazebo_model_path,
            close_evt1,
            close_evt2,
            gazebo,
            node_robot_state_publisher,
            spawn_entity,
        ]
    )

    return ld
