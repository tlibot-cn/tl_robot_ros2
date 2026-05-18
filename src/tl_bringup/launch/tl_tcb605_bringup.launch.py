import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    arm_type = 'tcb605'
    use_sim = 'false'

    tl_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tl_driver'),
                'launch',
                'tl_driver.launch.py'
            )
        ),
        launch_arguments={
            'arm_type': arm_type,
        }.items()
    )

    tl_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tl_description'),
                'launch',
                'tl_description.launch.py'
            )
        ),
        launch_arguments={
            'arm_type': arm_type,
            'use_sim': use_sim
        }.items(),
    )

    return LaunchDescription([
        tl_driver,
        tl_description,
    ])