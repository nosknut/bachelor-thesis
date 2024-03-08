import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_file(package, launch_file):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package), "launch", launch_file)]
        )
    )


def generate_launch_description():
    return LaunchDescription(
        [
            launch_file("twig_moveit_config", "moveit_rviz.launch.py"),
        ]
    )
