import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
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
            launch_file("twig", "controller_manager.launch.py"),
            launch_file("twig_moveit_config", "move_group.launch.py"),
            launch_file("twig_moveit_config", "rsp.launch.py"),
            launch_file("twig_moveit_config", "spawn_controllers.launch.py"),
            launch_file("twig_moveit_config", "static_virtual_joint_tfs.launch.py"),
        ]
    )
