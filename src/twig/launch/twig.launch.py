import os
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, LaunchConfigurationEquals

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('twig_teleop'), 'launch'), '/foxglove.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('twig_teleop'), 'launch'), '/teleop.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('twig_servo'), 'launch'), '/servo.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('twig_moveit_config'), 'launch'), '/demo.launch.py'])),
    ])