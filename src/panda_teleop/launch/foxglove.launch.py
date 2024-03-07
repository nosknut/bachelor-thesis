import os
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition, LaunchConfigurationEquals

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([os.path.join(
                get_package_share_directory('foxglove_bridge'), 'launch'),
                '/foxglove_bridge_launch.xml']),
        ),
    ])