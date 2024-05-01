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
        # Arguments
        DeclareLaunchArgument('joy_topic', default_value='/twig_joy'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('deadzone', default_value="0.3"),
        
        # Nodes
        Node(
            package='joy',
            executable='joy_node',
            remappings=[
                ('/joy', LaunchConfiguration('joy_topic')),
            ],
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
                'deadzone': LaunchConfiguration('deadzone'),
        }]),
    ])