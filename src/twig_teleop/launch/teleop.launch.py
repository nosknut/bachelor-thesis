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
    # republish as stamped
    # ros2 run topic_tools relay_field /turtle1/cmd_vel /servo_node/delta_twist_cmds geometry_msgs/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'panda_link0'}, twist: m}"
    # ros2 run topic_tools relay_field /turtle1/cmd_vel /servo_node/delta_twist_cmds geometry_msgs/TwistStamped "{header: {frame_id: 'panda_link0'}, twist: m}"
    # ros2 run topic_tools relay_field /turtle1/cmd_vel /servo_node/delta_twist_cmds geometry_msgs/TwistStamped "{twist: m}"
    return LaunchDescription([
        DeclareLaunchArgument('joy_topic', default_value='joy'),
        DeclareLaunchArgument('out_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('config_filepath', default_value=[
            TextSubstitution(text=os.path.join(
                    get_package_share_directory('twig_teleop'),
                    'config',
                    'teleop.config.yaml'
            )),
        ]),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            parameters=[LaunchConfiguration('config_filepath')],
            remappings={('/cmd_vel', LaunchConfiguration('out_topic'))},
        ),
        # Node(
        #     package='turtlesim',
        #     executable='turtlesim_node',
        #     remappings={('/turtle1/cmd_vel', LaunchConfiguration('out_topic'))},
        # ),
    ])