import os
from launch import LaunchDescription
from launch.substitutions import TextSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('out_jog_topic', default_value='/servo_node/delta_joint_cmds'),
        DeclareLaunchArgument('out_twist_topic', default_value='/servo_node/delta_twist_cmds'),
        DeclareLaunchArgument('start_servo_service', default_value='/servo_node/start_servo'),
        DeclareLaunchArgument('config_filepath', default_value=[
            TextSubstitution(text=os.path.join(
                    get_package_share_directory('panda_teleop'),
                    'config',
                    'joystick_teleop.config.yaml'
            )),
        ]),
        
        # Nodes
        ComposableNodeContainer(
            name='panda_teleop_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    name='panda_teleop',
                    package='panda_teleop',
                    plugin='panda_teleop::JoystickTeleop',
                    parameters=[LaunchConfiguration('config_filepath')],
                    remappings={
                        ('/joy', LaunchConfiguration('joy_topic')),
                        ('/cmd_jog', LaunchConfiguration('out_jog_topic')),
                        ('/cmd_vel', LaunchConfiguration('out_twist_topic')),
                        ('/start_servo', LaunchConfiguration('start_servo_service')),
                    },
                ),
            ]
        )
    ])