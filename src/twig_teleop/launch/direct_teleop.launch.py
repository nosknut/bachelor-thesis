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
        DeclareLaunchArgument('joy_topic', default_value='/twig_joy'),
        
        DeclareLaunchArgument('shoulder_topic', default_value='/shoulder/servo/velocity/cmd'),
        DeclareLaunchArgument('wrist_topic', default_value='/wrist/servo/velocity/cmd'),
        DeclareLaunchArgument('gripper_topic', default_value='/gripper/servo/velocity/cmd'),
        
        DeclareLaunchArgument('activate_shoulder_servo_service', default_value='/shoulder/servo/activate'),
        DeclareLaunchArgument('activate_wrist_servo_service', default_value='/wrist/servo/activate'),
        DeclareLaunchArgument('activate_gripper_servo_service', default_value='/gripper/servo/activate'),
        
        DeclareLaunchArgument('deactivate_shoulder_servo_service', default_value='/shoulder/servo/deactivate'),
        DeclareLaunchArgument('deactivate_wrist_servo_service', default_value='/wrist/servo/deactivate'),
        DeclareLaunchArgument('deactivate_gripper_servo_service', default_value='/gripper/servo/deactivate'),
        
        DeclareLaunchArgument('twig_teleop_direct_config_filepath', default_value=[
            TextSubstitution(text=os.path.join(
                    get_package_share_directory('twig_teleop'),
                    'config',
                    'joystick_direct_teleop.config.yaml'
            )),
        ]),
        
        # Nodes
        ComposableNodeContainer(
            name='twig_teleop_direct_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    name='twig_teleop_direct',
                    package='twig_teleop',
                    plugin='twig_teleop::JoystickDirectTeleop',
                    parameters=[LaunchConfiguration('twig_teleop_direct_config_filepath')],
                    remappings={
                        ('/twig_joy', LaunchConfiguration('joy_topic')),
                        
                        ('/shoulder/servo/velocity/cmd', LaunchConfiguration('shoulder_topic')),
                        ('/wrist/servo/velocity/cmd', LaunchConfiguration('wrist_topic')),
                        ('/gripper/servo/velocity/cmd', LaunchConfiguration('gripper_topic')),
                        
                        ('/shoulder/servo/activate', LaunchConfiguration('activate_shoulder_servo_service')),
                        ('/wrist/servo/activate', LaunchConfiguration('activate_wrist_servo_service')),
                        ('/gripper/servo/activate', LaunchConfiguration('activate_gripper_servo_service')),
                        
                        ('/shoulder/servo/deactivate', LaunchConfiguration('deactivate_shoulder_servo_service')),
                        ('/wrist/servo/deactivate', LaunchConfiguration('deactivate_wrist_servo_service')),
                        ('/gripper/servo/deactivate', LaunchConfiguration('deactivate_gripper_servo_service')),
                    },
                ),
            ]
        )
    ])