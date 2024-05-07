import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import TextSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
        
    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument('twig_hardware_config_filepath', default_value=[
                TextSubstitution(text=os.path.join(
                        get_package_share_directory('twig_hardware'),
                        'config',
                        'node.config.yaml'
                )),
            ]),
            
            # Nodes
            ComposableNodeContainer(
                name="twig_hardware_node_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        name="twig_hardware_node",
                        package="twig_hardware",
                        plugin="twig_hardware::TwigHardwareNode",
                        parameters=[LaunchConfiguration('twig_hardware_config_filepath')],
                    ),
                ],
            )
        ]
    )
