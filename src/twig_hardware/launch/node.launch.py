from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription(
        [
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
                    ),
                ],
            )
        ]
    )
