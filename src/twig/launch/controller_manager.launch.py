import os
from launch_ros.actions import Node
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "twig", package_name="twig_moveit_config"
    ).to_moveit_configs()

    control_config = os.path.join(
        get_package_share_directory("twig_hardware"),
        "config",
        "ros2_controllers.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    moveit_config.robot_description,
                    control_config,
                ],
            ),
        ]
    )
