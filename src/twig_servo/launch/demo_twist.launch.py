import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("twig")
        .robot_description(file_path="config/twig.urdf.xacro")
        .to_moveit_configs()
    )

    servo_yaml = load_yaml("twig_servo", "config/twig_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    return LaunchDescription(
        [
            Node(
                package="moveit_servo",
                executable="servo_node_main",
                parameters=[
                    servo_params,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
                output="screen",
            ),
            ComposableNodeContainer(
                name="twig_servo_container",
                namespace="/",
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=[
                    ComposableNode(
                        package="tf2_ros",
                        plugin="tf2_ros::StaticTransformBroadcasterNode",
                        name="static_tf2_broadcaster",
                        parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
                    ),
                ],
                output="screen",
            )
        ]
    )