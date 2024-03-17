from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # https://github.com/ros-tooling/topic_tools#relayfield
    # ros2 run topic_tools transform /joy /shoulder/servo/velocity/cmd std_msgs/Float32 'std_msgs.msg.Float32(data=m.axes[1])' --import std_msgs
    # ros2 run topic_tools transform /joy /wrist/servo/velocity/cmd std_msgs/Float32 'std_msgs.msg.Float32(data=m.axes[0])' --import std_msgs
    # ros2 run topic_tools transform /joy /gripper/servo/velocity/cmd std_msgs/Float32 'std_msgs.msg.Float32(data=m.axes[3])' --import std_msgs
    
    return LaunchDescription([
        Node(
            package='topic_tools',
            executable='transform',
            arguments=[
                '/joy',
                '/shoulder/servo/velocity/cmd',
                'std_msgs/Float32',
                "'std_msgs.msg.Float32(data=m.axes[1])'",
                '--import',
                'std_msgs'
            ],
        ),
        Node(
            package='topic_tools',
            executable='transform',
            arguments=[
                '/joy',
                '/wrist/servo/velocity/cmd',
                'std_msgs/Float32',
                "'std_msgs.msg.Float32(data=m.axes[0])'",
                '--import',
                'std_msgs'
            ],
        ),
        Node(
            package='topic_tools',
            executable='transform',
            arguments=[
                '/joy',
                '/gripper/servo/velocity/cmd',
                'std_msgs/Float32',
                "'std_msgs.msg.Float32(data=m.axes[3])'",
                '--import',
                'std_msgs'
            ],
        ),
    ])