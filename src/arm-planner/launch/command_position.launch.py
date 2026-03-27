from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="arm-planner",
            executable="command_position.py",
            name="command_position",
            output="screen",
        )
    ])
