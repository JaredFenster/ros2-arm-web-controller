from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package="panda_gz_sim",
            executable="map_points",
            name="map_points",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ])
