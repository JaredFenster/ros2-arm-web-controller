from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

WEB_DIR = '/home/jared/ros2_ws/src/arm-planner/web'


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['node', os.path.join(WEB_DIR, 'server/index.js')],
            output='screen',
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='arm_planner',
                    executable='ros_bridge',
                    name='ros_bridge',
                    output='screen',
                ),
            ]
        ),
        ExecuteProcess(
            cmd=['npm', 'run', 'dev'],
            cwd=os.path.join(WEB_DIR, 'client'),
            output='screen',
        ),
    ])
