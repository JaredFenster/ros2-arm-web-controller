from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch
from moveit_configs_utils.launches import generate_spawn_controllers_launch
from moveit_configs_utils.launches import generate_move_group_launch
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("URDF", package_name="arm_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    return LaunchDescription(
        generate_rsp_launch(moveit_config).entities
        + generate_move_group_launch(moveit_config).entities
        + [ros2_control_node]
        + generate_spawn_controllers_launch(moveit_config).entities
        + generate_moveit_rviz_launch(moveit_config).entities
    )