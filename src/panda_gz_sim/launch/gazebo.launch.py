"""
gazebo.launch.py
================
Launches a full Gazebo Sim + MoveIt 2 session for the Franka Panda arm
with a wrist-mounted depth camera.

What this file starts
---------------------
1. Gazebo Sim (gz sim) with a custom world.
2. robot_state_publisher — broadcasts URDF + TF.
3. ros_gz_sim spawn_entity — drops the robot into Gazebo.
4. gz_ros2_control — connects ros2_control to the Gazebo physics.
5. joint_state_broadcaster + panda_arm_controller + panda_hand_controller.
6. ros_gz_bridge — bridges Gazebo camera topics to ROS 2.
7. MoveIt 2 move_group with the existing panda-arm MoveIt config.
8. RViz (optional, enabled by default).

Required apt packages (install once)
--------------------------------------
  sudo apt install \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # ------------------------------------------------------------------ #
    # Package directories                                                  #
    # ------------------------------------------------------------------ #
    pkg_panda_gz_sim    = get_package_share_directory("panda_gz_sim")
    pkg_ros_gz_sim      = get_package_share_directory("ros_gz_sim")

    # ------------------------------------------------------------------ #
    # Launch arguments                                                     #
    # ------------------------------------------------------------------ #
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz with the MoveIt config",
    )
    gz_headless_arg = DeclareLaunchArgument(
        "gz_headless",
        default_value="false",
        description="Run Gazebo in headless/server mode (no GUI)",
    )

    use_rviz    = LaunchConfiguration("use_rviz")
    gz_headless = LaunchConfiguration("gz_headless")

    # ------------------------------------------------------------------ #
    # Paths                                                                #
    # ------------------------------------------------------------------ #
    urdf_xacro       = os.path.join(pkg_panda_gz_sim, "urdf", "panda_gz.urdf.xacro")
    srdf_file        = os.path.join(pkg_panda_gz_sim, "config", "panda_gz.srdf")
    controllers_yaml = os.path.join(pkg_panda_gz_sim, "config", "ros2_controllers.yaml")
    world_file       = os.path.join(pkg_panda_gz_sim, "worlds", "empty_with_ground.sdf")
    bridge_yaml      = os.path.join(pkg_panda_gz_sim, "config", "camera_bridge.yaml")

    # ------------------------------------------------------------------ #
    # Robot description (URDF via xacro)                                  #
    # ------------------------------------------------------------------ #
    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_xacro]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # ------------------------------------------------------------------ #
    # 1. Gazebo Sim                                                        #
    # ------------------------------------------------------------------ #
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r ", world_file],   # -r = run immediately
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # 2. robot_state_publisher                                             #
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # NOTE: world → panda_link0 is already published by robot_state_publisher
    # via the world_joint fixed joint in the URDF.  A duplicate static_transform_publisher
    # for the same frame pair conflicts with RSP's /tf_static publication and
    # can break the tf2_ros::MessageFilter inside PointCloudOctomapUpdater.

    # ------------------------------------------------------------------ #
    # 3. Spawn robot into Gazebo                                           #
    # ------------------------------------------------------------------ #
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "panda",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # 4-5. ros2_control nodes                                              #
    # gz_ros2_control is loaded via the URDF plugin; we only need to      #
    # spawn the individual controllers after the controller_manager is up. #
    # ------------------------------------------------------------------ #
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "120",
        ],
        output="screen",
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "120",
        ],
        output="screen",
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_hand_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "120",
        ],  
        output="screen",
    )

    # Delay arm/hand controllers until joint_state_broadcaster is active
    delay_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[panda_arm_controller_spawner, panda_hand_controller_spawner],
        )
    )

    # Delay jsb until the robot has been spawned
    delay_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # ------------------------------------------------------------------ #
    # 6. ros_gz_bridge — camera topics Gazebo → ROS 2                     #
    # ------------------------------------------------------------------ #
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        parameters=[{"config_file": bridge_yaml}],
        output="screen",
    )

    floor_image_projector = Node(
        package="panda_gz_sim",
        executable="floor_image_projector.py",
        name="floor_image_projector",
        parameters=[
            {"use_sim_time": True},
            {"image_topic": "/camera/color/image_raw"},
            {"camera_info_topic": "/camera/color/camera_info"},
            {"output_topic": "/camera/ground_marker"},
            {"world_frame": "world"},
            {"ground_z": 0.0},
            # World-space extent of the ground mesh (metres)
            {"x_min": -0.3},
            {"x_max":  1.0},
            {"y_min": -0.7},
            {"y_max":  0.7},
            # Cells per metre — raise for sharper image, lower for better perf
            {"pixels_per_meter": 50},
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # 7. MoveIt 2 move_group                                               #
    # Uses the existing panda-arm MoveIt config but overrides the         #
    # robot_description with the Gazebo-aware URDF and switches the       #
    # controller interface to the real ros2_control topics.               #
    # ------------------------------------------------------------------ #
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda-arm")
        # Override URDF with our Gazebo version (has gz_ros2_control + camera)
        .robot_description(file_path=urdf_xacro)
        # Override SRDF: removes virtual_joint clash, adds camera collision disables
        .robot_description_semantic(file_path=srdf_file)
        .trajectory_execution(
            file_path=os.path.join(pkg_panda_gz_sim, "config", "moveit_controllers.yaml")
        )
        # Feed the depth point cloud into MoveIt's Octomap for live collision avoidance
        .sensors_3d(
            file_path=os.path.join(pkg_panda_gz_sim, "config", "sensors_3d.yaml")
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            # Publish incremental planning scene updates so RViz sees live
            # Octomap/voxel changes instead of only the initial snapshot.
            {"publish_planning_scene": True},
            {"publish_geometry_updates": True},
            {"publish_state_updates": True},
            {"publish_transforms_updates": True},
            # panda_finger_joint1 drifts to ~-1e-15 after closing (float epsilon
            # below the lower bound of 0). Tolerate small bound violations so
            # CheckStartStateBounds doesn't abort every subsequent plan.
            {"start_state_max_bounds_error": 0.0001},
            # Octomap — must match sensors_3d.yaml
            {"octomap_frame": "world"},
            {"octomap_resolution": 0.025},
        ],
    )

    # ------------------------------------------------------------------ #
    # 8. RViz                                                              #
    # ------------------------------------------------------------------ #
    rviz_config = os.path.join(
        get_package_share_directory("panda-arm"), "config", "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(use_rviz),
    )

    # ------------------------------------------------------------------ #
    # Assemble                                                             #
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        use_rviz_arg,
        gz_headless_arg,

        gz_sim,

        # Start the clock bridge first so /clock is available before anything
        # that uses use_sim_time=True publishes TF or subscribes to topics.
        # robot_state_publisher must receive at least one /clock tick before it
        # publishes TF, otherwise all TF frames get wall-clock timestamps and
        # the tf2 message filter inside OccupancyMapMonitor silently drops every
        # point cloud (sim-time stamps never match wall-time TF entries).
        camera_bridge,

        # Give the clock bridge 1 s to connect and deliver the first /clock tick
        # before starting RSP, move_group, and the rest.
        TimerAction(period=1.0, actions=[
            robot_state_publisher,
            Node(
                package="panda_gz_sim",
                executable="pointcloud_restamper.py",
                name="pointcloud_restamper",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            move_group_node,
            floor_image_projector,
            rviz_node,
            # Box detector — waits idle until triggered via the service.
            # Trigger with:  ros2 run panda_gz_sim scan_and_add_box.py
            # Or:            ros2 service call /detect_and_add_box std_srvs/srv/Trigger "{}"
            Node(
                package="panda_gz_sim",
                executable="box_detector.py",
                name="box_detector",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]),

        # Move Gazebo GUI camera to a fixed vantage point once the GUI is up
        TimerAction(period=5.0, actions=[
            ExecuteProcess(
                cmd=[
                    "gz", "service",
                    "-s", "/gui/move_to/pose",
                    "--reqtype", "gz.msgs.GUICamera",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "500",
                    "--req", "pose: {position:{x: 0.5, y:-0.6, z:0.6}, orientation:{x:-0, y:0, z:1.6, w:1}}",
                ],
                output="screen",
            ),
        ]),

        # Spawn robot 3 s after launch (Gazebo needs time to load the world)
        TimerAction(period=3.0, actions=[spawn_robot]),

        delay_jsb_after_spawn,
        delay_arm_after_jsb,
    ])

