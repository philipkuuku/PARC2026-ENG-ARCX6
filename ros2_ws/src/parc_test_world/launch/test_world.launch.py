import os

import yaml

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders
    pkg_path = FindPackageShare(package="parc_test_world").find("parc_test_world")
    pkg_bringup = FindPackageShare(package="parc_robot_bringup").find("parc_robot_bringup")
    pkg_description = FindPackageShare(package="parc_robot_description").find(
        "parc_robot_description"
    )
    pkg_ros_gz_sim = FindPackageShare(package="ros_gz_sim").find("ros_gz_sim")

    world_file = os.path.join(pkg_path, "worlds/test_arena.sdf")
    # Reuse the robot's bridge and RViz configuration from the bringup package
    bridge_params = os.path.join(pkg_bringup, "config/gz_bridge.yaml")
    rviz_config_file = os.path.join(pkg_bringup, "rviz/task.rviz")

    # AprilTag poles live in this package; tables and chairs come from the bringup package
    set_env_vars_test_world_models = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(pkg_path, "models")
    )
    set_env_vars_bringup_models = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(pkg_bringup, "models")
    )

    # Default spawn pose comes from the world parameter file
    with open(os.path.join(pkg_path, "config/world_params.yaml"), "r") as f:
        spawn = yaml.safe_load(f)["spawn"]

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        name="rviz", default_value="true", description="Start RViz if true"
    )
    declare_spawn_cmds = [
        DeclareLaunchArgument(
            name=axis,
            default_value=str(spawn[axis]),
            description=f"Robot spawn {axis}",
        )
        for axis in ("x", "y", "z", "yaw")
    ]

    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_description, "launch", "robot_state_publisher.launch.py")]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Launch Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawn SITO-E robot in Gazebo
    spawn_robot_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "sitoe_robot",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-Y",
            LaunchConfiguration("yaw"),
        ],
    )

    # Start Gazebo ROS bridge
    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    # Start Gazebo ROS Top Camera Color Image bridge
    start_gazebo_ros_top_camera_color_image_bridge_cmd = Node(
        package="ros_gz_image",
        namespace="top_camera",
        executable="image_bridge",
        arguments=["/top_camera_color/image_raw"],
        output="screen",
    )

    # Start Gazebo ROS Bottom Camera Color Image bridge
    start_gazebo_ros_bottom_camera_color_image_bridge_cmd = Node(
        package="ros_gz_image",
        namespace="bottom_camera",
        executable="image_bridge",
        arguments=["/bottom_camera_color/image_raw"],
        output="screen",
    )

    # Adjust default Gazebo camera view to look down over the arena
    set_gazebo_camera_view_cmd = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            "/gui/move_to/pose",
            "--reqtype",
            "gz.msgs.GUICamera",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            "pose: {position: {x: 0.0, y: -6.0, z: 5.5} orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}",
        ],
        output="screen",
    )

    delayed_camera_adjustment = TimerAction(
        period=7.0, actions=[set_gazebo_camera_view_cmd]  # Delay in seconds
    )

    # Launch RViz
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)
    for cmd in declare_spawn_cmds:
        ld.add_action(cmd)
    ld.add_action(set_env_vars_test_world_models)
    ld.add_action(set_env_vars_bringup_models)

    # Add any actions
    ld.add_action(start_rviz_cmd)
    ld.add_action(delayed_camera_adjustment)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_top_camera_color_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_bottom_camera_color_image_bridge_cmd)

    return ld
