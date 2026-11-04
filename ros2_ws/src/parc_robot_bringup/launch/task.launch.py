import os

import yaml

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders
    pkg_path = FindPackageShare(
        package="parc_robot_bringup"
    ).find("parc_robot_bringup")

    pkg_description = FindPackageShare(
        package="parc_robot_description"
    ).find("parc_robot_description")

    pkg_ros_gz_sim = FindPackageShare(
        package="ros_gz_sim"
    ).find("ros_gz_sim")

    pkg_test_world = FindPackageShare(
        package="parc_test_world"
    ).find("parc_test_world")

    bridge_params = os.path.join(pkg_path, "config/gz_bridge.yaml")
    rviz_config_file = os.path.join(pkg_path, "rviz/task.rviz")

    world_file = os.path.join(
        pkg_test_world,
        "worlds",
        "test_arena.sdf",
    )

    # Add both model directories to Gazebo's resource search path
    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.pathsep.join([
            os.path.join(pkg_path, "models"),
            os.path.join(pkg_test_world, "models"),
        ])
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    pkg_description,
                    "launch",
                    "robot_state_publisher.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Launch Gazebo and spawn the SITO-E robot
    def spawn_gazebo_entities(context):

        nonlocal world_file

        actions = []

        # Set path to the task parameter YAML file
        params_file = os.path.join(
            pkg_path,
            "config/task_params.yaml",
        )

        # Open task parameter YAML file
        if os.path.exists(params_file):

            with open(params_file, "r") as f:
                params = yaml.safe_load(f)

            spawn_x_val = str(params["/**"]["ros__parameters"]["x"])
            spawn_y_val = str(params["/**"]["ros__parameters"]["y"])
            spawn_z_val = str(params["/**"]["ros__parameters"]["z"])
            spawn_yaw_val = str(params["/**"]["ros__parameters"]["yaw"])

        
            # Launch Gazebo
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            os.path.join(
                                pkg_ros_gz_sim,
                                "launch",
                                "gz_sim.launch.py",
                            )
                        ]
                    ),
                    launch_arguments={
                        "gz_args": ["-r -v4 ", world_file],
                        "on_exit_shutdown": "true",
                    }.items(),
                )
            )

            # Spawn SITO-E robot in Gazebo
            actions.append(
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    output="screen",
                    arguments=[
                        "-topic",
                        "robot_description",
                        "-name",
                        "sitoe_robot",
                        "-x",
                        spawn_x_val,
                        "-y",
                        spawn_y_val,
                        "-z",
                        spawn_z_val,
                        "-Y",
                        spawn_yaw_val,
                    ],
                )
            )

        return actions

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

    # Start Gazebo ROS top camera color image bridge
    start_gazebo_ros_top_camera_color_image_bridge_cmd = Node(
        package="ros_gz_image",
        namespace="top_camera",
        executable="image_bridge",
        arguments=["/top_camera_color/image_raw"],
        output="screen",
    )

    # Start Gazebo ROS bottom camera color image bridge
    start_gazebo_ros_bottom_camera_color_image_bridge_cmd = Node(
        package="ros_gz_image",
        namespace="bottom_camera",
        executable="image_bridge",
        arguments=["/bottom_camera_color/image_raw"],
        output="screen",
    )

    # Adjust default Gazebo camera view
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
            "pose: {position: {x: -0.2599, y: -9.7375, z: 3.8379} orientation: {x: -0.20186, y: 0.2022, z: 0.6772, w: 0.6780}}",
        ],
        output="screen",
    )

    delayed_camera_adjustment = TimerAction(
        period=7.0,
        actions=[set_gazebo_camera_view_cmd],
    )

    # Launch RViz
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # Create the launch description
    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(set_env_vars_resources)

    # Add launch actions
    ld.add_action(start_rviz_cmd)
    ld.add_action(delayed_camera_adjustment)
    ld.add_action(
        OpaqueFunction(function=spawn_gazebo_entities)
    )
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_top_camera_color_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_bottom_camera_color_image_bridge_cmd)

    return ld