# Launch file to start the robot state publisher node

import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch config variables
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Process files
    pkg_path = FindPackageShare(package="parc_robot_description").find(
        "parc_robot_description"
    )
    urdf_model_path = os.path.join(pkg_path, "urdf/sitoe_robot.urdf.xacro")
    robot_description_config = Command(
        [
            "xacro ",
            urdf_model_path,
            " sim_mode:=",
            use_sim_time,
        ]
    )

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) time if true",
    )

    # Start robot state publisher node
    params = {
        "robot_description": robot_description_config,
        "use_sim_time": use_sim_time,
    }
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[params],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
