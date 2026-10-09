import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_dir = get_package_share_directory('task_solution')
    launch_dir = os.path.join(package_dir, 'launch')

    # 1. Starting the localization script
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'localization_launch.py')
        )
    )

    # 2. Start ing the navigation script after 5 seconds
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_launch.py')
                )
            )
        ]
    )

    # 3. Start the green detector after 10 seconds
    green_detector = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='task_solution',
                executable='green_detector',
                name='green_detector',
                output='screen',
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )

    # 4. Start the green navigator after 12 seconds
    green_nav = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='task_solution',
                executable='green_nav',
                name='green_navigator',
                output='screen',
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )

    return LaunchDescription([
        localization_launch,
        navigation_launch,
        green_detector,
        green_nav,
    ])