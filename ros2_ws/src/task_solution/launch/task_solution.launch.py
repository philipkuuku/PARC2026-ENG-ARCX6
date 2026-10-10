import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package and launch directories
    package_dir = get_package_share_directory('task_solution')
    launch_dir = os.path.join(package_dir, 'launch')

    # Select which navigator to use
    navigator_version = LaunchConfiguration('navigator_version')

    declare_navigator_version = DeclareLaunchArgument(
        'navigator_version',
        default_value='1',
        description='Select navigator: 1=green_nav, 2=green_nav2'
    )

    # 1. Start localization immediately
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'localization_launch.py')
        )
    )

    # 2. Start Nav2 after 5 seconds
    navigation = TimerAction(
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
    detector = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='task_solution',
                executable='green_detector',
                name='green_detector',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    # 4. Start green_nav if navigator_version is 1
    navigator_1 = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='task_solution',
                executable='green_nav',
                name='green_navigator',
                output='screen',
                parameters=[{'use_sim_time': True}],
                condition=LaunchConfigurationEquals(
                    'navigator_version', '1'
                ),
            )
        ]
    )

    # 5. Start green_nav2 if navigator_version is 2
    navigator_2 = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='task_solution',
                executable='green_nav2',
                name='green_navigator',
                output='screen',
                parameters=[{'use_sim_time': True}],
                condition=LaunchConfigurationEquals(
                    'navigator_version', '2'
                ),
            )
        ]
    )

    return LaunchDescription([
        declare_navigator_version,
        localization,
        navigation,
        detector,
        navigator_1,
        navigator_2,
    ])