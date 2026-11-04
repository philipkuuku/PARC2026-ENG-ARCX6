from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'parcbot_docking'
    apriltag_ros_config_filename = 'apriltags_36h11.yaml'

    # Path to your YAML config file
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_config_path = PathJoinSubstitution([pkg_share, 'config', apriltag_ros_config_filename])

    # Launch configuration variables
    apriltag_config_file = LaunchConfiguration('apriltag_config_file')
    camera_namespace = LaunchConfiguration('camera_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare arguments
    declare_config_cmd = DeclareLaunchArgument(
        name='apriltag_config_file',
        default_value=default_config_path,
        description='Path to AprilTag config file'
    )
    
    declare_camera_namespace_cmd = DeclareLaunchArgument(
        name='camera_namespace',
        default_value='top_camera_color',
        description='Namespace for the camera and AprilTag nodes'
    )

    declare_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    # 1. Create the Rectify Node (image_proc)
    rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify_node',
        namespace=camera_namespace,
        remappings=[
            ('image', 'image_raw'),
            ('image_rect', 'image_rect'),
        ],
        parameters=[{
            'queue_size': 5,
            'interpolation': 1,
            'use_sim_time': use_sim_time,
            'image_transport': 'raw'
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 2. Create the AprilTag Node
    apriltag_ros_node = ComposableNode(
        package='apriltag_ros',
        plugin='AprilTagNode',
        name='apriltag_node',
        namespace=camera_namespace,
        remappings=[
            ('image_rect', 'image_rect'),
            ('camera_info', 'camera_info'),
        ],
        parameters=[
            apriltag_config_file,
            {'use_sim_time': use_sim_time}
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 3. Create the Container running both nodes efficiently together
    container = ComposableNodeContainer(
        name='parcbot_vision_container',
        namespace=camera_namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            rectify_node,
            apriltag_ros_node
        ],
        output='screen'
    )

    # Assemble launch description
    ld = LaunchDescription()
    ld.add_action(declare_config_cmd)
    ld.add_action(declare_camera_namespace_cmd)
    ld.add_action(declare_sim_time_cmd)
    ld.add_action(container)

    return ld