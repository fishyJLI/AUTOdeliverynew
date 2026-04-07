from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_lidar = LaunchConfiguration('use_lidar')
    use_imu = LaunchConfiguration('use_imu')
    use_camera = LaunchConfiguration('use_camera')

    lslidar_pkg = get_package_share_directory('lslidar_driver')
    imu_pkg = get_package_share_directory('yesense_std_ros2')

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lslidar_pkg, 'launch', 'lslidar_launch.py')
        ),
        condition=IfCondition(use_lidar)
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_pkg, 'launch', 'yesense_node.launch.py')
        ),
        condition=IfCondition(use_imu)
    )

    camera_node = Node(
        package='astra_camera',
        executable='astra_camera_node',
        name='astra_camera',
        output='screen',
        condition=IfCondition(use_camera),
        parameters=[
            {
                'enable_color': True,
                'enable_depth': False,
                'enable_ir': False,
                'enable_point_cloud': False,
                'enable_colored_point_cloud': False,
                'publish_tf': False,
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30,
            }
        ]
    )

    odom_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.12', '0.0', '0.18', '0', '0', '0', 'base_link', 'laser_link']
    )

    base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
    )

    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.10', '0.0', '0.20', '0', '0', '0', 'base_link', 'camera_link']
    )

    base_to_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),

        lidar_launch,
        imu_launch,
        camera_node,

        odom_to_base_tf,
        base_to_laser_tf,
        base_to_imu_tf,
        base_to_camera_tf,
        base_to_footprint_tf,
    ])