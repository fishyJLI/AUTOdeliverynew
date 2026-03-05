#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ---------- Launch args (ports + enable flags) ----------
    enable_camera = LaunchConfiguration('enable_camera')
    enable_imu    = LaunchConfiguration('enable_imu')
    enable_gps    = LaunchConfiguration('enable_gps')
    enable_lidar  = LaunchConfiguration('enable_lidar')

    imu_port   = LaunchConfiguration('imu_port')
    gps_port   = LaunchConfiguration('gps_port')
    lidar_port = LaunchConfiguration('lidar_port')

    gps_baud   = LaunchConfiguration('gps_baud')
    imu_baud   = LaunchConfiguration('imu_baud')

    # LSLidar params
    lidar_name = LaunchConfiguration('lidar_name')      # e.g. "N10_P"
    lidar_baud = LaunchConfiguration('lidar_baud')      # e.g. 460800

    # ---------- Package share paths ----------
    astra_share  = get_package_share_directory('astra_camera')
    yesense_share = get_package_share_directory('yesense_std_ros2')
    lslidar_share = get_package_share_directory('lslidar_driver')

    # Astra Pro launch (XML in astra_camera)
    astra_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(astra_share, 'launch', 'astra_pro.launch.xml')),
        condition=IfCondition(enable_camera),
    )

    # Yesense IMU launch (Python launch in yesense_std_ros2)
    yesense_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(yesense_share, 'launch', 'yesense_node.launch.py')),
        condition=IfCondition(enable_imu),
        launch_arguments={
            # These names must match the yesense launch file parameters.
            # If your yesense launch uses different arg names, tell me and I’ll adjust.
            'serial_port': imu_port,
            'baud_rate': imu_baud,
        }.items()
    )

    # GPS (nmea_navsat_driver)
    gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_navsat_driver',
        output='screen',
        condition=IfCondition(enable_gps),
        parameters=[{
            'port': gps_port,
            'baud': gps_baud,
            'frame_id': 'gps',
        }]
    )

    # LSLidar (serial mode) — use the driver yaml then override key params
    # NOTE: lslidar yaml key for serial is "serial_port_" (with underscore) in your file.
    default_lslidar_yaml = os.path.join(lslidar_share, 'params', 'lsx10.yaml')

    lslidar_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        condition=IfCondition(enable_lidar),
        parameters=[
            default_lslidar_yaml,
            {
                'interface_selection': 'serial',
                'serial_port_': lidar_port,
                'lidar_name': lidar_name,
                'baud_rate': lidar_baud,
                'pubScan': True,
                'pubPointCloud2': False,
                'scan_topic': '/scan',
                'frame_id': 'laser_link',
            }
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_imu',    default_value='false'),
        DeclareLaunchArgument('enable_gps',    default_value='false'),
        DeclareLaunchArgument('enable_lidar',  default_value='true'),

        DeclareLaunchArgument('imu_port',   default_value='/dev/ttyACM1'),
        DeclareLaunchArgument('imu_baud',   default_value='921600'),

        DeclareLaunchArgument('gps_port',   default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('gps_baud',   default_value='9600'),

        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_name', default_value='N10_P'),
        DeclareLaunchArgument('lidar_baud', default_value='460800'),

        astra_launch,
        yesense_launch,
        gps_node,
        lslidar_node,
    ])
