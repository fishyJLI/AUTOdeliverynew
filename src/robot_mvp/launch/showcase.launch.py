from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot_mvp_pkg = get_package_share_directory('robot_mvp')

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_mvp_pkg, 'launch', 'sensors.launch.py')
        ),
        launch_arguments={
            'use_camera': 'false',
            'use_imu': 'false'
        }.items()
    )

    cmdvel_serial_bridge = Node(
        package='robot_mvp',
        executable='cmdvel_serial_bridge',
        name='cmdvel_serial_bridge',
        output='screen'
    )

    lidar_safety_node = Node(
        package='robot_mvp',
        executable='lidar_safety_node',
        name='lidar_safety_node',
        output='screen'
    )

    person_detection_node = Node(
        package='robot_mvp',
        executable='person_detection_node',
        name='person_detection_node',
        output='screen'
    )

    semantic_decision_node = Node(
        package='robot_mvp',
        executable='semantic_decision_node',
        name='semantic_decision_node',
        output='screen'
    )

    showcase_behavior_node = Node(
        package='robot_mvp',
        executable='showcase_behavior_node',
        name='showcase_behavior_node',
        output='screen'
    )

    return LaunchDescription([
        sensors_launch,

        TimerAction(
            period=2.0,
            actions=[
                cmdvel_serial_bridge,
                lidar_safety_node,
                person_detection_node,
                semantic_decision_node,
                showcase_behavior_node,
            ]
        )
    ])
