from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    showcase_behavior_node = Node(
        package='robot_mvp',
        executable='showcase_behavior_node',
        name='showcase_behavior_node',
        output='screen'
    )

    return LaunchDescription([
        showcase_behavior_node
    ])