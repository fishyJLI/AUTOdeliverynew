from setuptools import find_packages, setup

package_name = 'robot_mvp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='justinli',
    maintainer_email='hijustinli123@gmail.com',
    description='ROS2 delivery robot nodes (lidar, depth camera, GPS, IMU)',
    license='Not needed yet',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'hello_node = robot_mvp.hello_node:main',
            'gps_waypoint_follower = robot_mvp.gps_waypoint_follower:main',
            'depth_stop_turn = robot_mvp.depth_stop_turn:main',
            'cmd_vel_mux = robot_mvp.cmd_vel_mux:main',
            'turn_90 = robot_mvp.turn_90:main',
            'follow_person_depth = robot_mvp.follow_person_depth:main',
            'mode_cmdvel_mux = robot_mvp.mode_cmdvel_mux:main',
            'cmdvel_serial_bridge = robot_mvp.cmdvel_serial_bridge:main',
        ],
    },
)
