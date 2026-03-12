#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarFollow(Node):
    def __init__(self):
        super().__init__('lidar_follow')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/follow_cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.target_distance = 1.0
        self.max_linear = 0.18
        self.max_angular = 0.8

        self.k_linear = 0.5
        self.k_angular = 1.2

        self.stop_distance = 0.45
        self.front_half_angle_deg = 35.0
        self.target_timeout_sec = 0.5

        self.target_distance_meas = None
        self.target_angle_meas = None
        self.last_target_time = self.get_clock().now()

        self.get_logger().info('LiDAR follow node started.')

    def scan_callback(self, msg: LaserScan):
        best_range = None
        best_angle = None

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle)

            if -self.front_half_angle_deg <= angle_deg <= self.front_half_angle_deg:
                if best_range is None or r < best_range:
                    best_range = r
                    best_angle = angle

        if best_range is not None:
            self.target_distance_meas = best_range
            self.target_angle_meas = best_angle
            self.last_target_time = self.get_clock().now()

    def control_loop(self):
        cmd = Twist()

        now = self.get_clock().now()
        age = (now - self.last_target_time).nanoseconds / 1e9

        if self.target_distance_meas is None or self.target_angle_meas is None or age > self.target_timeout_sec:
            self.cmd_pub.publish(cmd)
            return

        dist = self.target_distance_meas
        ang = self.target_angle_meas

        if dist < self.stop_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        distance_error = dist - self.target_distance

        linear_x = self.k_linear * distance_error
        angular_z = self.k_angular * ang

        linear_x = max(min(linear_x, self.max_linear), -self.max_linear)
        angular_z = max(min(angular_z, self.max_angular), -self.max_angular)

        if abs(distance_error) < 0.08:
            linear_x = 0.0

        if abs(ang) < math.radians(4.0):
            angular_z = 0.0

        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()