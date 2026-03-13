#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


TARGET_DISTANCE = 1.2
DIST_TOL = 0.3


class LidarFollow(Node):

    def __init__(self):
        super().__init__('lidar_follow')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.get_logger().info("LiDAR follow node started")

    def scan_callback(self, msg):

        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        closest_distance = 999
        closest_angle = 0

        for i, r in enumerate(ranges):

            if math.isinf(r) or math.isnan(r):
                continue

            angle = angle_min + i * angle_increment

            # only look in front sector (-30° to +30°)
            if -math.radians(30) < angle < math.radians(30):

                if r < closest_distance:
                    closest_distance = r
                    closest_angle = angle

        cmd = Twist()

        if closest_distance == 999:
            # nothing detected
            self.cmd_pub.publish(cmd)
            return

        distance_error = closest_distance - TARGET_DISTANCE

        # forward/back control
        if abs(distance_error) > DIST_TOL:
            cmd.linear.x = 0.4 * distance_error

        # steering control
        cmd.angular.z = -1.5 * closest_angle

        # limit speeds
        cmd.linear.x = max(min(cmd.linear.x, 0.3), -0.3)
        cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"target {closest_distance:.2f} m angle {math.degrees(closest_angle):.1f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()