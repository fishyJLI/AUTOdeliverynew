#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


TARGET_DISTANCE = 1.0
DIST_TOL = 0.15

FRONT_ANGLE_DEG = 30
ANGLE_DEADBAND_DEG = 8
BIG_TURN_DEG = 15

MAX_LINEAR = 0.25
MAX_ANGULAR = 0.8

KP_LINEAR = 0.35
KP_ANGULAR = 0.8


class LidarFollow(Node):
    def __init__(self):
        super().__init__('lidar_follow')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info('LiDAR follow node started')

    def scan_callback(self, msg: LaserScan):
        closest_distance = float('inf')
        closest_angle = 0.0

        front_limit = math.radians(FRONT_ANGLE_DEG)

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            # ignore nonsense / ultra-close noise
            if r < 0.15 or r > 6.0:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            if -front_limit < angle < front_limit:
                if r < closest_distance:
                    closest_distance = r
                    closest_angle = angle

        cmd = Twist()

        # no valid target -> stop
        if closest_distance == float('inf'):
            self.cmd_pub.publish(cmd)
            self.get_logger().info('No target detected -> stop')
            return

        distance_error = closest_distance - TARGET_DISTANCE
        angle_deadband = math.radians(ANGLE_DEADBAND_DEG)
        big_turn_thresh = math.radians(BIG_TURN_DEG)

        # Big angle error: rotate first, don't drive forward
        if abs(closest_angle) > big_turn_thresh:
            cmd.linear.x = 0.0
            cmd.angular.z = -KP_ANGULAR * closest_angle

        else:
            # Distance control
            if abs(distance_error) > DIST_TOL:
                cmd.linear.x = KP_LINEAR * distance_error
            else:
                cmd.linear.x = 0.0

            # Small angle correction only if needed
            if abs(closest_angle) > angle_deadband:
                cmd.angular.z = -0.4 * closest_angle
            else:
                cmd.angular.z = 0.0

        # Speed limits
        cmd.linear.x = max(min(cmd.linear.x, MAX_LINEAR), -MAX_LINEAR)
        cmd.angular.z = max(min(cmd.angular.z, MAX_ANGULAR), -MAX_ANGULAR)

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'target={closest_distance:.2f}m '
            f'angle={math.degrees(closest_angle):.1f}deg '
            f'cmd_x={cmd.linear.x:.2f} cmd_z={cmd.angular.z:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()