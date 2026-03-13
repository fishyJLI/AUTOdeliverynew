#!/usr/bin/env python3

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class State(Enum):
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3


class ObstacleAvoid(Node):
    def __init__(self):
        super().__init__('simple_obstacle_avoid')

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.state = State.FORWARD

        # Tunable parameters
        self.front_block_dist = 1.0
        self.side_check_max = 2.5
        self.forward_speed = 0.18
        self.turn_speed = 0.6
        self.clear_front_dist = 1.2

        self.latest_scan = None
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Obstacle avoidance node started')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def sector_min(self, scan, deg_min, deg_max):
        a_min = math.radians(deg_min)
        a_max = math.radians(deg_max)

        vals = []
        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < 0.10 or r > self.side_check_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            if a_min <= angle <= a_max:
                vals.append(r)

        if not vals:
            return float('inf')
        return min(vals)

    def control_loop(self):
        cmd = Twist()

        if self.latest_scan is None:
            self.cmd_pub.publish(cmd)
            return

        front_min = self.sector_min(self.latest_scan, -20, 20)
        left_min = self.sector_min(self.latest_scan, 20, 70)
        right_min = self.sector_min(self.latest_scan, -70, -20)

        if self.state == State.FORWARD:
            if front_min < self.front_block_dist:
                if left_min > right_min:
                    self.state = State.TURN_LEFT
                    self.get_logger().info(
                        f'Obstacle ahead. Turning LEFT | front={front_min:.2f} left={left_min:.2f} right={right_min:.2f}'
                    )
                else:
                    self.state = State.TURN_RIGHT
                    self.get_logger().info(
                        f'Obstacle ahead. Turning RIGHT | front={front_min:.2f} left={left_min:.2f} right={right_min:.2f}'
                    )
            else:
                cmd.linear.x = self.forward_speed

        elif self.state == State.TURN_LEFT:
            if front_min > self.clear_front_dist:
                self.state = State.FORWARD
                self.get_logger().info('Front clear. Moving FORWARD')
            else:
                cmd.angular.z = 0.6

        elif self.state == State.TURN_RIGHT:
            if front_min > self.clear_front_dist:
                self.state = State.FORWARD
                self.get_logger().info('Front clear. Moving FORWARD')
            else:
                cmd.angular.z = -0.6

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()