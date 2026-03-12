#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SimpleObstacleAvoid(Node):
    def __init__(self):
        super().__init__('simple_obstacle_avoid')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/obstacle_cmd_vel', 10)#self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.front_distance = float('inf')

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Simple obstacle avoidance node started.')

    def scan_callback(self, msg: LaserScan):
        # Check roughly the front sector: -20 deg to +20 deg
        front_ranges = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle)

            if -20.0 <= angle_deg <= 20.0:
                if math.isfinite(r) and r > 0.0:
                    front_ranges.append(r)

        if front_ranges:
            self.front_distance = min(front_ranges)
        else:
            self.front_distance = float('inf')

    def control_loop(self):
        cmd = Twist()

        if self.front_distance < 0.4:
            # Emergency stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Too close: STOP')
        elif self.front_distance < 1.0:
            # Obstacle ahead: turn left
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self.get_logger().info(f'Obstacle detected at {self.front_distance:.2f} m: TURN')
        else:
            # Clear path: move forward
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleObstacleAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()