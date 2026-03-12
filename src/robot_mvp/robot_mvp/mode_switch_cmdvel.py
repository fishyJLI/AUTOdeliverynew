#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ModeSwitch(Node):

    def __init__(self):
        super().__init__('mode_switch_cmdvel')

        self.mode = "manual"

        self.manual_cmd = Twist()
        self.obstacle_cmd = Twist()
        self.follow_cmd = Twist()

        self.create_subscription(
            Twist,
            '/manual_cmd_vel',
            self.manual_callback,
            10)

        self.create_subscription(
            Twist,
            '/obstacle_cmd_vel',
            self.obstacle_callback,
            10)

        self.create_subscription(
            Twist,
            '/follow_cmd_vel',
            self.follow_callback,
            10)

        self.create_subscription(
            String,
            '/drive_mode',
            self.mode_callback,
            10)

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.timer = self.create_timer(
            0.05,
            self.publish_cmd)

        self.get_logger().info("Mode switch started")

    def manual_callback(self, msg):
        self.manual_cmd = msg

    def obstacle_callback(self, msg):
        self.obstacle_cmd = msg

    def follow_callback(self, msg):
        self.follow_cmd = msg

    def mode_callback(self, msg):
        new_mode = msg.data.lower()

        if new_mode in ["manual", "obstacle", "follow", "stop"]:
            self.mode = new_mode
            self.get_logger().info(f"Mode changed to {self.mode}")

    def publish_cmd(self):

        cmd = Twist()

        if self.mode == "manual":
            cmd = self.manual_cmd

        elif self.mode == "obstacle":
            cmd = self.obstacle_cmd

        elif self.mode == "follow":
            cmd = self.follow_cmd

        elif self.mode == "stop":
            cmd = Twist()

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = ModeSwitch()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()