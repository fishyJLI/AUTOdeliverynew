import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys
import termios
import tty


class KeyboardCmdVelNode(Node):

    def __init__(self):
        super().__init__('keyboard_cmdvel_node')

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info('Keyboard CmdVel Node Started')
        self.get_logger().info('Controls:')
        self.get_logger().info('W = forward')
        self.get_logger().info('S = backward')
        self.get_logger().info('A = left')
        self.get_logger().info('D = right')
        self.get_logger().info('X = stop')
        self.get_logger().info('Q = quit')

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        return key

    def run(self):

        while rclpy.ok():

            key = self.get_key()

            cmd = Twist()

            # Forward
            if key == 'w':
                cmd.linear.x = 0.2

            # Backward
            elif key == 's':
                cmd.linear.x = -0.2

            # Turn left
            elif key == 'a':
                cmd.angular.z = 0.8

            # Turn right
            elif key == 'd':
                cmd.angular.z = -0.8

            # Stop
            elif key == 'x':
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            # Quit
            elif key == 'q':
                self.get_logger().info('Exiting...')
                break

            else:
                continue

            self.publisher.publish(cmd)

            self.get_logger().info(
                f'Published cmd_vel: '
                f'linear={cmd.linear.x:.2f}, '
                f'angular={cmd.angular.z:.2f}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = KeyboardCmdVelNode()

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()