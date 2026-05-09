import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class SimpleBehaviorNode(Node):

    def __init__(self):
        super().__init__('simple_behavior_node')

        self.create_subscription(
            String,
            '/semantic_state',
            self.semantic_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.semantic_state = "CLEAR"

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info('Simple Behavior Node Started')

    def semantic_callback(self, msg):
        self.semantic_state = msg.data

    def control_loop(self):

        cmd = Twist()

        # =========================
        # PERSON → STOP
        # =========================
        if self.semantic_state == "PERSON":

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # =========================
        # OBSTACLE → BACKWARD
        # =========================
        elif self.semantic_state == "OBSTACLE":

            cmd.linear.x = -0.15
            cmd.angular.z = 0.0

        # =========================
        # CLEAR → FORWARD
        # =========================
        else:

            cmd.linear.x = 0.20
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'Semantic: {self.semantic_state} | '
            f'linear={cmd.linear.x:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = SimpleBehaviorNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()