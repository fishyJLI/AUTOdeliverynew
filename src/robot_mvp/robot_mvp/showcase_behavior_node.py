import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class ShowcaseBehaviorNode(Node):
    def __init__(self):
        super().__init__('showcase_behavior_node')

        self.semantic_state = "CLEAR"

        self.create_subscription(
            String,
            '/semantic_state',
            self.semantic_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.led_pub = self.create_publisher(String, '/led_state', 10)

        self.forward_speed = 0.18

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Showcase Behavior Node started")

    def semantic_callback(self, msg):
        self.semantic_state = msg.data.strip().upper()

    def control_loop(self):
        cmd = Twist()
        led = String()

        if self.semantic_state == "PERSON":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            led.data = "FLASH"

        elif self.semantic_state == "OBSTACLE":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            led.data = "OFF"

        else:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
            led.data = "OFF"

        self.cmd_pub.publish(cmd)
        self.led_pub.publish(led)

        self.get_logger().info(
            f"semantic={self.semantic_state} "
            f"cmd_x={cmd.linear.x:.2f} led={led.data}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ShowcaseBehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
