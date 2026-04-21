import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class DemoBehaviorNode(Node):
    def __init__(self):
        super().__init__('demo_behavior_node')

        self.subscription = self.create_subscription(
            String,
            '/lidar_safety_state',
            self.state_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_demo',
            10
        )

        self.forward_speed = 0.10
        self.turn_speed = 0.30

        self.get_logger().info('Demo Behavior Node started')

    def state_callback(self, msg: String):
        cmd = Twist()

        if msg.data == 'BLOCKED':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('BLOCKED -> STOP')

        elif msg.data == 'CLEAR':
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
            self.get_logger().info('CLEAR -> MOVE FORWARD')

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().warn(f'Unknown state: {msg.data} -> STOP')

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DemoBehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()