import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def is_nonzero(cmd: Twist, eps=1e-3) -> bool:
    return abs(cmd.linear.x) > eps or abs(cmd.angular.z) > eps


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        self.declare_parameter('plan_topic', '/cmd_vel_plan')
        self.declare_parameter('safe_topic', '/cmd_vel_safe')
        self.declare_parameter('out_topic', '/cmd_vel')
        self.declare_parameter('safe_timeout_s', 0.25)  # how long safe cmd stays valid

        self.plan_topic = self.get_parameter('plan_topic').value
        self.safe_topic = self.get_parameter('safe_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.safe_timeout = float(self.get_parameter('safe_timeout_s').value)

        self.plan_cmd = Twist()
        self.safe_cmd = Twist()
        self.safe_time = 0.0

        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.create_subscription(Twist, self.plan_topic, self.on_plan, 10)
        self.create_subscription(Twist, self.safe_topic, self.on_safe, 10)

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz
        self.get_logger().info(f"Mux: plan={self.plan_topic}, safe={self.safe_topic} -> out={self.out_topic}")

    def on_plan(self, msg: Twist):
        self.plan_cmd = msg

    def on_safe(self, msg: Twist):
        self.safe_cmd = msg
        if is_nonzero(msg):
            self.safe_time = time.time()

    def loop(self):
        now = time.time()
        use_safe = (now - self.safe_time) < self.safe_timeout and is_nonzero(self.safe_cmd)
        self.pub.publish(self.safe_cmd if use_safe else self.plan_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()