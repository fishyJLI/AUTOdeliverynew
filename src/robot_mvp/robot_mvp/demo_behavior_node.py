import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math


class DriftBehaviorNode(Node):
    def __init__(self):
        super().__init__('drift_behavior_node')

        # Subscribers
        self.create_subscription(String, '/semantic_state', self.semantic_callback, 10)
        self.create_subscription(Imu, '/imu_data_ros', self.imu_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State
        self.semantic_state = "CLEAR"
        self.current_yaw = 0.0
        self.target_yaw = None

        # Gains
        self.k_heading = 1.5
        self.avoid_turn = 0.6

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Drift Behavior Node started")

    def semantic_callback(self, msg):
        self.semantic_state = msg.data

    def imu_callback(self, msg):
        # Convert quaternion → yaw
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.target_yaw is None:
            self.target_yaw = self.current_yaw

    def control_loop(self):
        cmd = Twist()

        if self.target_yaw is None:
            return

        # Heading correction
        error = self.target_yaw - self.current_yaw

        # normalize angle
        error = math.atan2(math.sin(error), math.cos(error))

        heading_correction = self.k_heading * error

        # =========================
        # PERSON → STOP
        # =========================
        if self.semantic_state == "PERSON":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # =========================
        # OBSTACLE → DRIFT
        # =========================
        elif self.semantic_state == "OBSTACLE":
            cmd.linear.x = 0.15
            cmd.angular.z = heading_correction + self.avoid_turn

        # =========================
        # CLEAR → STRAIGHT + CORRECT
        # =========================
        else:
            cmd.linear.x = 0.2
            cmd.angular.z = heading_correction

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"Yaw: {self.current_yaw:.2f} | Target: {self.target_yaw:.2f} | State: {self.semantic_state}"
        )