import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math


class LidarSafetyNode(Node):
    def __init__(self):
        super().__init__('lidar_safety_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            '/lidar_safety_state',
            10
        )

        # parameters
        self.stop_distance = 0.6  # meters
        self.front_angle = 30.0   # degrees

        self.get_logger().info("Lidar Safety Node started")

    def scan_callback(self, msg: LaserScan):
        min_distance = float('inf')

        # Convert front angle to radians
        half_angle = math.radians(self.front_angle / 2.0)

        angle = msg.angle_min

        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            # check if angle is within front window
            if -half_angle < angle < half_angle:
                if r < min_distance:
                    min_distance = r

            angle += msg.angle_increment

        state_msg = String()

        if min_distance < self.stop_distance:
            state_msg.data = "BLOCKED"
        else:
            state_msg.data = "CLEAR"

        self.publisher.publish(state_msg)

        self.get_logger().info(
            f"Front min distance: {min_distance:.2f} → {state_msg.data}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarSafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()