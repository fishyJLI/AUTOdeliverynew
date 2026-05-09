import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LidarSafetyNode(Node):
    def __init__(self):
        super().__init__('lidar_safety_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(
            String,
            '/lidar_safety_state',
            10
        )

        self.stop_distance = 1.20
        self.front_angle_deg = 30.0

        self.get_logger().info('Lidar Safety Node started')

    def scan_callback(self, msg: LaserScan):
        self.get_logger().info('Received scan')

        min_distance = float('inf')
        half_angle = math.radians(self.front_angle_deg / 2.0)

        angle = msg.angle_min

        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            if -half_angle <= angle <= half_angle:
                if r < min_distance:
                    min_distance = r

            angle += msg.angle_increment

        state_msg = String()

        if min_distance < self.stop_distance:
            state_msg.data = 'BLOCKED'
        else:
            state_msg.data = 'CLEAR'

        self.publisher.publish(state_msg)

        self.get_logger().info(
            f'Front min distance: {min_distance:.2f} -> {state_msg.data}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarSafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()