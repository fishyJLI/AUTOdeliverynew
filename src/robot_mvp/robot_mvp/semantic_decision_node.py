import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SemanticDecisionNode(Node):
    def __init__(self):
        super().__init__('semantic_decision_node')

        self.person_state = "CLEAR"
        self.lidar_state = "CLEAR"

        # Subscribers
        self.create_subscription(
            String,
            '/person_detection',
            self.person_callback,
            10
        )

        self.create_subscription(
            String,
            '/lidar_safety_state',
            self.lidar_callback,
            10
        )

        # Publisher
        self.publisher = self.create_publisher(
            String,
            '/semantic_state',
            10
        )

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_state)

        self.get_logger().info("Semantic Decision Node started")

    def person_callback(self, msg):
        self.person_state = msg.data

    def lidar_callback(self, msg):
        self.lidar_state = msg.data

    def publish_state(self):
        state_msg = String()

        # Priority logic
        if self.person_state == "PERSON":
            state_msg.data = "PERSON"
        elif self.lidar_state == "BLOCKED":
            state_msg.data = "OBSTACLE"
        else:
            state_msg.data = "CLEAR"

        self.publisher.publish(state_msg)

        self.get_logger().info(
            f"Semantic: {state_msg.data} "
            f"(person={self.person_state}, lidar={self.lidar_state})"
        )



def main(args=None):
    rclpy.init(args=args)
    node = SemanticDecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()