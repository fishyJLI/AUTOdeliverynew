import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
from ultralytics import YOLO


class PersonDetectionNode(Node):
    def __init__(self):
        super().__init__('person_detection_node')

        # Publisher
        self.publisher = self.create_publisher(
            String,
            '/person_detection',
            10
        )

        # Open webcam (NUMBER = working webcam)
        self.cap = cv2.VideoCapture(2)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
        else:
            self.get_logger().info("Camera opened successfully")

        # Load YOLO model
        self.model = YOLO('yolo11n.pt')

        # Timer to process frames (10 Hz)
        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning("Failed to read frame")
            return

        # Run detection
        results = self.model(frame, verbose=False)

        person_detected = False

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]

                if label == 'person':
                    person_detected = True

                    # Draw box
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, 'PERSON', (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Publish result
        msg = String()
        msg.data = 'PERSON' if person_detected else 'CLEAR'
        self.publisher.publish(msg)

        # Show image
        cv2.imshow("Person Detection", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()