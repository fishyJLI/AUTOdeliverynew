import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String


class GPSReportNode(Node):
    def __init__(self):
        super().__init__('gps_report_node')

        self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )

        self.report_pub = self.create_publisher(
            String,
            '/gps_report',
            10
        )

        self.get_logger().info('GPS Report Node started')
        self.get_logger().info('Listening to /fix')

    def gps_callback(self, msg):
        report = String()

        if msg.status.status >= NavSatStatus.STATUS_FIX:
            status_text = 'GPS FIX'
        else:
            status_text = 'NO FIX'

        report.data = (
            f'{status_text} | '
            f'lat={msg.latitude:.7f}, '
            f'lon={msg.longitude:.7f}, '
            f'alt={msg.altitude:.2f}m'
        )

        self.report_pub.publish(report)
        self.get_logger().info(report.data)


def main(args=None):
    rclpy.init(args=args)
    node = GPSReportNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()