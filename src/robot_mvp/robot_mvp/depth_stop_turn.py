import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import numpy as np
from cv_bridge import CvBridge


class DepthStopTurn(Node):
    """
    Simple obstacle avoider:
    - looks at a center ROI in depth image
    - if something is closer than min_dist_m:
        publish slow forward + turn (drift)
      else:
        publish nothing (so mux uses planned cmd_vel)
    """
    def __init__(self):
        super().__init__('depth_stop_turn')

        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('out_topic', '/cmd_vel_safe')

        self.declare_parameter('min_dist_m', 0.9)     # trigger distance
        self.declare_parameter('slow_v', 0.15)        # forward speed when avoiding
        self.declare_parameter('turn_w', 0.5)         # turn rate when avoiding
        self.declare_parameter('roi_w_frac', 0.35)    # width of center ROI (fraction)
        self.declare_parameter('roi_h_frac', 0.35)    # height of center ROI (fraction)

        self.declare_parameter('prefer_turn', 1)      # 1=left, -1=right

        self.depth_topic = self.get_parameter('depth_topic').value
        self.out_topic = self.get_parameter('out_topic').value

        self.min_dist_m = float(self.get_parameter('min_dist_m').value)
        self.slow_v = float(self.get_parameter('slow_v').value)
        self.turn_w = float(self.get_parameter('turn_w').value)
        self.roi_w_frac = float(self.get_parameter('roi_w_frac').value)
        self.roi_h_frac = float(self.get_parameter('roi_h_frac').value)
        self.prefer_turn = int(self.get_parameter('prefer_turn').value)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.sub = self.create_subscription(Image, self.depth_topic, self.on_depth, 10)

        self.get_logger().info(f"Depth avoider listening: {self.depth_topic} -> publishing {self.out_topic}")

    def on_depth(self, msg: Image):
        # Depth image is usually 16UC1 (mm) or 32FC1 (m)
        cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        arr = np.array(cv)

        h, w = arr.shape[:2]
        rw = int(w * self.roi_w_frac)
        rh = int(h * self.roi_h_frac)
        x0 = (w - rw) // 2
        y0 = (h - rh) // 2

        roi = arr[y0:y0+rh, x0:x0+rw]

        # Convert to meters + ignore invalid zeros
        if msg.encoding in ('16UC1', 'mono16'):
            roi_m = roi.astype(np.float32) / 1000.0
        else:
            roi_m = roi.astype(np.float32)

        valid = roi_m[(roi_m > 0.05) & (roi_m < 20.0)]
        if valid.size == 0:
            # nothing valid -> don't override
            self.pub.publish(Twist())
            return

        nearest = float(np.percentile(valid, 10))  # robust "near" estimate

        cmd = Twist()
        if nearest < self.min_dist_m:
            cmd.linear.x = self.slow_v
            cmd.angular.z = self.turn_w * float(self.prefer_turn)
        # else publish zero cmd (means "no override")
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DepthStopTurn()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()