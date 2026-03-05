#!/usr/bin/env python3
"""
Depth-only "follow a person" (no ML) MVP.

Idea:
- Look at a central vertical band (ROI) in the depth image.
- Find the nearest valid depth point in that ROI.
- Treat that nearest point as "the target" (often the closest body part in front).
- Control:
  - angular.z steers to center the target
  - linear.x keeps target at desired distance

This is NOT a real person detector. It is an MVP tracking "closest object in front".
Later can replace the target-selection part with ML bounding boxes - require Machine Learning tools

Publishes:
- /cmd_vel_follow (geometry_msgs/Twist)

Subscribes:
- depth image topic (sensor_msgs/Image), default: /camera/depth/image_raw
"""

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge


class DepthFollowPerson(Node):
    def __init__(self):
        super().__init__("follow_person_depth")

        # -------- Parameters (tune these) --------
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("cmd_vel_out", "/cmd_vel_follow")

        # Desired following distance (meters)
        self.declare_parameter("desired_distance_m", 1.2)

        # ROI: use middle band of the image, e.g. 0.3 means 30%..70% width
        self.declare_parameter("roi_half_width_ratio", 0.25)

        # Depth validity limits (meters)
        self.declare_parameter("min_depth_m", 0.35)
        self.declare_parameter("max_depth_m", 4.0)

        # Control gains
        self.declare_parameter("k_linear", 0.7)    # linear speed per meter error
        self.declare_parameter("k_angular", 1.6)   # angular speed per normalized x error

        # Speed limits
        self.declare_parameter("max_linear", 0.35)
        self.declare_parameter("max_angular", 1.2)

        # Behavior when too close: stop or back up slightly
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("reverse_max_linear", 0.10)

        # Smoothing
        self.declare_parameter("depth_median_kernel", 5)  # odd number (3/5/7)

        # If no valid target for this long, stop
        self.declare_parameter("target_timeout_s", 0.4)

        # Publish rate
        self.declare_parameter("control_rate_hz", 20.0)

        # -------- Read parameters --------
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.cmd_vel_out = str(self.get_parameter("cmd_vel_out").value)
        self.desired_distance = float(self.get_parameter("desired_distance_m").value)
        self.roi_half_width_ratio = float(self.get_parameter("roi_half_width_ratio").value)
        self.min_depth = float(self.get_parameter("min_depth_m").value)
        self.max_depth = float(self.get_parameter("max_depth_m").value)
        self.k_lin = float(self.get_parameter("k_linear").value)
        self.k_ang = float(self.get_parameter("k_angular").value)
        self.max_lin = float(self.get_parameter("max_linear").value)
        self.max_ang = float(self.get_parameter("max_angular").value)
        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)
        self.rev_max_lin = float(self.get_parameter("reverse_max_linear").value)
        self.kernel = int(self.get_parameter("depth_median_kernel").value)
        self.timeout_s = float(self.get_parameter("target_timeout_s").value)
        self.rate_hz = float(self.get_parameter("control_rate_hz").value)

        if self.kernel % 2 == 0:
            self.kernel += 1  # force odd

        # -------- ROS setup --------
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_out, 10)
        self.create_subscription(Image, self.depth_topic, self.on_depth, 10)

        self.latest_target: Optional[Tuple[float, float]] = None
        # latest_target = (target_depth_m, target_x_norm [-1..1])
        self.latest_target_time_ns: Optional[int] = None

        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

        self.get_logger().info(
            f"DepthFollowPerson ready. depth_topic={self.depth_topic} -> {self.cmd_vel_out}"
        )
        self.get_logger().info(
            "This MVP tracks the nearest object in the center ROI (not true person detection)."
        )

    def on_depth(self, msg: Image):
        """
        Convert depth image -> find nearest valid point in center ROI.

        Supports common depth encodings:
        - 16UC1: depth in millimeters
        - 32FC1: depth in meters (float)
        """
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        if depth is None or depth.size == 0:
            return

        # Convert to meters
        depth_m = self._to_meters(depth, msg.encoding)

        # Find target in ROI
        target = self._find_nearest_in_center_roi(depth_m)
        if target is None:
            return

        target_depth_m, target_x_norm = target
        self.latest_target = (target_depth_m, target_x_norm)
        self.latest_target_time_ns = self.get_clock().now().nanoseconds

    def _to_meters(self, depth: np.ndarray, encoding: str) -> np.ndarray:
        """
        Convert depth image to meters as float32.
        """
        # Common encodings: "16UC1" or "32FC1"
        if encoding.upper() == "16UC1" or depth.dtype == np.uint16:
            # millimeters -> meters
            depth_m = depth.astype(np.float32) * 0.001
        else:
            # assume already meters
            depth_m = depth.astype(np.float32)

        return depth_m

    def _find_nearest_in_center_roi(self, depth_m: np.ndarray) -> Optional[Tuple[float, float]]:
        """
        Returns (nearest_depth_m, x_norm) where x_norm is [-1..1], -1 left, +1 right.
        """
        h, w = depth_m.shape[:2]
        cx = w // 2

        half_roi = int(w * self.roi_half_width_ratio)
        x0 = max(0, cx - half_roi)
        x1 = min(w, cx + half_roi)

        roi = depth_m[:, x0:x1]  # full height, center band

        # Valid mask
        valid = np.isfinite(roi) & (roi > self.min_depth) & (roi < self.max_depth)
        if not np.any(valid):
            return None

        # Find nearest point in ROI
        roi_valid = np.where(valid, roi, np.inf)
        flat_idx = int(np.argmin(roi_valid))
        r, c = np.unravel_index(flat_idx, roi_valid.shape)
        nearest = float(roi_valid[r, c])
        if not math.isfinite(nearest) or nearest == np.inf:
            return None

        # Optional small median filter around that point (stabilizes jitter)
        k = self.kernel
        r0 = max(0, r - k // 2)
        r1 = min(roi.shape[0], r + k // 2 + 1)
        c0 = max(0, c - k // 2)
        c1 = min(roi.shape[1], c + k // 2 + 1)
        patch = roi[r0:r1, c0:c1]
        patch_valid = patch[np.isfinite(patch) & (patch > self.min_depth) & (patch < self.max_depth)]
        if patch_valid.size > 0:
            nearest = float(np.median(patch_valid))

        # Convert c (roi coords) -> image x
        x_img = x0 + c
        x_norm = (x_img - cx) / float(cx)  # roughly [-1..1]
        x_norm = float(np.clip(x_norm, -1.0, 1.0))

        return nearest, x_norm

    def control_loop(self):
        """
        Publish cmd_vel_follow based on latest target.
        Stop if target is stale.
        """
        now_ns = self.get_clock().now().nanoseconds

        # If no target or stale -> stop
        if self.latest_target is None or self.latest_target_time_ns is None:
            self._publish_stop()
            return

        age_s = (now_ns - self.latest_target_time_ns) / 1e9
        if age_s > self.timeout_s:
            self._publish_stop()
            return

        target_depth, x_norm = self.latest_target

        # Distance control: positive error means too far -> move forward
        dist_err = target_depth - self.desired_distance
        vx = self.k_lin * dist_err

        # Optional: don't reverse for MVP
        if not self.allow_reverse:
            vx = max(0.0, vx)
        else:
            vx = float(np.clip(vx, -self.rev_max_lin, self.max_lin))

        # Steering control: x_norm > 0 => target to the right => turn right (negative or positive depends on your base)
        # Standard ROS: +angular.z turns left (CCW). If target is right, we want to turn right => negative.
        wz = -self.k_ang * x_norm

        # Clamp
        vx = float(np.clip(vx, -self.rev_max_lin if self.allow_reverse else 0.0, self.max_lin))
        wz = float(np.clip(wz, -self.max_ang, self.max_ang))

        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def _publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = DepthFollowPerson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # stop on exit
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()