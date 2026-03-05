import math
import csv
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(x, y, z, w):
    # yaw (Z axis) from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GPSWaypointFollower(Node):
    def __init__(self):
        super().__init__("gps_waypoint_follower")

        self.declare_parameter("waypoints_csv", "")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("loop_hz", 10.0)

        # Speeds
        self.declare_parameter("v_base", 0.45)   # base forward speed (m/s)
        self.declare_parameter("v_min", 0.15)    # never go below this unless stop (m/s)
        self.declare_parameter("w_max", 0.7)     # max turning rate (rad/s) -> gradual turn

        # Controller gains
        self.declare_parameter("kp_heading", 1.2)     # angular gain
        self.declare_parameter("slow_k", 0.8)         # how much to slow down when error is large

        self.declare_parameter("waypoint_radius_m", 2.0)
        self.declare_parameter("gps_timeout_s", 1.5)
        self.declare_parameter("imu_timeout_s", 1.0)

        self.csv_path = self.get_parameter("waypoints_csv").value
        self.gps_topic = self.get_parameter("gps_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.loop_hz = float(self.get_parameter("loop_hz").value)

        self.v_base = float(self.get_parameter("v_base").value)
        self.v_min = float(self.get_parameter("v_min").value)
        self.w_max = float(self.get_parameter("w_max").value)

        self.kp_heading = float(self.get_parameter("kp_heading").value)
        self.slow_k = float(self.get_parameter("slow_k").value)

        self.wp_radius = float(self.get_parameter("waypoint_radius_m").value)
        self.gps_timeout = float(self.get_parameter("gps_timeout_s").value)
        self.imu_timeout = float(self.get_parameter("imu_timeout_s").value)

        self.waypoints = self.load_waypoints(self.csv_path)
        self.wp_idx = 0

        self.ref_latlon = None
        self.cur_latlon = None
        self.cur_yaw = None

        self.last_gps_time = 0.0
        self.last_imu_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(NavSatFix, self.gps_topic, self.on_gps, 10)
        self.create_subscription(Imu, self.imu_topic, self.on_imu, 10)

        self.timer = self.create_timer(1.0 / self.loop_hz, self.loop)

        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from: {self.csv_path}")
        self.get_logger().info("GPS waypoint follower started (gradual turning).")

    def load_waypoints(self, path):
        if not path:
            self.get_logger().warn("waypoints_csv param is empty.")
            return []
        wps = []
        with open(path, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                wps.append((float(row["lat"]), float(row["lon"])))
        return wps

    def on_gps(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        self.cur_latlon = (msg.latitude, msg.longitude)
        self.last_gps_time = time.time()
        if self.ref_latlon is None:
            self.ref_latlon = self.cur_latlon
            self.get_logger().info(f"Set local origin: {self.ref_latlon}")

    def on_imu(self, msg: Imu):
        q = msg.orientation
        # If IMU doesn't provide orientation, values might be all 0
        if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
            return
        self.cur_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.last_imu_time = time.time()

    def ll_to_xy(self, lat, lon):
        lat0, lon0 = self.ref_latlon
        R = 6378137.0
        x = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
        y = math.radians(lat - lat0) * R
        return x, y

    def stop(self):
        self.cmd_pub.publish(Twist())

    def loop(self):
        now = time.time()

        if self.cur_latlon is None or self.ref_latlon is None:
            self.stop()
            return
        if (now - self.last_gps_time) > self.gps_timeout:
            self.stop()
            return
        if self.cur_yaw is None or (now - self.last_imu_time) > self.imu_timeout:
            # Without yaw we can't steer; stop for safety  - sort of like a kill switch
            self.stop()
            return
        if self.wp_idx >= len(self.waypoints):
            self.stop()
            return

        cur_x, cur_y = self.ll_to_xy(*self.cur_latlon)
        wp_lat, wp_lon = self.waypoints[self.wp_idx]
        wp_x, wp_y = self.ll_to_xy(wp_lat, wp_lon)

        dx = wp_x - cur_x
        dy = wp_y - cur_y
        dist = math.hypot(dx, dy)

        if dist < self.wp_radius:
            self.wp_idx += 1
            self.get_logger().info(f"Reached waypoint {self.wp_idx}/{len(self.waypoints)}")
            self.stop()
            return

        desired = math.atan2(dy, dx)
        err = wrap_pi(desired - self.cur_yaw)

        # Gradual turning: always forward (>= v_min), limit w
        w = clamp(self.kp_heading * err, -self.w_max, self.w_max)

        # Slow down as heading error grows, but do not stop 
        v = self.v_base * (1.0 - self.slow_k * min(abs(err) / math.pi, 1.0))
        v = clamp(v, self.v_min, self.v_base)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()