#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

def quat_to_yaw(qx, qy, qz, qw) -> float:
    # yaw (Z) from quaternion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_pi(a: float) -> float:
    # wrap to (-pi, pi]
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a

class Turn90(Node):
    def __init__(self):
        super().__init__("turn_90")

        # Params (tune these)
        self.declare_parameter("imu_topic", "/imu_data_ros")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("forward_speed", 0.12)      # m/s-ish (depends on your base)
        self.declare_parameter("turn_rate", 0.55)          # rad/s (tune)
        self.declare_parameter("target_deg", 90.0)
        self.declare_parameter("direction", "left")        # "left" or "right"
        self.declare_parameter("settle_time", 0.25)        # seconds to stop after turn
        self.declare_parameter("yaw_tolerance_deg", 2.0)   # stop a bit early/late tolerance

        self.imu_topic = self.get_parameter("imu_topic").value
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value

        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.turn_rate = float(self.get_parameter("turn_rate").value)
        self.target_rad = math.radians(float(self.get_parameter("target_deg").value))
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.tol = math.radians(float(self.get_parameter("yaw_tolerance_deg").value))

        direction = self.get_parameter("direction").value.strip().lower()
        self.sign = +1.0 if direction == "left" else -1.0

        self.yaw = None
        self.start_yaw = None
        self.state = "IDLE"   # IDLE -> TURNING -> SETTLE -> DONE
        self.settle_until = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(Imu, self.imu_topic, self.on_imu, 50)

        self.timer = self.create_timer(0.02, self.loop)  # 50 Hz control loop

        self.get_logger().info(
            f"Turn90 ready. imu={self.imu_topic} cmd={self.cmd_topic} "
            f"dir={'left' if self.sign>0 else 'right'} target={math.degrees(self.target_rad):.1f}deg"
        )
        self.get_logger().info("Trigger with: ros2 service call /turn90 std_srvs/srv/Trigger {}")

        # Simple trigger service
        from std_srvs.srv import Trigger
        self.srv = self.create_service(Trigger, "turn90", self.on_trigger)

    def on_trigger(self, req, resp):
        if self.yaw is None:
            resp.success = False
            resp.message = "No IMU yet"
            return resp
        if self.state in ("TURNING", "SETTLE"):
            resp.success = False
            resp.message = f"Busy: {self.state}"
            return resp

        self.start_yaw = self.yaw
        self.state = "TURNING"
        resp.success = True
        resp.message = f"Started 90-deg turn from yaw={self.start_yaw:.3f} rad"
        return resp

    def on_imu(self, msg: Imu):
        q = msg.orientation
        self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def publish_cmd(self, vx: float, wz: float):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def loop(self):
        if self.yaw is None:
            return

        if self.state == "IDLE":
            return

        if self.state == "TURNING":
            # yaw error is how far we've rotated since start (signed)
            dyaw = wrap_pi(self.yaw - self.start_yaw) * self.sign
            remaining = self.target_rad - dyaw

            # Stop condition
            if remaining <= self.tol:
                self.publish_cmd(0.0, 0.0)
                self.state = "SETTLE"
                self.settle_until = self.get_clock().now().nanoseconds + int(self.settle_time * 1e9)
                self.get_logger().info(f"Reached target. dyaw={math.degrees(dyaw):.1f} deg. Settling...")
                return

            # Simple control: constant forward + constant turn rate
            self.publish_cmd(self.forward_speed, self.sign * self.turn_rate)
            return

        if self.state == "SETTLE":
            now_ns = self.get_clock().now().nanoseconds
            if now_ns >= self.settle_until:
                self.state = "DONE"
                self.get_logger().info("Turn complete ✅")
            return

        if self.state == "DONE":
            # keep stopped
            self.publish_cmd(0.0, 0.0)
            self.state = "IDLE"

def main():
    rclpy.init()
    node = Turn90()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # stop robot on exit
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()