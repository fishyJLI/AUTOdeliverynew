#!/usr/bin/env python3
"""
Mode + cmd_vel mux (no external twist_mux needed)

Goal:
- Keep ONE final motor topic: /cmd_vel
- Allow easy switching between behaviors:
    - delivery publishes to /cmd_vel_delivery
    - follow publishes to /cmd_vel_follow
- This node forwards the active one to /cmd_vel

Switch via service:
- /set_follow_mode (std_srvs/SetBool)
    True  -> FOLLOW
    False -> DELIVERY

Safety:
- On mode change, publishes a brief stop so the robot doesn't "jump".
- If active input command becomes stale, outputs stop.
"""

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


@dataclass
class TwistStampedLocal:
    msg: Twist
    t_ns: int


class ModeCmdVelMux(Node):
    def __init__(self):
        super().__init__("mode_cmdvel_mux")

        # -------- Parameters --------
        self.declare_parameter("cmd_in_delivery", "/cmd_vel_delivery")
        self.declare_parameter("cmd_in_follow", "/cmd_vel_follow")
        self.declare_parameter("cmd_out", "/cmd_vel")

        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("input_timeout_s", 0.5)

        # Start mode (delivery by default)
        self.declare_parameter("start_in_follow", False)

        # -------- Read parameters --------
        self.in_delivery = str(self.get_parameter("cmd_in_delivery").value)
        self.in_follow = str(self.get_parameter("cmd_in_follow").value)
        self.cmd_out = str(self.get_parameter("cmd_out").value)

        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timeout_s = float(self.get_parameter("input_timeout_s").value)
        self.follow_mode = bool(self.get_parameter("start_in_follow").value)

        # -------- ROS setup --------
        self.pub = self.create_publisher(Twist, self.cmd_out, 10)

        self.latest_delivery: Optional[TwistStampedLocal] = None
        self.latest_follow: Optional[TwistStampedLocal] = None

        self.create_subscription(Twist, self.in_delivery, self.on_delivery, 10)
        self.create_subscription(Twist, self.in_follow, self.on_follow, 10)

        self.srv = self.create_service(SetBool, "set_follow_mode", self.on_set_follow_mode)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.loop)

        self.get_logger().info(
            f"ModeCmdVelMux ready. DELIVERY:{self.in_delivery} FOLLOW:{self.in_follow} -> OUT:{self.cmd_out}"
        )
        self.get_logger().info("Switch with: ros2 service call /set_follow_mode std_srvs/srv/SetBool \"{data: true}\"")

    def on_delivery(self, msg: Twist):
        self.latest_delivery = TwistStampedLocal(msg=msg, t_ns=self.get_clock().now().nanoseconds)

    def on_follow(self, msg: Twist):
        self.latest_follow = TwistStampedLocal(msg=msg, t_ns=self.get_clock().now().nanoseconds)

    def on_set_follow_mode(self, req: SetBool.Request, resp: SetBool.Response):
        # Safety: stop before switching
        self._publish_stop()

        self.follow_mode = bool(req.data)
        resp.success = True
        resp.message = "Mode set to FOLLOW" if self.follow_mode else "Mode set to DELIVERY"
        self.get_logger().info(resp.message)
        return resp

    def loop(self):
        now_ns = self.get_clock().now().nanoseconds

        src = self.latest_follow if self.follow_mode else self.latest_delivery
        if src is None:
            self._publish_stop()
            return

        age_s = (now_ns - src.t_ns) / 1e9
        if age_s > self.timeout_s:
            # command too old -> stop
            self._publish_stop()
            return

        self.pub.publish(src.msg)

    def _publish_stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.pub.publish(t)


def main():
    rclpy.init()
    node = ModeCmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()