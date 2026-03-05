#!/usr/bin/env python3
"""
cmd_vel -> Arduino serial bridge

Subscribes:
    /cmd_vel  (geometry_msgs/Twist)

Sends over serial (ASCII):
    "vx wz\n"
Example:
    0.20 -0.50

Parameters:
    port: serial device (default placeholder)
    baudrate: default 115200
    cmd_topic: default /cmd_vel
    send_rate_hz: default 30
    timeout_s: stop sending nonzero if no cmd received
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


class CmdVelSerialBridge(Node):

    def __init__(self):
        super().__init__("cmdvel_serial_bridge")

        # -------- Parameters --------
        self.declare_parameter("port", "/dev/PORT_NAME_HERE")  # change later (we kept changing the port) 
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("send_rate_hz", 30.0)
        self.declare_parameter("timeout_s", 0.5)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.send_rate = float(self.get_parameter("send_rate_hz").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)

        # -------- Serial connection --------
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2.0)  # allow Arduino reset
            self.get_logger().info(f"Opened serial port {self.port} @ {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        # -------- ROS setup --------
        self.latest_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 10)

        self.timer = self.create_timer(1.0 / self.send_rate, self.timer_callback)

    def cmd_callback(self, msg: Twist):
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9

        vx = self.latest_cmd.linear.x
        wz = self.latest_cmd.angular.z

        # Safety timeout
        if dt > self.timeout_s:
            vx = 0.0
            wz = 0.0

        line = f"{vx:.3f} {wz:.3f}\n"

        try:
            self.ser.write(line.encode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")


def main():
    rclpy.init()
    node = CmdVelSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
