#!/usr/bin/env python3
"""速度安全层：在导航/遥控指令进入底盘前，根据 LaserScan 限速或停车。

输入:  /cmd_vel_nav (geometry_msgs/Twist), /scan (sensor_msgs/LaserScan)
输出:  /cmd_vel (geometry_msgs/Twist)
无有效扫描、扫描超时或前方距离低于 stop_distance 时默认停车。
"""
import math
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.node import Node


class SafetyCmdVelNode(Node):
    def __init__(self):
        super().__init__('safety_cmdvel_node')
        self.declare_parameter('stop_distance', 0.45)
        self.declare_parameter('slow_distance', 1.20)
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('scan_sector_deg', 35.0)
        self.declare_parameter('require_scan', True)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.sector = math.radians(float(self.get_parameter('scan_sector_deg').value))
        self.require_scan = bool(self.get_parameter('require_scan').value)
        self.last_cmd = Twist()
        self.min_front = float('inf')
        self.last_scan = None
        self.create_subscription(Twist, 'cmd_vel_nav', self._cmd_cb, 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.05, self._publish)
        self.get_logger().info('安全速度层已启动：等待 /scan，前方 %.2fm 停车', self.stop_distance)

    def _cmd_cb(self, msg):
        self.last_cmd = msg

    def _scan_cb(self, msg):
        values = []
        angle = msg.angle_min
        for distance in msg.ranges:
            if abs(angle) <= self.sector and math.isfinite(distance):
                if msg.range_min <= distance <= msg.range_max:
                    values.append(distance)
            angle += msg.angle_increment
        self.min_front = min(values, default=float('inf'))
        self.last_scan = self.get_clock().now()

    def _publish(self):
        out = Twist()
        scan_age = float('inf') if self.last_scan is None else (self.get_clock().now() - self.last_scan).nanoseconds / 1e9
        if self.require_scan and scan_age > self.scan_timeout:
            self.pub.publish(out)
            return
        factor = 1.0
        if self.min_front <= self.stop_distance:
            factor = 0.0
        elif self.min_front < self.slow_distance:
            factor = max(0.0, (self.min_front - self.stop_distance) / (self.slow_distance - self.stop_distance))
        out.linear.x = self.last_cmd.linear.x * factor
        out.linear.y = self.last_cmd.linear.y * factor
        out.angular.z = self.last_cmd.angular.z * (0.25 + 0.75 * factor) if factor else 0.0
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyCmdVelNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
