#!/usr/bin/env python3
"""速度安全层：/cmd_vel_nav + /scan → /cmd_vel。

无新指令超过 cmd_timeout，或要求扫描但扫描超时，输出全 0。
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from diffdrive_rover.safety import front_min_range, limit_twist


class SafetyCmdVelNode(Node):
    def __init__(self) -> None:
        super().__init__('safety_cmdvel_node')
        self.declare_parameter('stop_distance', 0.45)
        self.declare_parameter('slow_distance', 1.20)
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('scan_sector_deg', 35.0)
        self.declare_parameter('require_scan', True)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.sector = math.radians(float(self.get_parameter('scan_sector_deg').value))
        self.require_scan = bool(self.get_parameter('require_scan').value)
        self.last_cmd = Twist()
        self.min_front = float('inf')
        self.last_scan = None
        self.last_cmd_time = None
        self.create_subscription(Twist, 'cmd_vel_nav', self._cmd_cb, 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.05, self._publish)
        self.get_logger().info(
            f'安全速度层已启动：前方 {self.stop_distance:.2f} m 停车，'
            f'指令超时 {self.cmd_timeout:.2f} s'
        )

    def _cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _scan_cb(self, msg: LaserScan) -> None:
        self.min_front = front_min_range(msg, self.sector)
        self.last_scan = self.get_clock().now()

    def _age(self, stamp) -> float | None:
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _publish(self) -> None:
        out = limit_twist(
            self.last_cmd,
            self._age(self.last_cmd_time),
            self.cmd_timeout,
            self._age(self.last_scan),
            self.scan_timeout,
            self.require_scan,
            self.min_front,
            self.stop_distance,
            self.slow_distance,
        )
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyCmdVelNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
