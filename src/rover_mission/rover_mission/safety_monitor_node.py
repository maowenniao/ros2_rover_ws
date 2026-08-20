#!/usr/bin/env python3
"""安全监督节点：监测 /cmd_vel_nav 与 /scan，发现异常时发布 /estop。"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')
        self.declare_parameter('max_linear', 1.5)
        self.declare_parameter('max_angular', 2.0)
        self.declare_parameter('min_front_distance', 0.30)
        self.create_subscription(Twist, '/cmd_vel_nav', self._cmd_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.estop_pub = self.create_publisher(Bool, '/estop', 10)
        self.last_cmd = Twist()
        self.min_front = float('inf')
        self.create_timer(0.1, self._evaluate)
        self.get_logger().info('安全监督节点启动')

    def _cmd_cb(self, msg):
        self.last_cmd = msg

    def _scan_cb(self, msg):
        values = []
        angle = msg.angle_min
        for distance in msg.ranges:
            if abs(angle) < 0.3 and math.isfinite(distance):
                if msg.range_min <= distance <= msg.range_max:
                    values.append(distance)
            angle += msg.angle_increment
        self.min_front = min(values, default=float('inf'))

    def _evaluate(self):
        estop = Bool()
        if abs(self.last_cmd.linear.x) > self.get_parameter('max_linear').value:
            estop.data = True
        if abs(self.last_cmd.angular.z) > self.get_parameter('max_angular').value:
            estop.data = True
        if self.min_front <= self.get_parameter('min_front_distance').value:
            estop.data = True
        if estop.data:
            self.get_logger().warn('安全条件触发：发布 /estop')
        self.estop_pub.publish(estop)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
