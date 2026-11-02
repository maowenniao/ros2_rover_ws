#!/usr/bin/env python3
"""唯一的 /wheel_cmd 发布者：/cmd_vel → 四轮角速度。

几何与限幅来自 rover_geometry.yaml。/cmd_vel 超时后持续发 0。
"""
from __future__ import annotations

from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

from odrive_can_bridge.kinematics import twist_to_wheel_rads
from rover_description.geometry import declare_geometry


class ODriveVelocityControlNode(Node):
    def __init__(self) -> None:
        super().__init__('odrive_velocity_control')
        declare_geometry(self)
        self.declare_parameter('cmdvel_timeout', 0.5)
        self.declare_parameter('publish_hz', 50.0)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.directions: List[float] = [float(v) for v in self.get_parameter('wheel_directions').value]
        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)
        self.cmdvel_timeout = float(self.get_parameter('cmdvel_timeout').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, qos)
        self._pub = self.create_publisher(Float32MultiArray, '/wheel_cmd', qos)
        self._vx = 0.0
        self._wz = 0.0
        self._last_cmd = None
        period = 1.0 / float(self.get_parameter('publish_hz').value)
        self.create_timer(period, self._publish)
        self.get_logger().info(
            f'差速运动学：radius={self.wheel_radius} m, track={self.track_width} m, '
            f'max_wz={self.max_angular_vel} rad/s, directions={self.directions}'
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._vx = float(msg.linear.x)
        self._wz = float(msg.angular.z)
        self._last_cmd = self.get_clock().now()

    def _publish(self) -> None:
        if self._last_cmd is None:
            wheels = [0.0, 0.0, 0.0, 0.0]
        else:
            age = (self.get_clock().now() - self._last_cmd).nanoseconds / 1e9
            if age > self.cmdvel_timeout:
                self._vx = 0.0
                self._wz = 0.0
            wheels = twist_to_wheel_rads(
                self._vx,
                self._wz,
                self.wheel_radius,
                self.track_width,
                self.directions,
                self.max_linear_vel,
                self.max_angular_vel,
                self.max_wheel_speed,
            )
        msg = Float32MultiArray()
        msg.data = wheels
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ODriveVelocityControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
